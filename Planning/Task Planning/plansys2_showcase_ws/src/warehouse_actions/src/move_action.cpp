// The `move` action performer: the only place where a symbol becomes a metre.
//
// PlanSys2 dispatches `(move r1 c_e1 bay_e1)` and this node has to make it
// true of the world. The waypoint names mean nothing to the vehicle, so the
// node carries the same `waypoints.yaml` the problem generator used, looks the
// destination up in it, and drives there.
//
// The controller is deliberately the simplest thing that respects the map: turn
// towards the target, drive at cruise speed, stop inside the goal tolerance,
// under a laser safety stop. The roadmap guarantees the straight segment
// between two connected waypoints is free at the vehicle's own clearance, which
// is what makes a controller this simple legitimate — a planner that emitted
// edges the map does not support would need a local planner to rescue it, and
// the rescue would hide the planning failure.
//
// One performer per robot per action, each specialised on its own robot
// through PlanSys2's `specialized_arguments` parameter, so that a dispatch for
// `r2` is never bid on by `r1`'s performer.

#include <cmath>
#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <plansys2_executor/ActionExecutorClient.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <tf2/utils.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <yaml-cpp/yaml.h>

namespace warehouse_actions
{

struct Waypoint
{
  double x{0.0};
  double y{0.0};
};

double normalizeAngle(double a)
{
  while (a > M_PI) {a -= 2.0 * M_PI;}
  while (a < -M_PI) {a += 2.0 * M_PI;}
  return a;
}

class MoveAction : public plansys2::ActionExecutorClient
{
public:
  MoveAction()
  : plansys2::ActionExecutorClient("move_action", std::chrono::milliseconds(100))
  {
    declare_parameter<std::string>("robot_name", "r1");
    declare_parameter<std::string>("roadmap", "");
    declare_parameter<double>("cruise_speed", 0.22);
    declare_parameter<double>("turn_speed", 0.8);
    declare_parameter<double>("goal_tolerance", 0.25);
    declare_parameter<double>("front_clearance", 0.35);
    // A move that cannot finish must not hold the whole plan hostage: the
    // executor is told the action failed, which is information, where a
    // performer that never returns is not.
    declare_parameter<double>("timeout_factor", 4.0);
  }

  CallbackReturnT on_configure(const rclcpp_lifecycle::State & state) override
  {
    robot_name_ = get_parameter("robot_name").as_string();
    cruise_speed_ = get_parameter("cruise_speed").as_double();
    turn_speed_ = get_parameter("turn_speed").as_double();
    goal_tolerance_ = get_parameter("goal_tolerance").as_double();
    front_clearance_ = get_parameter("front_clearance").as_double();
    timeout_factor_ = get_parameter("timeout_factor").as_double();
    loadRoadmap(get_parameter("roadmap").as_string());

    cmd_pub_ = create_publisher<geometry_msgs::msg::Twist>(
      "/" + robot_name_ + "/cmd_vel", 10);
    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      "/" + robot_name_ + "/odom", 10,
      [this](nav_msgs::msg::Odometry::SharedPtr msg) {odom_ = msg;});
    scan_sub_ = create_subscription<sensor_msgs::msg::LaserScan>(
      "/" + robot_name_ + "/scan", rclcpp::SensorDataQoS(),
      [this](sensor_msgs::msg::LaserScan::SharedPtr msg) {scan_ = msg;});

    return plansys2::ActionExecutorClient::on_configure(state);
  }

  CallbackReturnT on_activate(const rclcpp_lifecycle::State & state)
  {
    cmd_pub_->on_activate();
    const auto & args = get_arguments();          // r, from, to
    if (args.size() < 3 || !waypoints_.count(args[2])) {
      RCLCPP_ERROR(get_logger(), "move dispatched to unknown waypoint");
      return plansys2::ActionExecutorClient::on_activate(state);
    }
    goal_ = waypoints_.at(args[2]);
    start_ = now();
    start_distance_ = -1.0;
    RCLCPP_INFO(
      get_logger(), "%s: move %s -> %s (%.2f, %.2f)", robot_name_.c_str(),
      args[1].c_str(), args[2].c_str(), goal_.x, goal_.y);
    return plansys2::ActionExecutorClient::on_activate(state);
  }

  CallbackReturnT on_deactivate(const rclcpp_lifecycle::State & state)
  {
    stop();
    cmd_pub_->on_deactivate();
    return plansys2::ActionExecutorClient::on_deactivate(state);
  }

private:
  void loadRoadmap(const std::string & path)
  {
    if (path.empty()) {
      RCLCPP_FATAL(get_logger(), "no roadmap given; the performer cannot place a symbol");
      return;
    }
    const auto root = YAML::LoadFile(path);
    for (const auto & entry : root["waypoints"]) {
      waypoints_[entry.first.as<std::string>()] =
        Waypoint{entry.second["x"].as<double>(), entry.second["y"].as<double>()};
    }
    RCLCPP_INFO(get_logger(), "roadmap: %zu waypoints", waypoints_.size());
  }

  void do_work() override
  {
    if (!odom_) {
      return;
    }
    const double x = odom_->pose.pose.position.x;
    const double y = odom_->pose.pose.position.y;
    const double yaw = tf2::getYaw(odom_->pose.pose.orientation);

    const double dx = goal_.x - x;
    const double dy = goal_.y - y;
    const double distance = std::hypot(dx, dy);
    if (start_distance_ < 0.0) {
      start_distance_ = std::max(distance, 1e-3);
      // The planner's own duration for this edge is the yardstick for how long
      // the drive may take before it is called a failure.
      deadline_ = start_ + rclcpp::Duration::from_seconds(
        timeout_factor_ * (start_distance_ / std::max(cruise_speed_, 1e-3) + 10.0));
    }

    if (distance < goal_tolerance_) {
      stop();
      finish(true, 1.0, "arrived");
      return;
    }
    if (now() > deadline_) {
      stop();
      RCLCPP_ERROR(get_logger(), "%s: move timed out %.2f m short", robot_name_.c_str(), distance);
      finish(false, 1.0 - distance / start_distance_, "timeout");
      return;
    }

    const double error = normalizeAngle(std::atan2(dy, dx) - yaw);
    geometry_msgs::msg::Twist cmd;
    if (std::fabs(error) > 0.35) {
      // Turn in place first: an arc at this heading error leaves the segment
      // the roadmap proved free, which is the one thing the controller must not
      // do.
      cmd.angular.z = std::copysign(turn_speed_, error);
    } else {
      cmd.linear.x = cruise_speed_ * std::max(0.3, 1.0 - std::fabs(error) / 0.35);
      cmd.angular.z = std::clamp(1.5 * error, -turn_speed_, turn_speed_);
    }
    if (frontBlocked()) {
      // Another robot, almost always: the map has no moving obstacles. Waiting
      // is the right response — the plan's ordering, not this controller, is
      // what resolves the conflict.
      cmd.linear.x = 0.0;
      cmd.angular.z = 0.0;
    }
    cmd_pub_->publish(cmd);

    send_feedback(
      std::clamp(1.0 - distance / start_distance_, 0.0, 0.99),
      "driving");
  }

  bool frontBlocked() const
  {
    if (!scan_) {
      return false;
    }
    for (size_t i = 0; i < scan_->ranges.size(); ++i) {
      const double angle = normalizeAngle(scan_->angle_min + i * scan_->angle_increment);
      if (std::fabs(angle) > 0.35) {
        continue;
      }
      const double range = scan_->ranges[i];
      if (std::isfinite(range) && range >= scan_->range_min && range < front_clearance_) {
        return true;
      }
    }
    return false;
  }

  void stop()
  {
    if (cmd_pub_->is_activated()) {
      cmd_pub_->publish(geometry_msgs::msg::Twist{});
    }
  }

  std::string robot_name_;
  std::unordered_map<std::string, Waypoint> waypoints_;
  Waypoint goal_;
  double cruise_speed_{0.22};
  double turn_speed_{0.8};
  double goal_tolerance_{0.25};
  double front_clearance_{0.35};
  double timeout_factor_{4.0};
  double start_distance_{-1.0};
  rclcpp::Time start_;
  rclcpp::Time deadline_;

  nav_msgs::msg::Odometry::SharedPtr odom_;
  sensor_msgs::msg::LaserScan::SharedPtr scan_;
  rclcpp_lifecycle::LifecyclePublisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
};

}  // namespace warehouse_actions

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<warehouse_actions::MoveAction>();

  // `action_name` is what the executor matches on, and `specialized_arguments`
  // is what keeps one robot's performer from bidding on another robot's
  // dispatch. Both are set from the launch file; the defaults here only make
  // the node runnable on its own.
  node->set_parameter(rclcpp::Parameter("action_name", "move"));

  // CascadeLifecycleNode::on_configure creates publishers and subscriptions,
  // which needs a spinning executor, so the transition is deferred by one tick.
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node->get_node_base_interface());
  auto timer = node->create_wall_timer(
    std::chrono::milliseconds(200),
    [node]() {
      static bool configured = false;
      if (!configured) {
        configured = true;
        node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);
      }
    });
  executor.spin();
  rclcpp::shutdown();
  return 0;
}
