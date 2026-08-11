// Reactive exploration driver: keeps each UGV moving so the SLAM front end has
// something to map without a human at a teleop keyboard.
//
// This is intentionally a placeholder for the deliberative layer. The project's
// planner decides *where* an agent should go from what the fleet knows and does
// not know; until that exists, a laser-driven wander with a wall-following bias
// gives repeatable coverage of the warehouse. Everything it needs is the scan,
// so it works identically for every robot in the fleet.

#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>
#include <memory>
#include <random>
#include <string>

#include <geometry_msgs/msg/twist.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <std_msgs/msg/bool.hpp>

namespace mrs_coordination
{

class ExplorerNode : public rclcpp::Node
{
public:
  ExplorerNode()
  : rclcpp::Node("explorer"), rng_(std::random_device{}())
  {
    declare_parameter<std::string>("robot_name", "robot1");
    declare_parameter<double>("linear_speed", 0.25);
    // Kept modest on purpose: fast in-place rotation is what most often breaks
    // correlative scan matching at a 10 Hz scan rate.
    declare_parameter<double>("angular_speed", 0.7);
    declare_parameter<double>("front_clearance", 0.65);
    declare_parameter<double>("side_clearance", 0.45);
    declare_parameter<double>("front_half_angle", 0.45);   // rad, ~26 deg
    declare_parameter<double>("control_rate", 20.0);

    robot_name_ = get_parameter("robot_name").as_string();
    linear_speed_ = get_parameter("linear_speed").as_double();
    angular_speed_ = get_parameter("angular_speed").as_double();
    front_clearance_ = get_parameter("front_clearance").as_double();
    side_clearance_ = get_parameter("side_clearance").as_double();
    front_half_angle_ = get_parameter("front_half_angle").as_double();
    const double rate = get_parameter("control_rate").as_double();

    // Relative names: the node already runs inside the robot's namespace.
    cmd_pub_ = create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);

    scan_sub_ = create_subscription<sensor_msgs::msg::LaserScan>(
      "scan", rclcpp::SensorDataQoS(),
      [this](sensor_msgs::msg::LaserScan::SharedPtr msg) {scan_ = msg;});

    enable_sub_ = create_subscription<std_msgs::msg::Bool>(
      "explore_enabled", 10,
      [this](std_msgs::msg::Bool::SharedPtr msg) {enabled_ = msg->data;});

    timer_ = create_wall_timer(
      std::chrono::duration<double>(1.0 / std::max(rate, 1.0)),
      std::bind(&ExplorerNode::step, this));

    RCLCPP_INFO(get_logger(), "explorer active for %s", robot_name_.c_str());
  }

private:
  // Smallest valid range within [center - half_width, center + half_width].
  // Returns +inf when the sector holds no valid return, which reads naturally
  // as "nothing in the way".
  double sectorMin(double center, double half_width) const
  {
    const auto & scan = *scan_;
    double best = std::numeric_limits<double>::infinity();

    for (size_t i = 0; i < scan.ranges.size(); ++i) {
      const double angle = scan.angle_min + static_cast<double>(i) * scan.angle_increment;
      double delta = std::remainder(angle - center, 2.0 * M_PI);
      if (std::abs(delta) > half_width) {
        continue;
      }
      const double r = scan.ranges[i];
      if (!std::isfinite(r) || r < scan.range_min || r > scan.range_max) {
        continue;
      }
      best = std::min(best, static_cast<double>(r));
    }
    return best;
  }

  void step()
  {
    geometry_msgs::msg::Twist cmd;

    if (!enabled_ || !scan_ || scan_->ranges.empty()) {
      cmd_pub_->publish(cmd);
      return;
    }

    const double front = sectorMin(0.0, front_half_angle_);
    const double left = sectorMin(M_PI / 2.0, 0.6);
    const double right = sectorMin(-M_PI / 2.0, 0.6);

    if (turn_ticks_ > 0) {
      // Committed to a turn: rotating for a fixed number of ticks avoids the
      // left/right dithering a purely instantaneous rule produces in corners.
      --turn_ticks_;
      cmd.angular.z = turn_direction_ * angular_speed_;
      cmd_pub_->publish(cmd);
      return;
    }

    if (front < front_clearance_) {
      turn_direction_ = (left > right) ? 1.0 : -1.0;

      // Dead end: both sides blocked too, so commit to a near half turn.
      const bool dead_end = left < side_clearance_ && right < side_clearance_;
      std::uniform_int_distribution<int> spread(dead_end ? 25 : 8, dead_end ? 45 : 22);
      turn_ticks_ = spread(rng_);

      cmd.angular.z = turn_direction_ * angular_speed_;
      cmd_pub_->publish(cmd);
      return;
    }

    // Clear ahead: creep away from whichever wall is closer. This keeps the
    // robot off the shelves without needing a costmap.
    cmd.linear.x = linear_speed_;
    if (left < side_clearance_ || right < side_clearance_) {
      cmd.angular.z = (left < right ? -1.0 : 1.0) * 0.5 * angular_speed_;
      cmd.linear.x *= 0.6;
    }
    cmd_pub_->publish(cmd);
  }

  std::string robot_name_;
  double linear_speed_{0.25};
  double angular_speed_{0.9};
  double front_clearance_{0.65};
  double side_clearance_{0.45};
  double front_half_angle_{0.45};

  bool enabled_{true};
  int turn_ticks_{0};
  double turn_direction_{1.0};
  std::mt19937 rng_;

  sensor_msgs::msg::LaserScan::SharedPtr scan_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr enable_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace mrs_coordination

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<mrs_coordination::ExplorerNode>());
  rclcpp::shutdown();
  return 0;
}
