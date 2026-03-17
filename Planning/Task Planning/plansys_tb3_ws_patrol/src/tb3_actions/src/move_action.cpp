#include <memory>
#include <string>
#include <cmath>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "rclcpp_lifecycle/lifecycle_publisher.hpp"
#include "lifecycle_msgs/msg/transition.hpp"
#include "plansys2_executor/ActionExecutorClient.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"

using namespace std::chrono_literals;

class MoveAction : public plansys2::ActionExecutorClient
{
public:
  MoveAction()
  : plansys2::ActionExecutorClient("move", 500ms),
    motion_started_(false),
    distance_traveled_(0.0),
    start_x_(0.0),
    start_y_(0.0),
    current_x_(0.0),
    current_y_(0.0),
    target_distance_(1.5)
  {
  }

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_activate(const rclcpp_lifecycle::State & previous_state)
  {
    cmd_vel_pub_ = create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
    cmd_vel_pub_->on_activate();

    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      "/odom", 10,
      [this](nav_msgs::msg::Odometry::SharedPtr msg) {
        current_x_ = msg->pose.pose.position.x;
        current_y_ = msg->pose.pose.position.y;
      });

    motion_started_ = false;
    distance_traveled_ = 0.0;

    return ActionExecutorClient::on_activate(previous_state);
  }

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_deactivate(const rclcpp_lifecycle::State & previous_state)
  {
    if (cmd_vel_pub_) {
      geometry_msgs::msg::Twist stop;
      cmd_vel_pub_->publish(stop);
      cmd_vel_pub_->on_deactivate();
    }
    odom_sub_.reset();
    return ActionExecutorClient::on_deactivate(previous_state);
  }

  void do_work() override
  {
    auto args = get_arguments();
    if (args.size() < 3) {
      finish(false, 0.0, "Invalid arguments");
      return;
    }

    if (!motion_started_) {
      start_x_ = current_x_;
      start_y_ = current_y_;
      motion_started_ = true;
      RCLCPP_INFO(get_logger(), "Moving [%s] from [%s] to [%s]",
        args[0].c_str(), args[1].c_str(), args[2].c_str());
    }

    double dx = current_x_ - start_x_;
    double dy = current_y_ - start_y_;
    distance_traveled_ = std::sqrt(dx * dx + dy * dy);

    double progress = std::min(1.0, distance_traveled_ / target_distance_);
    send_feedback(static_cast<float>(progress), "Moving...");

    RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000,
      "Distance traveled: %.2f / %.2f m", distance_traveled_, target_distance_);

    if (distance_traveled_ >= target_distance_) {
      geometry_msgs::msg::Twist stop;
      cmd_vel_pub_->publish(stop);
      motion_started_ = false;
      distance_traveled_ = 0.0;
      RCLCPP_INFO(get_logger(), "Reached [%s]", args[2].c_str());
      finish(true, 1.0, "Move completed");
      return;
    }

    geometry_msgs::msg::Twist vel;
    vel.linear.x = 0.2;
    vel.angular.z = 0.0;
    cmd_vel_pub_->publish(vel);
  }

private:
  // rclcpp_lifecycle::LifecyclePublisher — NOT rclcpp::LifecyclePublisher
  rclcpp_lifecycle::LifecyclePublisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;

  bool motion_started_;
  double distance_traveled_;
  double start_x_;
  double start_y_;
  double current_x_;
  double current_y_;
  double target_distance_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<MoveAction>();

  // Trigger CONFIGURE after spin starts (CascadeLifecycleNode needs executor running first).
  // Shared ptr trick lets the lambda cancel itself after the first fire.
  // "action_name" is the parameter ActionExecutorClient uses in should_execute()
  // to match incoming /actions_hub requests to this performer.
  node->set_parameter(rclcpp::Parameter("action_name", "move"));

  auto config_timer = std::make_shared<rclcpp::TimerBase *>(nullptr);
  auto t = node->create_wall_timer(
    std::chrono::milliseconds(500),
    [&node, config_timer]() {
      node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);
      if (*config_timer) {
        (*config_timer)->cancel();
      }
    });
  *config_timer = t.get();

  rclcpp::spin(node->get_node_base_interface());

  rclcpp::shutdown();
  return 0;
}
