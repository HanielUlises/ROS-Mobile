// Models the restricted connectivity the project targets.
//
// Publishes a latched /<robot>/comms_ok flag per robot. Consumers (currently the
// map merger) treat a false flag as "this agent's observations are not available
// to the fleet right now". Each robot gets a phase-shifted duty cycle so the
// fleet spends time in partial-connectivity states, which is where the epistemic
// asymmetries the project is about actually arise: with a single global on/off
// switch every agent would always share the same information.
//
// Dropouts are off by default; enable with `comms_dropout:=true` on the launch.

#include <chrono>
#include <cmath>
#include <memory>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>

namespace mrs_coordination
{

class CommsLinkNode : public rclcpp::Node
{
public:
  CommsLinkNode()
  : rclcpp::Node("comms_link")
  {
    declare_parameter<std::vector<std::string>>("robot_names", {"robot1", "robot2"});
    declare_parameter<bool>("dropout_enabled", false);
    declare_parameter<double>("uptime_sec", 45.0);
    declare_parameter<double>("downtime_sec", 25.0);
    declare_parameter<double>("phase_offset_sec", 12.0);
    declare_parameter<double>("publish_rate", 2.0);

    robot_names_ = get_parameter("robot_names").as_string_array();
    dropout_enabled_ = get_parameter("dropout_enabled").as_bool();
    uptime_ = get_parameter("uptime_sec").as_double();
    downtime_ = get_parameter("downtime_sec").as_double();
    phase_offset_ = get_parameter("phase_offset_sec").as_double();
    const double rate = get_parameter("publish_rate").as_double();

    const auto qos = rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable();
    for (const auto & name : robot_names_) {
      publishers_.push_back(
        create_publisher<std_msgs::msg::Bool>("/" + name + "/comms_ok", qos));
      last_state_.push_back(true);
    }

    start_ = now();
    timer_ = create_wall_timer(
      std::chrono::duration<double>(1.0 / std::max(rate, 0.1)),
      std::bind(&CommsLinkNode::tick, this));

    RCLCPP_INFO(
      get_logger(), "comms model for %zu robots, dropouts %s (%.0fs up / %.0fs down)",
      robot_names_.size(), dropout_enabled_ ? "enabled" : "disabled", uptime_, downtime_);
  }

private:
  void tick()
  {
    const double elapsed = (now() - start_).seconds();
    const double period = uptime_ + downtime_;

    for (size_t i = 0; i < robot_names_.size(); ++i) {
      bool connected = true;
      if (dropout_enabled_ && period > 0.0) {
        // std::fmod keeps the sign of the dividend, so shift into [0, period).
        double phase = std::fmod(elapsed + static_cast<double>(i) * phase_offset_, period);
        if (phase < 0.0) {
          phase += period;
        }
        connected = phase < uptime_;
      }

      if (connected != last_state_[i]) {
        RCLCPP_INFO(
          get_logger(), "[t=%.1fs] %s link %s",
          elapsed, robot_names_[i].c_str(), connected ? "restored" : "lost");
        last_state_[i] = connected;
      }

      std_msgs::msg::Bool msg;
      msg.data = connected;
      publishers_[i]->publish(msg);
    }
  }

  std::vector<std::string> robot_names_;
  bool dropout_enabled_{false};
  double uptime_{45.0};
  double downtime_{25.0};
  double phase_offset_{12.0};

  rclcpp::Time start_;
  std::vector<bool> last_state_;
  std::vector<rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr> publishers_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace mrs_coordination

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<mrs_coordination::CommsLinkNode>());
  rclcpp::shutdown();
  return 0;
}
