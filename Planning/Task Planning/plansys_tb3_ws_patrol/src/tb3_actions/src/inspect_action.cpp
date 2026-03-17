#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "lifecycle_msgs/msg/transition.hpp"
#include "plansys2_executor/ActionExecutorClient.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

using namespace std::chrono_literals;

class InspectAction : public plansys2::ActionExecutorClient
{
public:
  InspectAction()
  : plansys2::ActionExecutorClient("inspect", 500ms),
    scan_count_(0)
  {}

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_activate(const rclcpp_lifecycle::State & previous_state)
  {
    scan_count_ = 0;

    scan_sub_ = create_subscription<sensor_msgs::msg::LaserScan>(
      "/scan", rclcpp::SensorDataQoS(),
      [this](sensor_msgs::msg::LaserScan::SharedPtr) { ++scan_count_; });

    return ActionExecutorClient::on_activate(previous_state);
  }

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_deactivate(const rclcpp_lifecycle::State & previous_state)
  {
    scan_sub_.reset();
    return ActionExecutorClient::on_deactivate(previous_state);
  }

  void do_work() override
  {
    auto args = get_arguments();
    if (args.size() < 2) { finish(false, 0.0, "Invalid arguments"); return; }

    const int scans_needed = 16;
    float progress = std::min(1.0f, static_cast<float>(scan_count_) / scans_needed);
    send_feedback(progress, "Scanning...");

    RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000,
      "[inspect] %s  —  %d/%d scans", args[1].c_str(), scan_count_, scans_needed);

    if (scan_count_ >= scans_needed) {
      RCLCPP_INFO(get_logger(), "[inspect] %s complete.", args[1].c_str());
      finish(true, 1.0, "Inspection complete");
    }
  }

private:
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
  int scan_count_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<InspectAction>();
  node->set_parameter(rclcpp::Parameter("action_name", "inspect"));

  auto config_timer = std::make_shared<rclcpp::TimerBase *>(nullptr);
  auto t = node->create_wall_timer(500ms, [&node, config_timer]() {
    node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);
    if (*config_timer) (*config_timer)->cancel();
  });
  *config_timer = t.get();

  rclcpp::spin(node->get_node_base_interface());
  rclcpp::shutdown();
  return 0;
}
