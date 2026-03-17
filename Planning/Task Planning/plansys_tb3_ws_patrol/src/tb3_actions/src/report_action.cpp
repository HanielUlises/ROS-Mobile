#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "rclcpp_lifecycle/lifecycle_publisher.hpp"
#include "lifecycle_msgs/msg/transition.hpp"
#include "plansys2_executor/ActionExecutorClient.hpp"
#include "std_msgs/msg/string.hpp"

using namespace std::chrono_literals;

// Publishes one report message to /mission_reports, then waits one extra tick
// to confirm publication before finishing. No ACK dependency.
class ReportAction : public plansys2::ActionExecutorClient
{
public:
  ReportAction()
  : plansys2::ActionExecutorClient("report", 500ms),
    published_(false),
    ticks_after_pub_(0)
  {}

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_activate(const rclcpp_lifecycle::State & previous_state)
  {
    published_       = false;
    ticks_after_pub_ = 0;

    report_pub_ = create_publisher<std_msgs::msg::String>("/mission_reports", 10);
    report_pub_->on_activate();

    return ActionExecutorClient::on_activate(previous_state);
  }

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_deactivate(const rclcpp_lifecycle::State & previous_state)
  {
    report_pub_->on_deactivate();
    return ActionExecutorClient::on_deactivate(previous_state);
  }

  void do_work() override
  {
    auto args = get_arguments();
    if (args.size() < 2) { finish(false, 0.0, "Invalid arguments"); return; }

    if (!published_) {
      std_msgs::msg::String msg;
      msg.data = "REPORT | robot=" + args[0] + " | zone=" + args[1];
      report_pub_->publish(msg);
      published_ = true;
      send_feedback(0.5f, "Report transmitted");
      RCLCPP_INFO(get_logger(), "[report] %s", msg.data.c_str());
      return;
    }

    ++ticks_after_pub_;
    if (ticks_after_pub_ >= 2) {
      RCLCPP_INFO(get_logger(), "[report] %s done.", args[1].c_str());
      finish(true, 1.0, "Report complete");
    }
  }

private:
  rclcpp_lifecycle::LifecyclePublisher<std_msgs::msg::String>::SharedPtr report_pub_;
  bool published_;
  int  ticks_after_pub_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<ReportAction>();
  node->set_parameter(rclcpp::Parameter("action_name", "report"));

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
