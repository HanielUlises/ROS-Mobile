#include "robot_plansys/pick_action.hpp"
#include <memory>

namespace robot_plansys
{

PickAction::PickAction()
: plansys2::ActionExecutorClient("pick_object", std::chrono::milliseconds(500)),
  progress_(0.0f)
{
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
PickAction::on_activate(const rclcpp_lifecycle::State & previous_state)
{
  progress_ = 0.0f;
  RCLCPP_INFO(get_logger(), "PickAction activated: picking [%s] at [%s]",
    get_arguments()[1].c_str(), get_arguments()[2].c_str());
  return ActionExecutorClient::on_activate(previous_state);
}

void PickAction::do_work()
{
  progress_ += 0.25f;

  RCLCPP_INFO(get_logger(), "Picking... %.0f%%", progress_ * 100.0f);
  send_feedback(progress_, "Picking object");

  if (progress_ >= 1.0f) {
    finish(true, 1.0f, "Pick completed successfully");
    progress_ = 0.0f;
    RCLCPP_INFO(get_logger(), "Pick DONE: robot now carrying [%s]",
      get_arguments()[1].c_str());
  }
}

}  // namespace robot_plansys

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<robot_plansys::PickAction>();
  node->set_parameter(rclcpp::Parameter("action_name", "pick_object"));
  node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);

  rclcpp::spin(node->get_node_base_interface());
  rclcpp::shutdown();
  return 0;
}
