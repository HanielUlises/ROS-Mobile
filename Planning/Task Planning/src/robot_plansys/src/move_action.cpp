#include "robot_plansys/move_action.hpp"
#include <memory>

namespace robot_plansys
{

MoveAction::MoveAction()
: plansys2::ActionExecutorClient("move", std::chrono::milliseconds(500)),
  progress_(0.0f)
{
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
MoveAction::on_activate(const rclcpp_lifecycle::State & previous_state)
{
  progress_ = 0.0f;
  RCLCPP_INFO(get_logger(), "MoveAction activated: moving from [%s] to [%s]",
    get_arguments()[1].c_str(), get_arguments()[2].c_str());
  return ActionExecutorClient::on_activate(previous_state);
}

void MoveAction::do_work()
{
  progress_ += 0.1f;

  RCLCPP_INFO(get_logger(), "Moving... %.0f%%", progress_ * 100.0f);
  send_feedback(progress_, "Moving robot");

  if (progress_ >= 1.0f) {
    finish(true, 1.0f, "Move completed successfully");
    progress_ = 0.0f;
    RCLCPP_INFO(get_logger(), "Move DONE: robot now at [%s]",
      get_arguments()[2].c_str());
  }
}

}  // namespace robot_plansys

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<robot_plansys::MoveAction>();
  node->set_parameter(rclcpp::Parameter("action_name", "move"));
  node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);

  rclcpp::spin(node->get_node_base_interface());
  rclcpp::shutdown();
  return 0;
}
