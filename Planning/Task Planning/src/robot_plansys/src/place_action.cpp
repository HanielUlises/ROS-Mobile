#include "robot_plansys/place_action.hpp"
#include <memory>

namespace robot_plansys
{

PlaceAction::PlaceAction()
: plansys2::ActionExecutorClient("place_object", std::chrono::milliseconds(500)),
  progress_(0.0f)
{
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
PlaceAction::on_activate(const rclcpp_lifecycle::State & previous_state)
{
  progress_ = 0.0f;
  RCLCPP_INFO(get_logger(), "PlaceAction activated: placing [%s] at [%s]",
    get_arguments()[1].c_str(), get_arguments()[2].c_str());
  return ActionExecutorClient::on_activate(previous_state);
}

void PlaceAction::do_work()
{
  progress_ += 0.25f;

  RCLCPP_INFO(get_logger(), "Placing... %.0f%%", progress_ * 100.0f);
  send_feedback(progress_, "Placing object");

  if (progress_ >= 1.0f) {
    finish(true, 1.0f, "Place completed successfully");
    progress_ = 0.0f;
    RCLCPP_INFO(get_logger(), "Place DONE: [%s] delivered to [%s]",
      get_arguments()[1].c_str(), get_arguments()[2].c_str());
  }
}

}  // namespace robot_plansys

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<robot_plansys::PlaceAction>();
  node->set_parameter(rclcpp::Parameter("action_name", "place_object"));
  node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);

  rclcpp::spin(node->get_node_base_interface());
  rclcpp::shutdown();
  return 0;
}
