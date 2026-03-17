#pragma once

#include "plansys2_executor/ActionExecutorClient.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"

namespace robot_plansys
{

class MoveAction : public plansys2::ActionExecutorClient
{
public:
  MoveAction();

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_activate(const rclcpp_lifecycle::State & previous_state) override;

private:
  void do_work() override;
  float progress_;
};

}  // namespace robot_plansys
