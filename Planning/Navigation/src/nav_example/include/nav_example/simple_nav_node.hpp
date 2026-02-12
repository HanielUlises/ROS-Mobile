#ifndef NAV_EXAMPLE_SIMPLE_NAV_NODE_HPP_
#define NAV_EXAMPLE_SIMPLE_NAV_NODE_HPP_

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <nav2_msgs/action/navigate_to_pose.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>

#include <vector>

class SimpleNavNode : public rclcpp::Node
{
public:
  explicit SimpleNavNode();

private:
  using NavigateToPose = nav2_msgs::action::NavigateToPose;
  using GoalHandle = rclcpp_action::ClientGoalHandle<NavigateToPose>;

  void loadGoals();
  void sendNextGoal();

  void goalResponseCallback(const GoalHandle::SharedPtr &goal_handle);
  void feedbackCallback(GoalHandle::SharedPtr, const std::shared_ptr<const NavigateToPose::Feedback> feedback);
  void resultCallback(const GoalHandle::WrappedResult &result);

  rclcpp_action::Client<NavigateToPose>::SharedPtr client_;

  std::vector<std::vector<double>> goals_;

  size_t current_goal_index_ = 0;
  GoalHandle::SharedPtr current_goal_handle_;
};

#endif  // NAV_EXAMPLE_SIMPLE_NAV_NODE_HPP_