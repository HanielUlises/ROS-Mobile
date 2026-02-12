#ifndef NAV_EXAMPLE_SIMPLE_NAV_NODE_HPP_
#define NAV_EXAMPLE_SIMPLE_NAV_NODE_HPP_

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <nav2_msgs/action/navigate_to_pose.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>

class SimpleNavNode : public rclcpp::Node
{
public:
  explicit SimpleNavNode();

private:
  using NavigateToPose = nav2_msgs::action::NavigateToPose;
  using GoalHandle = rclcpp_action::ClientGoalHandle<NavigateToPose>;

  void sendGoal();

  void goalResponseCallback(const GoalHandle::SharedPtr &goal_handle);

  void feedbackCallback(
    GoalHandle::SharedPtr,
    const std::shared_ptr<const NavigateToPose::Feedback> feedback);

  void resultCallback(const GoalHandle::WrappedResult &result);

  // Action client
  rclcpp_action::Client<NavigateToPose>::SharedPtr client_;
};

#endif  // NAV_EXAMPLE_SIMPLE_NAV_NODE_HPP_