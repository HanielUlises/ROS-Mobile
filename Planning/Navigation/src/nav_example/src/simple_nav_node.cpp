#include "nav_example/simple_nav_node.hpp"

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <nav2_msgs/action/navigate_to_pose.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>

#include <memory>
#include <vector>
#include <chrono>
#include <functional>

SimpleNavNode::SimpleNavNode() : Node("simple_nav_node")
{
    this->declare_parameter<double>("goal_x", 1.0);
    this->declare_parameter<double>("goal_y", 0.5);
    this->declare_parameter<double>("goal_yaw", 0.0);

    this->declare_parameter<std::vector<std::vector<double>>>("goals", {});

    client_ = rclcpp_action::create_client<NavigateToPose>(this, "navigate_to_pose");

    if (!client_->wait_for_action_server(std::chrono::seconds(30))) {
        RCLCPP_ERROR(this->get_logger(), "NavigateToPose action server not available after 30s");
        rclcpp::shutdown();
        return;
    }

    RCLCPP_INFO(this->get_logger(), "Connected to /navigate_to_pose action server");

    loadGoals();

    if (!goals_.empty()) {
        current_goal_index_ = 0;
        sendNextGoal();
    } else {
        RCLCPP_WARN(this->get_logger(), "No goals defined. Nothing to do.");
    }
}

void SimpleNavNode::loadGoals()
{
    auto multi_goals = this->get_parameter("goals").as_double_array_2d();
    if (!multi_goals.empty()) {
        for (const auto& g : multi_goals) {
            if (g.size() == 3) {
                goals_.push_back({g[0], g[1], g[2]});
            } else {
                RCLCPP_WARN(this->get_logger(), "Invalid goal format (need x,y,yaw). Skipping.");
            }
        }
        RCLCPP_INFO(this->get_logger(), "Loaded %zu goals from 'goals' parameter", goals_.size());
        return;
    }

    double x = this->get_parameter("goal_x").as_double();
    double y = this->get_parameter("goal_y").as_double();
    double yaw = this->get_parameter("goal_yaw").as_double();

    // radians -> quaternion
    goals_.push_back({x, y, yaw});
    RCLCPP_INFO(this->get_logger(), "Loaded single goal from parameters: (%.2f, %.2f, yaw=%.2f rad)", x, y, yaw);
}

void SimpleNavNode::sendNextGoal()
{
    if (current_goal_index_ >= goals_.size()) {
        RCLCPP_INFO(this->get_logger(), "All goals completed!");
        rclcpp::shutdown();
        return;
    }

    const auto& goal_data = goals_[current_goal_index_];
    double x = goal_data[0];
    double y = goal_data[1];
    double yaw = goal_data[2];

    auto goal_msg = NavigateToPose::Goal();
    goal_msg.pose.header.frame_id = "map";
    goal_msg.pose.header.stamp = this->now();

    goal_msg.pose.pose.position.x = x;
    goal_msg.pose.pose.position.y = y;
    goal_msg.pose.pose.position.z = 0.0;

    // Convert yaw (radians) to quaternion (only yaw rotation)
    double cy = std::cos(yaw * 0.5);
    double sy = std::sin(yaw * 0.5);
    goal_msg.pose.pose.orientation.x = 0.0;
    goal_msg.pose.pose.orientation.y = 0.0;
    goal_msg.pose.pose.orientation.z = sy;
    goal_msg.pose.pose.orientation.w = cy;

    RCLCPP_INFO(this->get_logger(), "Sending goal #%zu / %zu → (%.2f, %.2f, yaw=%.2f rad)",
                current_goal_index_ + 1, goals_.size(), x, y, yaw);

    auto send_goal_options = rclcpp_action::Client<NavigateToPose>::SendGoalOptions();
    send_goal_options.goal_response_callback =
        std::bind(&SimpleNavNode::goalResponseCallback, this, std::placeholders::_1);
    send_goal_options.feedback_callback =
        std::bind(&SimpleNavNode::feedbackCallback, this, std::placeholders::_1, std::placeholders::_2);
    send_goal_options.result_callback =
        std::bind(&SimpleNavNode::resultCallback, this, std::placeholders::_1);

    current_goal_handle_ = client_->async_send_goal(goal_msg, send_goal_options);
}

void SimpleNavNode::goalResponseCallback(const GoalHandle::SharedPtr &goal_handle)
{
    if (!goal_handle) {
        RCLCPP_ERROR(this->get_logger(), "Goal was rejected");
        current_goal_index_++;
        sendNextGoal();
        return;
    }

    RCLCPP_INFO(this->get_logger(), "Goal accepted");
    current_goal_handle_ = goal_handle;  
}

void SimpleNavNode::feedbackCallback(
    GoalHandle::SharedPtr,
    const std::shared_ptr<const NavigateToPose::Feedback> feedback)
{
    RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                         "Remaining distance: %.2f m", feedback->distance_remaining);
}

void SimpleNavNode::resultCallback(const GoalHandle::WrappedResult &result)
{
    switch (result.code) {
        case rclcpp_action::ResultCode::SUCCEEDED:
            RCLCPP_INFO(this->get_logger(), "Goal SUCCEEDED!");
            break;
        case rclcpp_action::ResultCode::ABORTED:
            RCLCPP_ERROR(this->get_logger(), "Goal ABORTED");
            break;
        case rclcpp_action::ResultCode::CANCELED:
            RCLCPP_ERROR(this->get_logger(), "Goal CANCELED");
            break;
        default:
            RCLCPP_ERROR(this->get_logger(), "Unknown result code: %d", result.code);
    }

    // Move to next goal regardless of success/failure
    current_goal_index_++;
    sendNextGoal();
}