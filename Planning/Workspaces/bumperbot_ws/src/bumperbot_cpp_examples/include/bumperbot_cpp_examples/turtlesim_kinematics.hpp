#pragma once

#include <rclcpp/rclcpp.hpp>
#include <turtlesim/msg/pose.hpp>

class TurtlesimKinematics : public rclcpp::Node {
        void turtle1_pose_callback(const turtlesim::msg::Pose &pose);
        void turtle2_pose_callback(const turtlesim::msg::Pose &pose);
        
        rclcpp::Subscription<turtlesim::msg::Pose>::SharedPtr turtle1_pose_sub;
        rclcpp::Subscription<turtlesim::msg::Pose>::SharedPtr turtle2_pose_sub;

        turtlesim::msg::Pose last_turtle1_pose;
        turtlesim::msg::Pose last_turtle2_pose;
    public:
        TurtlesimKinematics(const std::string &name);
};