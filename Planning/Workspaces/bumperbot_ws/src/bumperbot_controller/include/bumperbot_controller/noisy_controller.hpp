#ifndef SIMPLE_CONTROLLER_HPP
#define SIMPLE_CONTROLLER_HPP

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <tf2_ros/transform_broadcaster.hpp>

#include <Eigen/Core>

class SimpleController : public rclcpp::Node{
    public:
        SimpleController(const std::string &name);

    private: 
        void velocity_callback(const geometry_msgs::msg::TwistStamped &msg);
        void joint_callback(const sensor_msgs::msg::JointState &msg);

        rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr vel_sub;
        rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_sub;
        rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr wheel_cmd_pub;
        rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub;

        double wheel_radius;
        double wheel_separation;
        Eigen::Matrix2d speed_conversion;
        
        // Positions for wheel odometry
        double left_wheel_prev_pos;
        double right_wheel_prev_pos;
        rclcpp::Time previous_time;

        double x, y, theta;

        nav_msgs::msg::Odometry odom_msg;

        std::unique_ptr<tf2_ros::TransformBroadcaster> transform_broadcaster;
        geometry_msgs::msg::TransformStamped transform_stamped;
};

#endif