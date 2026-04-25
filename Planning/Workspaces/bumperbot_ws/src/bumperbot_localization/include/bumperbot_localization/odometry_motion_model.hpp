#ifndef ODOMETRY_MOTION_MODEL_HPP
#define ODOMETRY_MOTION_MODEL_HPP

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>

class OdometryMotionModel : public rclcpp::Node {
    public:
        OdometryMotionModel(const std::string &name);
    private:
        void odom_callback(const nav_msgs::msg::Odometry &);
        rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
        rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;

};

#endif // ODOMETRY_MOTION_MODEL_HPP