#ifndef KALMAN_FILTER_HPP
#define KALMAN_FILTER_HPP

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/imu.hpp>

class KalmanFilter : public rclcpp::Node {
    public:
        KalmanFilter(const std::string &name);

    private:
        rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
        rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
        rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub;

        double mean;
        double variance;
        double imu_angular_z;
        double last_angular_z_;

        bool is_first_odom_;

        nav_msgs::msg::Odometry kalman_odom;

        void odom_callback(const nav_msgs::msg::Odometry &odom);
        void imu_callback(const sensor_msgs::msg::Imu &imu);
};

#endif