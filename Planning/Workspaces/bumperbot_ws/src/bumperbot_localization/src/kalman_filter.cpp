#include "bumperbot_localization/kalman_filter.hpp"

using std::placeholders::_1;

KalmanFilter::KalmanFilter(const std::string &name) : 
    Node(name), mean(0.0f), variance(1000.0f), imu_angular_z(0.0), is_first_odom_(true), last_angular_z_(0.0), motion_(0.0) {
    
    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>("bumperbot_controller/odom_noisy", 10, std::bind(&KalmanFilter::odom_callback, this, _1));
    imu_sub_ = create_subscription<sensor_msgs::msg::Imu>("imu/data", 10, std::bind(&KalmanFilter::imu_callback, this, _1));

    odom_pub_ = create_publisher<nav_msgs::msg::Odometry>("bumperbot_controller/odom_kalman", 10);
}

void KalmanFilter::odom_callback(const nav_msgs::msg::Odometry &odom) {
    kalman_odom = odom;

    if(is_first_odom_) {
        mean = odom.twist.twist.angular.z;
        last_angular_z_ = odom.twist.twist.angular.z;
        is_first_odom_ = false;
        return;
    }

    
}

void KalmanFilter::imu_callback(const sensor_msgs::msg::Imu &imu) {

}