#include "bumperbot_localization/odometry_motion_model.hpp"

using std::placeholders::_1;

OdometryMotionModel::OdometryMotionModel(const std::string &name) : 
    Node(name) {
    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>("bumperbot_controller/odom", 10, std::bind(&OdometryMotionModel::odom_callback, this, _1));

    pose_array_pub = create_publisher<geometry_msgs::msg::PoseArray>("bumperbot_controller/odom_kalman", 10);
}

void OdometryMotionModel::odom_callback(const nav_msgs::msg::Odometry &odom) {

}

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<OdometryMotionModel>("kalman_filter");
    rclcpp::spin(node);
    rclcpp::shutdown();

    return 0;
}