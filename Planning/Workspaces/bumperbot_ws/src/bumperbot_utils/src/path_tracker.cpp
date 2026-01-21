#include "path_tracker.hpp"

PathTracker::PathTracker() : Node("path_tracker") {
    declare_parameter<std::string> ("odom/topic", "bumperbot_controller/odom");
    std::string odometry_topic = get_parameter("odom/topic").as_string();

    odometry_subscription = create_subscription<nav_msgs::msg::Odometry> (
        odometry_topic, 10, std::bind(&PathTracker::odometry_callback, this, std::placeholders::_1)
    );

    trajectory_publisher = create_publisher<nav_msgs::msg::Path> ("bumperbot_controller/trajectory", 10);
}

void PathTracker::odometry_callback(const nav_msgs::msg::Odometry &msg) {
    path.header.frame_id = msg.header.frame_id;

    geometry_msgs::msg::PoseStamped current_position;
    current_position.header.frame_id = msg.header.frame_id;
    current_position.header.stamp = msg.header.stamp;
    current_position.pose = msg.pose.pose;

    path.poses.push_back(current_position);
    trajectory_publisher -> publish(path);
}

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PathTracker>());
    rclcpp::shutdown();

    return 0;
}