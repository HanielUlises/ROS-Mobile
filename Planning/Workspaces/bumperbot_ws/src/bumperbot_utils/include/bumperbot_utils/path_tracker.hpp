#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>

#include <memory>
#include <vector>
#include <vector>

class PathTracker : public rclcpp::Node {
    public:
        PathTracker();
    private:
        void odometry_callback(const nav_msgs::msg::Odometry &);

        rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odometry_subscription;
        rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr trajectory_publisher;
        
        nav_msgs::msg::Path path;
};
