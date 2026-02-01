
#ifndef MY_SLAM_CORE_SLAM_NODE_HPP_
#define MY_SLAM_CORE_SLAM_NODE_HPP_

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <Eigen/Core>

class SlamNode : public rclcpp::Node {
public:
  SlamNode();

private:
  void scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr scan);

  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  pcl::PointCloud<pcl::PointXYZ>::Ptr previous_cloud_;
  Eigen::Matrix4f current_transform_;
};

#endif  // MY_SLAM_CORE_SLAM_NODE_HPP_

Add ICP algorithm details

Implement particle filter localization

