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

class SlamNode : public rclcpp::Node {
public:
  SlamNode() : Node("slam_node"), previous_cloud_(new pcl::PointCloud<pcl::PointXYZ>) {
    scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
      "/scan", 10, std::bind(&SlamNode::scanCallback, this, std::placeholders::_1));

    odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("/odom", 10);
    tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);

    current_transform_.setIdentity();
  }

private:
  void scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr scan) {
    pcl::PointCloud<pcl::PointXYZ>::Ptr current_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    for (size_t i = 0; i < scan->ranges.size(); ++i) {
      if (std::isfinite(scan->ranges[i])) {
        float angle = scan->angle_min + i * scan->angle_increment;
        pcl::PointXYZ point;
        point.x = scan->ranges[i] * cos(angle);
        point.y = scan->ranges[i] * sin(angle);
        point.z = 0.0;
        current_cloud->points.push_back(point);
      }
    }

    if (previous_cloud_->empty()) {
      *previous_cloud_ = *current_cloud;
      return;
    }

    pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
    icp.setInputSource(current_cloud);
    icp.setInputTarget(previous_cloud_);
    pcl::PointCloud<pcl::PointXYZ> aligned_cloud;
    icp.align(aligned_cloud);

    if (icp.hasConverged()) {
      Eigen::Matrix4f transformation = icp.getFinalTransformation();
      current_transform_ = current_transform_ * transformation;

      geometry_msgs::msg::TransformStamped transform_stamped;
      transform_stamped.header.stamp = this->now();
      transform_stamped.header.frame_id = "odom";
      transform_stamped.child_frame_id = "base_link";
      transform_stamped.transform.translation.x = current_transform_(0, 3);
      transform_stamped.transform.translation.y = current_transform_(1, 3);
      transform_stamped.transform.translation.z = 0.0;
      tf2::Quaternion q;
      q.setRPY(0, 0, std::atan2(current_transform_(1, 0), current_transform_(0, 0)));
      transform_stamped.transform.rotation.x = q.x();
      transform_stamped.transform.rotation.y = q.y();
      transform_stamped.transform.rotation.z = q.z();
      transform_stamped.transform.rotation.w = q.w();
      tf_broadcaster_->sendTransform(transform_stamped);

      nav_msgs::msg::Odometry odom;
      odom.header.stamp = this->now();
      odom.header.frame_id = "odom";
      odom.child_frame_id = "base_link";
      odom.pose.pose.position.x = current_transform_(0, 3);
      odom.pose.pose.position.y = current_transform_(1, 3);
      odom.pose.pose.position.z = 0.0;
      odom.pose.pose.orientation = transform_stamped.transform.rotation;
      odom_pub_->publish(odom);
    }

    *previous_cloud_ = *current_cloud;
  }

  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  pcl::PointCloud<pcl::PointXYZ>::Ptr previous_cloud_;
  Eigen::Matrix4f current_transform_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SlamNode>());
  rclcpp::shutdown();
  return 0;
}