#ifndef SIMPLE_TF_KINEMATICS_HPP
#define SIMPLE_TF_KINEMATICS_HPP

#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/static_transform_broadcaster.hpp>
#include <tf2_ros/transform_broadcaster.hpp>
#include <tf2_ros/buffer.hpp>
#include <tf2_ros/transform_listener.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <bumperbot_msgs/srv/get_transform.hpp>

#include <memory>

class SimpleTfKinematics : public rclcpp::Node{
    public:
        SimpleTfKinematics(const std::string &name);

    private:
        std::shared_ptr<tf2_ros::StaticTransformBroadcaster> static_tf_broadcaster;
        std::unique_ptr<tf2_ros::TransformBroadcaster> dynamic_tf_broadcaster;
        
        geometry_msgs::msg::TransformStamped static_transform_stamped;
        geometry_msgs::msg::TransformStamped dynamic_transform_stamped;

        rclcpp::Service<bumperbot_msgs::srv::GetTransform>::SharedPtr get_transform_srv;
        
        rclcpp::TimerBase::SharedPtr timer;

        std::unique_ptr<tf2_ros::Buffer> tf_buffer;
        std::shared_ptr<tf2_ros::TransformListener> tf_listener {nullptr};

        double x_increment;
        double last_x;

        void timer_callback();

        bool get_transform_callback(const std::shared_ptr<bumperbot_msgs::srv::GetTransform::Request> req,
                                    const std::shared_ptr<bumperbot_msgs::srv::GetTransform::Response> resp);
};

#endif