#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"

class TwistRelay : public rclcpp::Node {
    public:
        TwistRelay() : Node("twist_relay") {
            controller_sub = create_subscription<geometry_msgs::msg::Twist>(
                "/bumperbot_controller/cmd_vel_unstamped",
                10,
                std::bind(&TwistRelay::controller_twist_callback, this, std::placeholders::_1)
            );
        }
    
    private:
        rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr controller_sub;
        rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr controller_pub;
};