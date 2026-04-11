#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"

class TwistRelay : public rclcpp::Node {
    public:
        TwistRelay() : Node("twist_relay") {
            controller_sub = create_subscription<geometry_msgs::msg::Twist>(
                
            );
        }
    
    private:
        rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr controller_sub;
        rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr controller_pub;
};