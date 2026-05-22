#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "std_msgs/msg/bool.hpp"

class SafetyStop : public rclcpp::Node {
    public:
        SafetyStop() : Node("safety_stop_node") {
            declare_parameter<double>("danger_distance", 0.2);
            declare_parameter<std::string>("scan_topic", "scan");
            declare_parameter<std::string>("safety_stop_topic", "");

            danger_distance_ = get_parameter("danger_distance").as_double();
            scan_topic_ = get_parameter("scan_topic").as_string();
            safety_stop_topic_ = get_parameter("safety_stop_topic").as_string();

            laser_sub = create_subscription<sensor_msgs::msg::LaserScan>(
                scan_topic_, 10, std::bind(&SafetyStop::laser_callback, this, std::placeholders::_1)
            );

            safety_stop_pub = create_publisher<std_msgs::msg::Bool>(safety_stop_topic_, 10);
        }
    private:
        double danger_distance_;
        std::string scan_topic_;
        std::string safety_stop_topic_;
        
        rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laser_sub;
        rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr safety_stop_pub;
};