#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64.hpp"

#include <cmath>
#include <iostream>

const double WHEEL_RADIUS = 12.5/100.0f;

class SpeedCalcNode : public rclcpp :: Node{
    public:
        SpeedCalcNode() : Node ("speed_calc_node") {
            rpm_subscription_ = this -> create_subscription<std_msgs::msg::Float64>(
                "rpm_pub", 10, std::bind(&SpeedCalcNode::calculate_and_pub_speed, this, std::placeholders::_1)
            ); 
            speed_publisher_ = this -> create_publisher<std_msgs::msg::Float64>("speed", 10);
            std::cout << "Speed Calc Node Is Running... " << '\n';
        }
    
    private:
        void calculate_and_pub_speed(const std_msgs::msg::Float64 & rpm_msg) const {
            auto speed_msg = std_msgs::msg::Float64(); 
            // Speed [m/s] = RPM [rev/min] * Wheel Circumference [meters/rev] / 60 [seconds/min]
            speed_msg.data = rpm_msg.data * (2 * WHEEL_RADIUS * M_PI)/60; // m/s
            speed_publisher_ -> publish(speed_msg);
        }

        rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr rpm_subscription_;
        rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr speed_publisher_;
};

int main (int argc, char** argv){
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SpeedCalcNode>());
    rclcpp::shutdown();

    return 0;
}