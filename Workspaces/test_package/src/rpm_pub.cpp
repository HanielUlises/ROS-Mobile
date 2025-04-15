#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64.hpp"

#include <chrono>
#include <functional>
#include <iostream>

using namespace std::chrono_literals;

const double RPM_VAL = 100.0f;

class RpmPubNode : public rclcpp :: Node{
    public:
        RpmPubNode() : Node ("rpm_pub_node") {
            publisher_ = this -> create_publisher<std_msgs::msg::Float64>(
                "hello_world", 10
            );
            timer_ = this -> create_wall_timer(1s,
            std::bind(&RpmPubNode::publish_hello_world, this));
            std::cout << "RPM Publisher Node is running... " << '\n';
        }

    private:
        void publish_hello_world(){
            auto message = std_msgs::msg::Float64();
            message.data = RPM_VAL;

            publisher_ -> publish(message);
        }
        
        rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr publisher_;
        rclcpp::TimerBase::SharedPtr timer_;
};

int main (int argc, char** argv){
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<RpmPubNode>());
    rclcpp::shutdown();

    return 0;
}