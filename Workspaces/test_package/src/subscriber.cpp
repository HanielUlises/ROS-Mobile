#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

#include <iostream>

class HWSubNode : public rclcpp :: Node{
    public:
        HWSubNode() : Node ("hello_world_sub_node") {
            subscription_ = this -> create_subscription<std_msgs::msg::String>(
                "hello world", 10, std::bind(&HWSubNode::sub_callback, this, std::placeholders::_1)
            ); 
        }
    
    private:
        void sub_callback(const std_msgs::msg::String & msg){
            std::cout << msg.data << "\n";
        }

        rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
};

int main (int argc, char** argv){
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<HWSubNode>());
    rclcpp::shutdown();

    return 0;
}