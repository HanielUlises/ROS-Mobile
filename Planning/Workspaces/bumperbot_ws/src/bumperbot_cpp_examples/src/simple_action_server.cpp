#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "bumperbot_msgs/action/fibonacci.hpp"

namespace bumperbot_cpp_examples {
    class SimpleActionServer : public rclcpp::Node {
        public:
            explicit SimpleActionServer(const rclcpp::NodeOptions &options = rclcpp::NodeOptions()) : Node("simple_action_server", options) {

            }

        private:
            rclcpp_action::Server<>::SharedPtr action_server_;
    };
}