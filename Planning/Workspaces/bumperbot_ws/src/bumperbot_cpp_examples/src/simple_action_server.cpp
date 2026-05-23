#include "rclcpp/rclcpp.hpp"

namespace bumperbot_cpp_examples {
    class SimpleActionServer : public rclcpp::Node {
        public:
            explicit SimpleActionServer(const rclcpp::NodeOptions &options = rclcpp::NodeOptions()) : Node("simple_action_server") {

            }

        private:
    };
}