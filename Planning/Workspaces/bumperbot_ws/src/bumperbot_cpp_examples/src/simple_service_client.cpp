#include <rclcpp/rclcpp.hpp>
#include <bumperbot_msgs/srv/add_two_ints.hpp>

#include <chrono>

using namespace std::chrono_literals;
using namespace std::placeholders;

class SimpleServiceClient : public rclcpp::Node {
    public:
        SimpleServiceClient(int _a, int _b) : Node("simple_service_client") {
            client = create_client<bumperbot_msgs::srv::AddTwoInts> ("add_two_ints");
            auto request = std::make_shared<bumperbot_msgs::srv::AddTwoInts::Request>();
            request -> a = _a;
            request -> b = _b;

            while(!client -> wait_for_service(1s)){
                if(!rclcpp::ok){
                    RCLCPP_ERROR(get_logger(), "Interrupted while waiting for service.");
                    return;
                }
                RCLCPP_INFO_STREAM(get_logger(), "Service not available, waiting again...");
            }

            // Future
            auto result = client -> async_send_request(request, std::bind(&SimpleServiceClient::response_callback, this, _1));
        }
    private:
        rclcpp::Client<bumperbot_msgs::srv::AddTwoInts>::SharedPtr client;

        void response_callback(rclcpp::Client<bumperbot_msgs::srv::AddTwoInts>::SharedFuture future){
            if(future.valid()){
                RCLCPP_INFO_STREAM(get_logger(), "Service Response: " << future.get() -> sum );
            } else {
                RCLCPP_ERROR(get_logger(), "Serive Failure");
            }
        }
};

int main(int argc, char* argv[]) {
    if(argc != 3) {
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Usage: simple_service_client A B");
        return 1;
    }


    auto node = std::make_shared<SimpleServiceClient>(atoi(argv[1]), atoi(argv[2]));
    rclcpp::spin(node);
    rclcpp::shutdown();
    
    return 0;
}