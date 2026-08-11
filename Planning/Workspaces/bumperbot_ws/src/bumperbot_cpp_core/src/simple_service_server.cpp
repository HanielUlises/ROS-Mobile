#include <rclcpp/rclcpp.hpp>
#include <bumperbot_msgs/srv/add_two_ints.hpp>

using namespace std::placeholders;

class SimpleServiceServer : public rclcpp::Node{
    public:
        SimpleServiceServer() : Node("simple_service_server") {
            service = create_service<bumperbot_msgs::srv::AddTwoInts> ("add_two_ints",
                std::bind(&SimpleServiceServer::service_callback, this, _1, _2));

            RCLCPP_INFO_STREAM(get_logger(), "Service add_two_ints ready");
        }
    
    private:
        rclcpp::Service<bumperbot_msgs::srv::AddTwoInts>::SharedPtr service; 
        
        void service_callback(std::shared_ptr<bumperbot_msgs::srv::AddTwoInts::Request> req,
                              std::shared_ptr<bumperbot_msgs::srv::AddTwoInts::Response> resp) {
            RCLCPP_INFO_STREAM(get_logger(), "New Request Received\n a: " << req -> a << " b: " << req -> b );
            resp -> sum = req -> a + req -> b;
            RCLCPP_INFO_STREAM(get_logger(), "Response Sum: " << resp -> sum);
        }
};  

int main(int argc, char* argv[]){
    rclcpp::init(argc, argv);
    auto node = std::make_shared<SimpleServiceServer>();
    rclcpp::spin(node);
    rclcpp::shutdown();

    return 0;
}