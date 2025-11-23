#include <rclcpp/rclcpp.hpp>

class SimpleServiceServer : rclcpp::Node{
    public:
        SimpleServiceServer() : Node("simple_service_server") {}
    
    private:
        rclcpp::Service<>::SharedPtr service;   
};