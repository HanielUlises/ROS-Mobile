#include "rclcpp/rclcpp.hpp"
#include "test_package/srv/odd_even_check.hpp"

#include <iostream>

typedef test_package::srv::OddEvenCheck OddEvenCheckSrv;

int main (int argc, char** argv){
    rclcpp::init(argc, argv);

    auto service_client_node = rclcpp::Node::make_shared("odd_even_check_client_node");
    auto client = service_client_node -> create_client<OddEvenCheckSrv> ("odd_even_check");

    auto request = std::make_shared<OddEvenCheckSrv::Request>();

    std::cout << "Type a number to check if it is Odd or Even: ";
    std::cin >> request -> number;

    client -> wait_for_service();

    auto result = client -> async_send_request(request);
    if(rclcpp::spin_until_future_complete(service_client_node, result) == rclcpp::FutureReturnCode::SUCCESS){
        std::cout << "Number is: " << result.get() -> decision << std::endl;
    }else{
        std::cout << "There was an error in processing the request..." << std::endl;
    }
     
    rclcpp::shutdown();

    return 0;
}