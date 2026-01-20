#include "rclcpp/rclcpp.hpp"
#include "test_package/srv/turn_camera.hpp"
#include <cv_bridge/cv_bridge.h>
#include <opencv2/highgui.hpp>
#include <iostream>

typedef test_package::srv::TurnCamera TurnCameraSrv;

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    auto service_client_node = rclcpp::Node::make_shared("turn_camera_client_node");
    auto client = service_client_node->create_client<TurnCameraSrv>("turn_camera");
    auto request = std::make_shared<TurnCameraSrv::Request>();

    std::cout << "Enter the position (in degrees) you want to turn the robot's camera to: ";
    if (!(std::cin >> request->degree_turn)) {
        std::cerr << "Invalid input!" << std::endl;
        rclcpp::shutdown();
        return 1;
    }

    if (!client->wait_for_service(std::chrono::seconds(5))) {
        std::cerr << "Service not available after 5 seconds!" << std::endl;
        rclcpp::shutdown();
        return 1;
    }

    auto result = client->async_send_request(request);
    if (rclcpp::spin_until_future_complete(service_client_node, result) != rclcpp::FutureReturnCode::SUCCESS) {
        std::cerr << "Service call failed!" << std::endl;
        rclcpp::shutdown();
        return 1;
    }

    auto response = result.get();
    if (response->camera_image.data.empty() || response->camera_image.encoding != "bgr8") {
        std::cerr << "Invalid camera image received!" << std::endl;
        rclcpp::shutdown();
        return 1;
    }

    try {
        auto cv_ptr = cv_bridge::toCvCopy(response->camera_image, "bgr8");
        if (cv_ptr->image.empty()) {
            std::cerr << "Converted image is empty!" << std::endl;
            rclcpp::shutdown();
            return 1;
        }
        cv::imshow("Robot Camera Image", cv_ptr->image);
        cv::waitKey(0);
    } catch (const cv_bridge::Exception& e) {
        std::cerr << "CvBridge error: " << e.what() << std::endl;
        rclcpp::shutdown();
        return 1;
    } catch (const std::exception& e) {
        std::cerr << "Unexpected error: " << e.what() << std::endl;
        rclcpp::shutdown();
        return 1;
    }

    rclcpp::shutdown();
    return 0;
}