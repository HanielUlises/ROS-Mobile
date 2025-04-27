#include "rclcpp/rclcpp.hpp"
#include "test_package/srv/turn_camera.hpp"

#include <iostream>

#include <opencv2/imgcodecs.hpp>
#include <cv_bridge/cv_bridge.h>

typedef test_package::srv::TurnCamera TurnCameraSrv;

class TurnCameraServiceNode : public rclcpp::Node{
    public:
        TurnCameraServiceNode(std::string exe_dir) : Node ("turn_camera_service_node"){
            ws_dir_ = get_ws_dir(exe_dir);
            service_server_ = this -> create_service<TurnCameraSrv> (
                "turn_camera",
                std::bind(&TurnCameraServiceNode::get_camera_image, this, std::placeholders::_1,
                                                                              std::placeholders::_2)
            );
            std::cout << "Turn Camera Service is Running..." << std::endl;
        }
    private:
        void get_camera_image(const TurnCameraSrv::Request::SharedPtr request,
            TurnCameraSrv::Response::SharedPtr response){
                float closest_num = available_angles_[0];
                float angle_diff = 0.0f;
                float smallest_angle = std::abs(request -> degree_turn - available_angles_[0]);
                for(int i = 0; i < 5; ++i){
                    angle_diff = std::abs(request -> degree_turn - available_angles_[i]);
                    if(angle_diff < smallest_angle){
                        smallest_angle = angle_diff;
                        closest_num = available_angles_[i];
                    }
                }
                std::string image_path = ws_dir_ + "/images/" + std::to_string(static_cast<int> (closest_num)) + ".png";
                std::cout << image_path << std::endl;

                cv::Mat image = cv::imread(image_path);
                auto image_ptr = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", image).toImageMsg();

                response -> camera_image = *image_ptr;
                
            }

        std::string get_ws_dir (std::string executable_directory){
            std::string::size_type substr_index = executable_directory.find("install");             
            return executable_directory.substr(0, substr_index);
        }
        
        rclcpp::Service<TurnCameraSrv>::SharedPtr service_server_;
        const float available_angles_ [5] = {-30, -15, 0, 15, 30};
        std::string ws_dir_;
};

int main (int argc, char **argv){
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<TurnCameraServiceNode> (argv[0]));
    rclcpp::shutdown();

    return 0;
}