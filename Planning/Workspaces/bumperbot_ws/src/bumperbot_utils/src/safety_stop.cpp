#include "math.h"

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "std_msgs/msg/bool.hpp"
#include "twist_mux_msgs/action/joy_turbo.hpp"

enum State {
    FREE = 0,
    WARNING = 1,
    DANGER = 2
};

class SafetyStop : public rclcpp::Node {
    public:
        SafetyStop() : Node("safety_stop_node"), state_(State::FREE), prev_state_{State::FREE} {
            declare_parameter<double>("danger_distance", 0.2);
            declare_parameter<double>("warning_distance", 0.6);
            declare_parameter<std::string>("scan_topic", "scan");
            declare_parameter<std::string>("safety_stop_topic", "");

            danger_distance_ = get_parameter("danger_distance").as_double();
            scan_topic_ = get_parameter("scan_topic").as_string();
            safety_stop_topic_ = get_parameter("safety_stop_topic").as_string();

            laser_sub = create_subscription<sensor_msgs::msg::LaserScan>(
                scan_topic_, 10, std::bind(&SafetyStop::laser_callback, this, std::placeholders::_1)
            );

            safety_stop_pub = create_publisher<std_msgs::msg::Bool>(safety_stop_topic_, 10);
            decrease_speed_client_ = rclcpp_action::create_client<twist_mux_msgs::action::JoyTurbo> (this, "joy_turbo_decrease");
            increase_speed_client_ = rclcpp_action::create_client<twist_mux_msgs::action::JoyTurbo> (this, "joy_turbo_increase");
            
            while(!decrease_speed_client_ -> wait_for_action_server(std::chrono::seconds(1)) && rclcpp::ok()) {
                RCLCPP_WARN(get_logger(), "Action /joy_turbo_decrease not available. Waiting...");
                std::this_thread::sleep_for(std::chrono::seconds(2));
            }

            while(!increase_speed_client_ -> wait_for_action_server(std::chrono::seconds(1)) && rclcpp::ok()) {
                RCLCPP_WARN(get_logger(), "Action /joy_turbo_increase not available. Waiting...");
                std::this_thread::sleep_for(std::chrono::seconds(2));
            }
        }

    private:
        double danger_distance_, warning_distance_;
        std::string scan_topic_;
        std::string safety_stop_topic_;
        
        State state_, prev_state_;

        rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laser_sub;
        rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr safety_stop_pub;
        rclcpp_action::Client<twist_mux_msgs::action::JoyTurbo>::SharedPtr decrease_speed_client_;
        rclcpp_action::Client<twist_mux_msgs::action::JoyTurbo>::SharedPtr increase_speed_client_;

        void laser_callback(const sensor_msgs::msg::LaserScan &msg) {
            state_ = State::FREE;
            for(const auto &range: msg.ranges) {
                if(!std::isinf(range) && range <= warning_distance_) {
                    state_ = State::WARNING;
                    if(!std::isinf(range) && range <= danger_distance_) {
                        state_ = State::DANGER;
                        break;
                    }                   
                }
            }

            if(state_ != prev_state_) {
                std_msgs::msg::Bool is_safety_stop;
                if(state_ == State::WARNING) {
                    is_safety_stop.data = false;
                    decrease_speed_client_ -> async_send_goal(twist_mux_msgs::action::JoyTurbo::Goal());
                }
                if(state_ == State::DANGER) 
                    is_safety_stop.data = true;
                else if(state_ == State::FREE) {
                    is_safety_stop.data = false;
                    increase_speed_client_ -> async_send_goal(twist_mux_msgs::action::JoyTurbo::Goal());
                }
                    

                safety_stop_pub -> publish(is_safety_stop);
            }

            
        }
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<SafetyStop>();
    rclcpp::spin(node);
    rclcpp::shutdown();

    return 0;
}