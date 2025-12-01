#include "bumperbot_controller/simple_controller.hpp"

#include <Eigen/Geometry>

using std::placeholders::_1;

SimpleController::SimpleController(const std::string &name) : Node(name), 
    left_wheel_prev_pos(0.0), right_wheel_prev_pos(0.0) {
    declare_parameter("wheel_radius", 0.033);
    declare_parameter("wheel_separation", 0.17);

    wheel_radius = get_parameter("wheel_radius").as_double();
    wheel_separation = get_parameter("wheel_separation").as_double();

    RCLCPP_INFO_STREAM(get_logger(), "Using wheel_radius: " << wheel_radius);
    RCLCPP_INFO_STREAM(get_logger(), "Using wheel_separation: " << wheel_separation);

    previous_time = get_clock() -> now();

    wheel_cmd_pub = create_publisher<std_msgs::msg::Float64MultiArray>("/simple_velocity_controller/commands", 10);
    vel_sub = create_subscription<geometry_msgs::msg::TwistStamped>("/bumperbot_controller/cmd_vel", 
        10, std::bind(&SimpleController::velocity_callback, this, _1));

    joint_sub = create_subscription<sensor_msgs::msg::JointState>("/joint_states",
        10, std::bind(&SimpleController::joint_callback, this, _1));

    speed_conversion << wheel_radius/2, wheel_radius/2, wheel_radius/wheel_separation, -wheel_radius/wheel_separation;  

    RCLCPP_INFO_STREAM(get_logger(), "Conversion matrix: \n" << speed_conversion);

}

void SimpleController::velocity_callback(const geometry_msgs::msg::TwistStamped &msg){
    Eigen::Vector2d robot_speed(msg.twist.linear.x, msg.twist.angular.z);   
    Eigen::Vector2d wheel_speed = speed_conversion.inverse() * robot_speed;
    
    std_msgs::msg::Float64MultiArray wheel_speed_msg;
    wheel_speed_msg.data.push_back(wheel_speed.coeff(1));
    wheel_speed_msg.data.push_back(wheel_speed.coeff(0));
    
    wheel_cmd_pub -> publish(wheel_speed_msg);

}

void SimpleController::joint_callback(const sensor_msgs::msg::JointState &msg) {
    double delta_right = msg.position.at(0) - right_wheel_prev_pos;
    double delta_left = msg.position.at(1) - left_wheel_prev_pos;
    
    rclcpp::Time msg_time = msg.header.stamp;
    rclcpp::Duration dt = msg_time - previous_time;

    right_wheel_prev_pos = msg.position.at(0);
    left_wheel_prev_pos = msg.position.at(1);

    previous_time = msg_time;

    double fi_right = delta_right / dt.seconds();
    double fi_left = delta_left / dt.seconds();

    double linear = (wheel_radius * fi_right + wheel_radius * fi_left) / 2;
    double angular = (wheel_radius * fi_right - wheel_radius * fi_left) / wheel_separation;

    RCLCPP_INFO_STREAM(get_logger(), "Linear velocity: " << linear << "\n" <<"Angular Velocity" << angular);
}

int main(int argc, char* argv[]){
    rclcpp::init(argc, argv);
    auto node = std::make_shared<SimpleController>("simple_controller");
    rclcpp::spin(node);
    rclcpp::shutdown();
    
    return 0;
}