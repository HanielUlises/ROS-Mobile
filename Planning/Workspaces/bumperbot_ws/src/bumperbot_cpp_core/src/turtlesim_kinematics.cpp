#include "bumperbot_cpp_examples/turtlesim_kinematics.hpp"

using std::placeholders::_1;

TurtlesimKinematics::TurtlesimKinematics(const std::string &name) : Node(name){
    turtle1_pose_sub = create_subscription<turtlesim::msg::Pose> ("/turtle1/pose", 10, 
        std::bind(&TurtlesimKinematics::turtle1_pose_callback, this, _1));
    turtle2_pose_sub = create_subscription<turtlesim::msg::Pose> ("/turtle2/pose", 10, 
        std::bind(&TurtlesimKinematics::turtle2_pose_callback, this, _1));

}

void TurtlesimKinematics::turtle1_pose_callback(const turtlesim::msg::Pose &pose){
    last_turtle1_pose = pose;
}

void TurtlesimKinematics::turtle2_pose_callback(const turtlesim::msg::Pose &pose){
    last_turtle2_pose = pose;
    float Tx = last_turtle2_pose.x - last_turtle1_pose.x;
    float Ty = last_turtle2_pose.y - last_turtle1_pose.y;

    float theta_rad = last_turtle2_pose.theta - last_turtle1_pose.theta;
    float theta_deg = 180 * theta_rad / std::numbers::pi;

    RCLCPP_INFO_STREAM(get_logger(), "\nTranslation Vector turtle1 -> turtle2 \n" <<
        "Tx: " << Tx << "\n" <<
        "Ty: " << Ty << "\n" <<
        "Rotation Matrix t1 -> t2 \n" <<
        "theta(radians): " << theta_rad << "\n" <<
        "theta(degrees): " << theta_deg << "\n" <<
        "|R11       R12| : |" << std::cos(theta_rad) <<"\t" << -std::sin(theta_rad) << "|\n" <<
        "|R21       R22| : |" << std::sin(theta_rad) <<"\t" << std::cos(theta_rad) << "|\n"
    );
}

int main(int argc, char* argv[]){
    rclcpp::init(argc, argv);
    auto node = std::make_shared<TurtlesimKinematics>("Simple_turtlesim_kinematics");
    rclcpp::spin(node);
    rclcpp::shutdown();

    return 0;
}