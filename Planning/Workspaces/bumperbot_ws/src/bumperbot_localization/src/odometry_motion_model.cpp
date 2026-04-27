#include "bumperbot_localization/odometry_motion_model.hpp"

#include <geometry_msgs/msg/pose.hpp>

using std::placeholders::_1;

OdometryMotionModel::OdometryMotionModel(const std::string &name) : 
    Node(name), alpha1_(0.0f), alpha2_(0.0f), alpha3_(0.0f), alpha4_(0.0f), nr_samples_s(300), last_odom_x_(0.0f), last_odom_y_(0.0f), last_odom_theta_(0.0f), is_first_odom_(true) {
    declare_parameter("alpha1", 0.1);
    declare_parameter("alpha2", 0.1);
    declare_parameter("alpha3", 0.1);
    declare_parameter("alpha4", 0.1);
    declare_parameter("nr_samples", 300);

    alpha1_ = get_parameter("alpha1").as_double();
    alpha2_ = get_parameter("alpha2").as_double();
    alpha3_ = get_parameter("alpha3").as_double();
    alpha4_ = get_parameter("alpha4").as_double();
    nr_samples_s = get_parameter("nr_samples").as_double();

    if(nr_samples_s > 0) {
        samples_.poses = std::vector<geometry_msgs::msg::Pose>(nr_samples_s, geometry_msgs::msg::Pose());
    } else {
        RCLCPP_FATAL_STREAM(get_logger(), "Invalid Number of Samples requested: " << nr_samples_s);
        return;
    }

    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>("bumperbot_controller/odom", 10, std::bind(&OdometryMotionModel::odom_callback, this, _1));
    pose_array_pub = create_publisher<geometry_msgs::msg::PoseArray>("bumperbot_controller/odom_kalman", 10);
}

void OdometryMotionModel::odom_callback(const nav_msgs::msg::Odometry &odom) {

}

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<OdometryMotionModel>("kalman_filter");
    rclcpp::spin(node);
    rclcpp::shutdown();

    return 0;
}