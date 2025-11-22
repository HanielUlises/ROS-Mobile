#include "bumperbot_cpp_examples/simple_tf_kinematics.hpp"

using namespace std::chrono_literals;

SimpleTfKinematics::SimpleTfKinematics(const std::string &name) : Node(name), x_increment(0.05f), last_x(0.0f) {
    static_tf_broadcaster = std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);
    dynamic_tf_broadcaster = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

    static_transform_stamped.header.stamp = get_clock() -> now();
    static_transform_stamped.header.frame_id = "bumperbot_base";
    static_transform_stamped.child_frame_id = "bumperbot_top";

    static_transform_stamped.transform.translation.x = 0.0f;
    static_transform_stamped.transform.translation.y = 0.0f;
    static_transform_stamped.transform.translation.z = 0.3f;

    // Quaternions
    static_transform_stamped.transform.rotation.x = 0.0;
    static_transform_stamped.transform.rotation.y = 0.0;
    static_transform_stamped.transform.rotation.z = 0.0;
    static_transform_stamped.transform.rotation.w = 0.0;

    static_tf_broadcaster -> sendTransform(static_transform_stamped);

    RCLCPP_INFO_STREAM(get_logger(), "Publishing static transform: "
                                     << static_transform_stamped.header.frame_id << " & " << static_transform_stamped.child_frame_id);

    timer = create_wall_timer(0.1s, std::bind(&SimpleTfKinematics::timer_callback, this));
}

void SimpleTfKinematics::timer_callback() {
    dynamic_transform_stamped.header.stamp = get_clock() -> now();
    dynamic_transform_stamped.header.frame_id = "odom";
    dynamic_transform_stamped.child_frame_id = "bumperbot_base";

    dynamic_transform_stamped.transform.translation.x = last_x + x_increment;
    dynamic_transform_stamped.transform.translation.y = 0.0f;
    dynamic_transform_stamped.transform.translation.z = 0.0f;

    dynamic_transform_stamped.transform.rotation.x = 0.0f;
    dynamic_transform_stamped.transform.rotation.y = 0.0f;
    dynamic_transform_stamped.transform.rotation.z = 0.0f;
    dynamic_transform_stamped.transform.rotation.w = 1.0f;

    dynamic_tf_broadcaster -> sendTransform(dynamic_transform_stamped);

    last_x = dynamic_transform_stamped.transform.translation.x;
}

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<SimpleTfKinematics>("simple_tf_kinematics");
    rclcpp::spin(node);
    rclcpp::shutdown();

    return 0;
}
