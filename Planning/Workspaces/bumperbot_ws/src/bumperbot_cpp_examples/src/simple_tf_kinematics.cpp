#include "bumperbot_cpp_examples/simple_tf_kinematics.hpp"

using namespace std::chrono_literals;
using namespace std::placeholders;

SimpleTfKinematics::SimpleTfKinematics(const std::string &name) : Node(name), x_increment(0.05f), last_x(0.0f), rotations_counter(0) {
    static_tf_broadcaster = std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);
    dynamic_tf_broadcaster = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

    tf_buffer = std::make_unique<tf2_ros::Buffer> (get_clock());
    tf_listener = std::make_shared<tf2_ros::TransformListener> (*tf_buffer);

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
    static_transform_stamped.transform.rotation.w = 1.0;

    static_tf_broadcaster -> sendTransform(static_transform_stamped);

    RCLCPP_INFO_STREAM(get_logger(), "Publishing static transform: "
                                     << static_transform_stamped.header.frame_id << " & " << static_transform_stamped.child_frame_id);

    timer = create_wall_timer(0.1s, std::bind(&SimpleTfKinematics::timer_callback, this));

    get_transform_srv = create_service<bumperbot_msgs::srv::GetTransform>("get_transform", 
        std::bind(&SimpleTfKinematics::get_transform_callback, this, _1, _2));

    last_orientation.setRPY(0, 0,0);
    orientation_increment.setRPY(0, 0, 0.05);
}

void SimpleTfKinematics::timer_callback() {
    dynamic_transform_stamped.header.stamp = get_clock() -> now();
    dynamic_transform_stamped.header.frame_id = "odom";
    dynamic_transform_stamped.child_frame_id = "bumperbot_base";

    dynamic_transform_stamped.transform.translation.x = last_x + x_increment;
    dynamic_transform_stamped.transform.translation.y = 0.0f;
    dynamic_transform_stamped.transform.translation.z = 0.0f;

    tf2::Quaternion q;
    q = last_orientation * orientation_increment; q.normalize();

    // Base and frame have the same orientation
    dynamic_transform_stamped.transform.rotation.x = q.x();
    dynamic_transform_stamped.transform.rotation.y = q.y();
    dynamic_transform_stamped.transform.rotation.z = q.z();
    dynamic_transform_stamped.transform.rotation.w = q.w();

    dynamic_tf_broadcaster -> sendTransform(dynamic_transform_stamped);

    last_x = dynamic_transform_stamped.transform.translation.x;
    rotations_counter++;
    last_orientation = q;

    if(rotations_counter >= 100) {
        orientation_increment = orientation_increment.inverse();
        rotations_counter = 0;
    }
}

bool SimpleTfKinematics::get_transform_callback(const std::shared_ptr<bumperbot_msgs::srv::GetTransform::Request> req,
                                    const std::shared_ptr<bumperbot_msgs::srv::GetTransform::Response> resp){
    RCLCPP_INFO_STREAM(get_logger(), "Requested Transform Between" << req -> frame_id << " & " << req -> child_frame_id);
    geometry_msgs::msg::TransformStamped requested_transform;

    try{
        requested_transform = tf_buffer -> lookupTransform(req -> frame_id, req -> child_frame_id, tf2::TimePointZero);   
    } catch (tf2::TransformException &e){
        RCLCPP_ERROR_STREAM(get_logger(), "An error ocurred while transforming from " << req -> frame_id << " & " << req -> child_frame_id);
        resp -> success = false;
        return true; 
    }

    resp -> success = true;
    return true;
}

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<SimpleTfKinematics>("simple_tf_kinematics");
    rclcpp::spin(node);
    rclcpp::shutdown();

    return 0;
}
