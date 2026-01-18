#include <rclcpp/rclcpp.hpp>
#include <nav2_msgs/action/navigate_to_pose.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>

using NavigateToPose = nav2_msgs::action::NavigateToPose;
using GoalHandle = rclcpp_action::ClientGoalHandle<NavigateToPose>;

class SimpleNavNode : public rclcpp::Node {
public:
    SimpleNavNode() : Node("simple_nav_node") {
        client_ = rclcpp_action::create_client<NavigateToPose>(this, "navigate_to_pose");
        if (!client_->wait_for_action_server(std::chrono::seconds(10))) {
            RCLCPP_ERROR(this->get_logger(), "Action server not available");
        }
        sendGoal();
    }

private:
    void sendGoal() {
        auto goal_msg = NavigateToPose::Goal();
        goal_msg.pose.header.frame_id = "map";
        goal_msg.pose.header.stamp = this->now();
        goal_msg.pose.pose.position.x = 1.0;  // Simple waypoint: move to (1.0, 0.5, 0.0)
        goal_msg.pose.pose.position.y = 0.5;
        goal_msg.pose.pose.orientation.w = 1.0;  // Facing forward

        RCLCPP_INFO(this->get_logger(), "Sending goal");
        auto send_goal_options = rclcpp_action::Client<NavigateToPose>::SendGoalOptions();
        send_goal_options.goal_response_callback = std::bind(&SimpleNavNode::goalResponseCallback, this, std::placeholders::_1);
        send_goal_options.result_callback = std::bind(&SimpleNavNode::resultCallback, this, std::placeholders::_1);
        client_->async_send_goal(goal_msg, send_goal_options);
    }

    void goalResponseCallback(const GoalHandle::SharedPtr &goal_handle) {
        if (!goal_handle) {
            RCLCPP_ERROR(this->get_logger(), "Goal rejected");
        } else {
            RCLCPP_INFO(this->get_logger(), "Goal accepted");
        }
    }

    void resultCallback(const GoalHandle::WrappedResult &result) {
        switch (result.code) {
            case rclcpp_action::ResultCode::SUCCEEDED:
                RCLCPP_INFO(this->get_logger(), "Navigation succeeded!");
                break;
            case rclcpp_action::ResultCode::ABORTED:
                RCLCPP_ERROR(this->get_logger(), "Navigation aborted");
                break;
            case rclcpp_action::ResultCode::CANCELED:
                RCLCPP_ERROR(this->get_logger(), "Navigation canceled");
                break;
            default:
                RCLCPP_ERROR(this->get_logger(), "Unknown result code");
        }
    }

    rclcpp_action::Client<NavigateToPose>::SharedPtr client_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<SimpleNavNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
