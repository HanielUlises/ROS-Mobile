#include <chrono>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"

using namespace std::chrono_literals;

class VelPublisherTest : public rclcpp::Node
{
public:
  VelPublisherTest()
  : Node("vel_publisher_test")
  {
    publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
    timer_ = this->create_wall_timer(
      500ms, std::bind(&VelPublisherTest::timer_callback, this));
  }

private:
  void timer_callback()
  {
    auto msg = geometry_msgs::msg::Twist();
    msg.linear.x = 0.2;
    msg.angular.z = 0.1;
    RCLCPP_INFO(this->get_logger(), "Publishing cmd_vel: linear.x=%.2f angular.z=%.2f",
      msg.linear.x, msg.angular.z);
    publisher_->publish(msg);
  }

  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<VelPublisherTest>());
  rclcpp::shutdown();
  return 0;
}
