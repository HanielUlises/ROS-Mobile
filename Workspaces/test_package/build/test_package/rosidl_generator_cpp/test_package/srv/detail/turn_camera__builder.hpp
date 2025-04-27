// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from test_package:srv/TurnCamera.idl
// generated code does not contain a copyright notice

#ifndef TEST_PACKAGE__SRV__DETAIL__TURN_CAMERA__BUILDER_HPP_
#define TEST_PACKAGE__SRV__DETAIL__TURN_CAMERA__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "test_package/srv/detail/turn_camera__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace test_package
{

namespace srv
{

namespace builder
{

class Init_TurnCamera_Request_degree_turn
{
public:
  Init_TurnCamera_Request_degree_turn()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::test_package::srv::TurnCamera_Request degree_turn(::test_package::srv::TurnCamera_Request::_degree_turn_type arg)
  {
    msg_.degree_turn = std::move(arg);
    return std::move(msg_);
  }

private:
  ::test_package::srv::TurnCamera_Request msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::test_package::srv::TurnCamera_Request>()
{
  return test_package::srv::builder::Init_TurnCamera_Request_degree_turn();
}

}  // namespace test_package


namespace test_package
{

namespace srv
{

namespace builder
{

class Init_TurnCamera_Response_camera_image
{
public:
  Init_TurnCamera_Response_camera_image()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::test_package::srv::TurnCamera_Response camera_image(::test_package::srv::TurnCamera_Response::_camera_image_type arg)
  {
    msg_.camera_image = std::move(arg);
    return std::move(msg_);
  }

private:
  ::test_package::srv::TurnCamera_Response msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::test_package::srv::TurnCamera_Response>()
{
  return test_package::srv::builder::Init_TurnCamera_Response_camera_image();
}

}  // namespace test_package

#endif  // TEST_PACKAGE__SRV__DETAIL__TURN_CAMERA__BUILDER_HPP_
