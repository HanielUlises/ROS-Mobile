// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from test_package:srv/OddEvenCheck.idl
// generated code does not contain a copyright notice

#ifndef TEST_PACKAGE__SRV__DETAIL__ODD_EVEN_CHECK__BUILDER_HPP_
#define TEST_PACKAGE__SRV__DETAIL__ODD_EVEN_CHECK__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "test_package/srv/detail/odd_even_check__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace test_package
{

namespace srv
{

namespace builder
{

class Init_OddEvenCheck_Request_number
{
public:
  Init_OddEvenCheck_Request_number()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::test_package::srv::OddEvenCheck_Request number(::test_package::srv::OddEvenCheck_Request::_number_type arg)
  {
    msg_.number = std::move(arg);
    return std::move(msg_);
  }

private:
  ::test_package::srv::OddEvenCheck_Request msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::test_package::srv::OddEvenCheck_Request>()
{
  return test_package::srv::builder::Init_OddEvenCheck_Request_number();
}

}  // namespace test_package


namespace test_package
{

namespace srv
{

namespace builder
{

class Init_OddEvenCheck_Response_decision
{
public:
  Init_OddEvenCheck_Response_decision()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::test_package::srv::OddEvenCheck_Response decision(::test_package::srv::OddEvenCheck_Response::_decision_type arg)
  {
    msg_.decision = std::move(arg);
    return std::move(msg_);
  }

private:
  ::test_package::srv::OddEvenCheck_Response msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::test_package::srv::OddEvenCheck_Response>()
{
  return test_package::srv::builder::Init_OddEvenCheck_Response_decision();
}

}  // namespace test_package

#endif  // TEST_PACKAGE__SRV__DETAIL__ODD_EVEN_CHECK__BUILDER_HPP_
