// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from test_package:action/Navigate.idl
// generated code does not contain a copyright notice

#ifndef TEST_PACKAGE__ACTION__DETAIL__NAVIGATE__BUILDER_HPP_
#define TEST_PACKAGE__ACTION__DETAIL__NAVIGATE__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "test_package/action/detail/navigate__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace test_package
{

namespace action
{

namespace builder
{

class Init_Navigate_Goal_goal_point
{
public:
  Init_Navigate_Goal_goal_point()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::test_package::action::Navigate_Goal goal_point(::test_package::action::Navigate_Goal::_goal_point_type arg)
  {
    msg_.goal_point = std::move(arg);
    return std::move(msg_);
  }

private:
  ::test_package::action::Navigate_Goal msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::test_package::action::Navigate_Goal>()
{
  return test_package::action::builder::Init_Navigate_Goal_goal_point();
}

}  // namespace test_package


namespace test_package
{

namespace action
{

namespace builder
{

class Init_Navigate_Result_elapsed_time
{
public:
  Init_Navigate_Result_elapsed_time()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::test_package::action::Navigate_Result elapsed_time(::test_package::action::Navigate_Result::_elapsed_time_type arg)
  {
    msg_.elapsed_time = std::move(arg);
    return std::move(msg_);
  }

private:
  ::test_package::action::Navigate_Result msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::test_package::action::Navigate_Result>()
{
  return test_package::action::builder::Init_Navigate_Result_elapsed_time();
}

}  // namespace test_package


namespace test_package
{

namespace action
{

namespace builder
{

class Init_Navigate_Feedback_distance_to_point
{
public:
  Init_Navigate_Feedback_distance_to_point()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::test_package::action::Navigate_Feedback distance_to_point(::test_package::action::Navigate_Feedback::_distance_to_point_type arg)
  {
    msg_.distance_to_point = std::move(arg);
    return std::move(msg_);
  }

private:
  ::test_package::action::Navigate_Feedback msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::test_package::action::Navigate_Feedback>()
{
  return test_package::action::builder::Init_Navigate_Feedback_distance_to_point();
}

}  // namespace test_package


namespace test_package
{

namespace action
{

namespace builder
{

class Init_Navigate_SendGoal_Request_goal
{
public:
  explicit Init_Navigate_SendGoal_Request_goal(::test_package::action::Navigate_SendGoal_Request & msg)
  : msg_(msg)
  {}
  ::test_package::action::Navigate_SendGoal_Request goal(::test_package::action::Navigate_SendGoal_Request::_goal_type arg)
  {
    msg_.goal = std::move(arg);
    return std::move(msg_);
  }

private:
  ::test_package::action::Navigate_SendGoal_Request msg_;
};

class Init_Navigate_SendGoal_Request_goal_id
{
public:
  Init_Navigate_SendGoal_Request_goal_id()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_Navigate_SendGoal_Request_goal goal_id(::test_package::action::Navigate_SendGoal_Request::_goal_id_type arg)
  {
    msg_.goal_id = std::move(arg);
    return Init_Navigate_SendGoal_Request_goal(msg_);
  }

private:
  ::test_package::action::Navigate_SendGoal_Request msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::test_package::action::Navigate_SendGoal_Request>()
{
  return test_package::action::builder::Init_Navigate_SendGoal_Request_goal_id();
}

}  // namespace test_package


namespace test_package
{

namespace action
{

namespace builder
{

class Init_Navigate_SendGoal_Response_stamp
{
public:
  explicit Init_Navigate_SendGoal_Response_stamp(::test_package::action::Navigate_SendGoal_Response & msg)
  : msg_(msg)
  {}
  ::test_package::action::Navigate_SendGoal_Response stamp(::test_package::action::Navigate_SendGoal_Response::_stamp_type arg)
  {
    msg_.stamp = std::move(arg);
    return std::move(msg_);
  }

private:
  ::test_package::action::Navigate_SendGoal_Response msg_;
};

class Init_Navigate_SendGoal_Response_accepted
{
public:
  Init_Navigate_SendGoal_Response_accepted()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_Navigate_SendGoal_Response_stamp accepted(::test_package::action::Navigate_SendGoal_Response::_accepted_type arg)
  {
    msg_.accepted = std::move(arg);
    return Init_Navigate_SendGoal_Response_stamp(msg_);
  }

private:
  ::test_package::action::Navigate_SendGoal_Response msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::test_package::action::Navigate_SendGoal_Response>()
{
  return test_package::action::builder::Init_Navigate_SendGoal_Response_accepted();
}

}  // namespace test_package


namespace test_package
{

namespace action
{

namespace builder
{

class Init_Navigate_GetResult_Request_goal_id
{
public:
  Init_Navigate_GetResult_Request_goal_id()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::test_package::action::Navigate_GetResult_Request goal_id(::test_package::action::Navigate_GetResult_Request::_goal_id_type arg)
  {
    msg_.goal_id = std::move(arg);
    return std::move(msg_);
  }

private:
  ::test_package::action::Navigate_GetResult_Request msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::test_package::action::Navigate_GetResult_Request>()
{
  return test_package::action::builder::Init_Navigate_GetResult_Request_goal_id();
}

}  // namespace test_package


namespace test_package
{

namespace action
{

namespace builder
{

class Init_Navigate_GetResult_Response_result
{
public:
  explicit Init_Navigate_GetResult_Response_result(::test_package::action::Navigate_GetResult_Response & msg)
  : msg_(msg)
  {}
  ::test_package::action::Navigate_GetResult_Response result(::test_package::action::Navigate_GetResult_Response::_result_type arg)
  {
    msg_.result = std::move(arg);
    return std::move(msg_);
  }

private:
  ::test_package::action::Navigate_GetResult_Response msg_;
};

class Init_Navigate_GetResult_Response_status
{
public:
  Init_Navigate_GetResult_Response_status()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_Navigate_GetResult_Response_result status(::test_package::action::Navigate_GetResult_Response::_status_type arg)
  {
    msg_.status = std::move(arg);
    return Init_Navigate_GetResult_Response_result(msg_);
  }

private:
  ::test_package::action::Navigate_GetResult_Response msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::test_package::action::Navigate_GetResult_Response>()
{
  return test_package::action::builder::Init_Navigate_GetResult_Response_status();
}

}  // namespace test_package


namespace test_package
{

namespace action
{

namespace builder
{

class Init_Navigate_FeedbackMessage_feedback
{
public:
  explicit Init_Navigate_FeedbackMessage_feedback(::test_package::action::Navigate_FeedbackMessage & msg)
  : msg_(msg)
  {}
  ::test_package::action::Navigate_FeedbackMessage feedback(::test_package::action::Navigate_FeedbackMessage::_feedback_type arg)
  {
    msg_.feedback = std::move(arg);
    return std::move(msg_);
  }

private:
  ::test_package::action::Navigate_FeedbackMessage msg_;
};

class Init_Navigate_FeedbackMessage_goal_id
{
public:
  Init_Navigate_FeedbackMessage_goal_id()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_Navigate_FeedbackMessage_feedback goal_id(::test_package::action::Navigate_FeedbackMessage::_goal_id_type arg)
  {
    msg_.goal_id = std::move(arg);
    return Init_Navigate_FeedbackMessage_feedback(msg_);
  }

private:
  ::test_package::action::Navigate_FeedbackMessage msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::test_package::action::Navigate_FeedbackMessage>()
{
  return test_package::action::builder::Init_Navigate_FeedbackMessage_goal_id();
}

}  // namespace test_package

#endif  // TEST_PACKAGE__ACTION__DETAIL__NAVIGATE__BUILDER_HPP_
