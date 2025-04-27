// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from test_package:srv/TurnCamera.idl
// generated code does not contain a copyright notice

#ifndef TEST_PACKAGE__SRV__DETAIL__TURN_CAMERA__STRUCT_HPP_
#define TEST_PACKAGE__SRV__DETAIL__TURN_CAMERA__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__test_package__srv__TurnCamera_Request __attribute__((deprecated))
#else
# define DEPRECATED__test_package__srv__TurnCamera_Request __declspec(deprecated)
#endif

namespace test_package
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct TurnCamera_Request_
{
  using Type = TurnCamera_Request_<ContainerAllocator>;

  explicit TurnCamera_Request_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->degree_turn = 0.0f;
    }
  }

  explicit TurnCamera_Request_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_alloc;
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->degree_turn = 0.0f;
    }
  }

  // field types and members
  using _degree_turn_type =
    float;
  _degree_turn_type degree_turn;

  // setters for named parameter idiom
  Type & set__degree_turn(
    const float & _arg)
  {
    this->degree_turn = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    test_package::srv::TurnCamera_Request_<ContainerAllocator> *;
  using ConstRawPtr =
    const test_package::srv::TurnCamera_Request_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<test_package::srv::TurnCamera_Request_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<test_package::srv::TurnCamera_Request_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      test_package::srv::TurnCamera_Request_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<test_package::srv::TurnCamera_Request_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      test_package::srv::TurnCamera_Request_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<test_package::srv::TurnCamera_Request_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<test_package::srv::TurnCamera_Request_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<test_package::srv::TurnCamera_Request_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__test_package__srv__TurnCamera_Request
    std::shared_ptr<test_package::srv::TurnCamera_Request_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__test_package__srv__TurnCamera_Request
    std::shared_ptr<test_package::srv::TurnCamera_Request_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const TurnCamera_Request_ & other) const
  {
    if (this->degree_turn != other.degree_turn) {
      return false;
    }
    return true;
  }
  bool operator!=(const TurnCamera_Request_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct TurnCamera_Request_

// alias to use template instance with default allocator
using TurnCamera_Request =
  test_package::srv::TurnCamera_Request_<std::allocator<void>>;

// constant definitions

}  // namespace srv

}  // namespace test_package


// Include directives for member types
// Member 'camera_image'
#include "sensor_msgs/msg/detail/image__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__test_package__srv__TurnCamera_Response __attribute__((deprecated))
#else
# define DEPRECATED__test_package__srv__TurnCamera_Response __declspec(deprecated)
#endif

namespace test_package
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct TurnCamera_Response_
{
  using Type = TurnCamera_Response_<ContainerAllocator>;

  explicit TurnCamera_Response_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : camera_image(_init)
  {
    (void)_init;
  }

  explicit TurnCamera_Response_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : camera_image(_alloc, _init)
  {
    (void)_init;
  }

  // field types and members
  using _camera_image_type =
    sensor_msgs::msg::Image_<ContainerAllocator>;
  _camera_image_type camera_image;

  // setters for named parameter idiom
  Type & set__camera_image(
    const sensor_msgs::msg::Image_<ContainerAllocator> & _arg)
  {
    this->camera_image = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    test_package::srv::TurnCamera_Response_<ContainerAllocator> *;
  using ConstRawPtr =
    const test_package::srv::TurnCamera_Response_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<test_package::srv::TurnCamera_Response_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<test_package::srv::TurnCamera_Response_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      test_package::srv::TurnCamera_Response_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<test_package::srv::TurnCamera_Response_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      test_package::srv::TurnCamera_Response_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<test_package::srv::TurnCamera_Response_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<test_package::srv::TurnCamera_Response_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<test_package::srv::TurnCamera_Response_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__test_package__srv__TurnCamera_Response
    std::shared_ptr<test_package::srv::TurnCamera_Response_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__test_package__srv__TurnCamera_Response
    std::shared_ptr<test_package::srv::TurnCamera_Response_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const TurnCamera_Response_ & other) const
  {
    if (this->camera_image != other.camera_image) {
      return false;
    }
    return true;
  }
  bool operator!=(const TurnCamera_Response_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct TurnCamera_Response_

// alias to use template instance with default allocator
using TurnCamera_Response =
  test_package::srv::TurnCamera_Response_<std::allocator<void>>;

// constant definitions

}  // namespace srv

}  // namespace test_package

namespace test_package
{

namespace srv
{

struct TurnCamera
{
  using Request = test_package::srv::TurnCamera_Request;
  using Response = test_package::srv::TurnCamera_Response;
};

}  // namespace srv

}  // namespace test_package

#endif  // TEST_PACKAGE__SRV__DETAIL__TURN_CAMERA__STRUCT_HPP_
