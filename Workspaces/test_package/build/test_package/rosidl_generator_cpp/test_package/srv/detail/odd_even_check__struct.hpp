// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from test_package:srv/OddEvenCheck.idl
// generated code does not contain a copyright notice

#ifndef TEST_PACKAGE__SRV__DETAIL__ODD_EVEN_CHECK__STRUCT_HPP_
#define TEST_PACKAGE__SRV__DETAIL__ODD_EVEN_CHECK__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__test_package__srv__OddEvenCheck_Request __attribute__((deprecated))
#else
# define DEPRECATED__test_package__srv__OddEvenCheck_Request __declspec(deprecated)
#endif

namespace test_package
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct OddEvenCheck_Request_
{
  using Type = OddEvenCheck_Request_<ContainerAllocator>;

  explicit OddEvenCheck_Request_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->number = 0ll;
    }
  }

  explicit OddEvenCheck_Request_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_alloc;
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->number = 0ll;
    }
  }

  // field types and members
  using _number_type =
    int64_t;
  _number_type number;

  // setters for named parameter idiom
  Type & set__number(
    const int64_t & _arg)
  {
    this->number = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    test_package::srv::OddEvenCheck_Request_<ContainerAllocator> *;
  using ConstRawPtr =
    const test_package::srv::OddEvenCheck_Request_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<test_package::srv::OddEvenCheck_Request_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<test_package::srv::OddEvenCheck_Request_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      test_package::srv::OddEvenCheck_Request_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<test_package::srv::OddEvenCheck_Request_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      test_package::srv::OddEvenCheck_Request_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<test_package::srv::OddEvenCheck_Request_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<test_package::srv::OddEvenCheck_Request_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<test_package::srv::OddEvenCheck_Request_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__test_package__srv__OddEvenCheck_Request
    std::shared_ptr<test_package::srv::OddEvenCheck_Request_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__test_package__srv__OddEvenCheck_Request
    std::shared_ptr<test_package::srv::OddEvenCheck_Request_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const OddEvenCheck_Request_ & other) const
  {
    if (this->number != other.number) {
      return false;
    }
    return true;
  }
  bool operator!=(const OddEvenCheck_Request_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct OddEvenCheck_Request_

// alias to use template instance with default allocator
using OddEvenCheck_Request =
  test_package::srv::OddEvenCheck_Request_<std::allocator<void>>;

// constant definitions

}  // namespace srv

}  // namespace test_package


#ifndef _WIN32
# define DEPRECATED__test_package__srv__OddEvenCheck_Response __attribute__((deprecated))
#else
# define DEPRECATED__test_package__srv__OddEvenCheck_Response __declspec(deprecated)
#endif

namespace test_package
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct OddEvenCheck_Response_
{
  using Type = OddEvenCheck_Response_<ContainerAllocator>;

  explicit OddEvenCheck_Response_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->decision = "";
    }
  }

  explicit OddEvenCheck_Response_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : decision(_alloc)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->decision = "";
    }
  }

  // field types and members
  using _decision_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _decision_type decision;

  // setters for named parameter idiom
  Type & set__decision(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->decision = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    test_package::srv::OddEvenCheck_Response_<ContainerAllocator> *;
  using ConstRawPtr =
    const test_package::srv::OddEvenCheck_Response_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<test_package::srv::OddEvenCheck_Response_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<test_package::srv::OddEvenCheck_Response_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      test_package::srv::OddEvenCheck_Response_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<test_package::srv::OddEvenCheck_Response_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      test_package::srv::OddEvenCheck_Response_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<test_package::srv::OddEvenCheck_Response_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<test_package::srv::OddEvenCheck_Response_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<test_package::srv::OddEvenCheck_Response_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__test_package__srv__OddEvenCheck_Response
    std::shared_ptr<test_package::srv::OddEvenCheck_Response_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__test_package__srv__OddEvenCheck_Response
    std::shared_ptr<test_package::srv::OddEvenCheck_Response_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const OddEvenCheck_Response_ & other) const
  {
    if (this->decision != other.decision) {
      return false;
    }
    return true;
  }
  bool operator!=(const OddEvenCheck_Response_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct OddEvenCheck_Response_

// alias to use template instance with default allocator
using OddEvenCheck_Response =
  test_package::srv::OddEvenCheck_Response_<std::allocator<void>>;

// constant definitions

}  // namespace srv

}  // namespace test_package

namespace test_package
{

namespace srv
{

struct OddEvenCheck
{
  using Request = test_package::srv::OddEvenCheck_Request;
  using Response = test_package::srv::OddEvenCheck_Response;
};

}  // namespace srv

}  // namespace test_package

#endif  // TEST_PACKAGE__SRV__DETAIL__ODD_EVEN_CHECK__STRUCT_HPP_
