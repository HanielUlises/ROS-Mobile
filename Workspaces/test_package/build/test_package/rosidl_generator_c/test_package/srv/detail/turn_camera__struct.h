// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from test_package:srv/TurnCamera.idl
// generated code does not contain a copyright notice

#ifndef TEST_PACKAGE__SRV__DETAIL__TURN_CAMERA__STRUCT_H_
#define TEST_PACKAGE__SRV__DETAIL__TURN_CAMERA__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

/// Struct defined in srv/TurnCamera in the package test_package.
typedef struct test_package__srv__TurnCamera_Request
{
  float degree_turn;
} test_package__srv__TurnCamera_Request;

// Struct for a sequence of test_package__srv__TurnCamera_Request.
typedef struct test_package__srv__TurnCamera_Request__Sequence
{
  test_package__srv__TurnCamera_Request * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} test_package__srv__TurnCamera_Request__Sequence;


// Constants defined in the message

// Include directives for member types
// Member 'camera_image'
#include "sensor_msgs/msg/detail/image__struct.h"

/// Struct defined in srv/TurnCamera in the package test_package.
typedef struct test_package__srv__TurnCamera_Response
{
  sensor_msgs__msg__Image camera_image;
} test_package__srv__TurnCamera_Response;

// Struct for a sequence of test_package__srv__TurnCamera_Response.
typedef struct test_package__srv__TurnCamera_Response__Sequence
{
  test_package__srv__TurnCamera_Response * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} test_package__srv__TurnCamera_Response__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // TEST_PACKAGE__SRV__DETAIL__TURN_CAMERA__STRUCT_H_
