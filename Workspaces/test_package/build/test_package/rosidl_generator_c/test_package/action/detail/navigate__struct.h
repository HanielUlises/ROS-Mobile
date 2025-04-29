// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from test_package:action/Navigate.idl
// generated code does not contain a copyright notice

#ifndef TEST_PACKAGE__ACTION__DETAIL__NAVIGATE__STRUCT_H_
#define TEST_PACKAGE__ACTION__DETAIL__NAVIGATE__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'goal_point'
#include "geometry_msgs/msg/detail/point__struct.h"

/// Struct defined in action/Navigate in the package test_package.
typedef struct test_package__action__Navigate_Goal
{
  /// X, Y, Z
  geometry_msgs__msg__Point goal_point;
} test_package__action__Navigate_Goal;

// Struct for a sequence of test_package__action__Navigate_Goal.
typedef struct test_package__action__Navigate_Goal__Sequence
{
  test_package__action__Navigate_Goal * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} test_package__action__Navigate_Goal__Sequence;


// Constants defined in the message

/// Struct defined in action/Navigate in the package test_package.
typedef struct test_package__action__Navigate_Result
{
  float elapsed_time;
} test_package__action__Navigate_Result;

// Struct for a sequence of test_package__action__Navigate_Result.
typedef struct test_package__action__Navigate_Result__Sequence
{
  test_package__action__Navigate_Result * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} test_package__action__Navigate_Result__Sequence;


// Constants defined in the message

/// Struct defined in action/Navigate in the package test_package.
typedef struct test_package__action__Navigate_Feedback
{
  float distance_to_point;
} test_package__action__Navigate_Feedback;

// Struct for a sequence of test_package__action__Navigate_Feedback.
typedef struct test_package__action__Navigate_Feedback__Sequence
{
  test_package__action__Navigate_Feedback * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} test_package__action__Navigate_Feedback__Sequence;


// Constants defined in the message

// Include directives for member types
// Member 'goal_id'
#include "unique_identifier_msgs/msg/detail/uuid__struct.h"
// Member 'goal'
#include "test_package/action/detail/navigate__struct.h"

/// Struct defined in action/Navigate in the package test_package.
typedef struct test_package__action__Navigate_SendGoal_Request
{
  unique_identifier_msgs__msg__UUID goal_id;
  test_package__action__Navigate_Goal goal;
} test_package__action__Navigate_SendGoal_Request;

// Struct for a sequence of test_package__action__Navigate_SendGoal_Request.
typedef struct test_package__action__Navigate_SendGoal_Request__Sequence
{
  test_package__action__Navigate_SendGoal_Request * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} test_package__action__Navigate_SendGoal_Request__Sequence;


// Constants defined in the message

// Include directives for member types
// Member 'stamp'
#include "builtin_interfaces/msg/detail/time__struct.h"

/// Struct defined in action/Navigate in the package test_package.
typedef struct test_package__action__Navigate_SendGoal_Response
{
  bool accepted;
  builtin_interfaces__msg__Time stamp;
} test_package__action__Navigate_SendGoal_Response;

// Struct for a sequence of test_package__action__Navigate_SendGoal_Response.
typedef struct test_package__action__Navigate_SendGoal_Response__Sequence
{
  test_package__action__Navigate_SendGoal_Response * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} test_package__action__Navigate_SendGoal_Response__Sequence;


// Constants defined in the message

// Include directives for member types
// Member 'goal_id'
// already included above
// #include "unique_identifier_msgs/msg/detail/uuid__struct.h"

/// Struct defined in action/Navigate in the package test_package.
typedef struct test_package__action__Navigate_GetResult_Request
{
  unique_identifier_msgs__msg__UUID goal_id;
} test_package__action__Navigate_GetResult_Request;

// Struct for a sequence of test_package__action__Navigate_GetResult_Request.
typedef struct test_package__action__Navigate_GetResult_Request__Sequence
{
  test_package__action__Navigate_GetResult_Request * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} test_package__action__Navigate_GetResult_Request__Sequence;


// Constants defined in the message

// Include directives for member types
// Member 'result'
// already included above
// #include "test_package/action/detail/navigate__struct.h"

/// Struct defined in action/Navigate in the package test_package.
typedef struct test_package__action__Navigate_GetResult_Response
{
  int8_t status;
  test_package__action__Navigate_Result result;
} test_package__action__Navigate_GetResult_Response;

// Struct for a sequence of test_package__action__Navigate_GetResult_Response.
typedef struct test_package__action__Navigate_GetResult_Response__Sequence
{
  test_package__action__Navigate_GetResult_Response * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} test_package__action__Navigate_GetResult_Response__Sequence;


// Constants defined in the message

// Include directives for member types
// Member 'goal_id'
// already included above
// #include "unique_identifier_msgs/msg/detail/uuid__struct.h"
// Member 'feedback'
// already included above
// #include "test_package/action/detail/navigate__struct.h"

/// Struct defined in action/Navigate in the package test_package.
typedef struct test_package__action__Navigate_FeedbackMessage
{
  unique_identifier_msgs__msg__UUID goal_id;
  test_package__action__Navigate_Feedback feedback;
} test_package__action__Navigate_FeedbackMessage;

// Struct for a sequence of test_package__action__Navigate_FeedbackMessage.
typedef struct test_package__action__Navigate_FeedbackMessage__Sequence
{
  test_package__action__Navigate_FeedbackMessage * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} test_package__action__Navigate_FeedbackMessage__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // TEST_PACKAGE__ACTION__DETAIL__NAVIGATE__STRUCT_H_
