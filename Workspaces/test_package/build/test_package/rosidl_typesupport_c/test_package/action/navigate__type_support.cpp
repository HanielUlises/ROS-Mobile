// generated from rosidl_typesupport_c/resource/idl__type_support.cpp.em
// with input from test_package:action/Navigate.idl
// generated code does not contain a copyright notice

#include "cstddef"
#include "rosidl_runtime_c/message_type_support_struct.h"
#include "test_package/action/detail/navigate__struct.h"
#include "test_package/action/detail/navigate__type_support.h"
#include "rosidl_typesupport_c/identifier.h"
#include "rosidl_typesupport_c/message_type_support_dispatch.h"
#include "rosidl_typesupport_c/type_support_map.h"
#include "rosidl_typesupport_c/visibility_control.h"
#include "rosidl_typesupport_interface/macros.h"

namespace test_package
{

namespace action
{

namespace rosidl_typesupport_c
{

typedef struct _Navigate_Goal_type_support_ids_t
{
  const char * typesupport_identifier[2];
} _Navigate_Goal_type_support_ids_t;

static const _Navigate_Goal_type_support_ids_t _Navigate_Goal_message_typesupport_ids = {
  {
    "rosidl_typesupport_fastrtps_c",  // ::rosidl_typesupport_fastrtps_c::typesupport_identifier,
    "rosidl_typesupport_introspection_c",  // ::rosidl_typesupport_introspection_c::typesupport_identifier,
  }
};

typedef struct _Navigate_Goal_type_support_symbol_names_t
{
  const char * symbol_name[2];
} _Navigate_Goal_type_support_symbol_names_t;

#define STRINGIFY_(s) #s
#define STRINGIFY(s) STRINGIFY_(s)

static const _Navigate_Goal_type_support_symbol_names_t _Navigate_Goal_message_typesupport_symbol_names = {
  {
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, test_package, action, Navigate_Goal)),
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, test_package, action, Navigate_Goal)),
  }
};

typedef struct _Navigate_Goal_type_support_data_t
{
  void * data[2];
} _Navigate_Goal_type_support_data_t;

static _Navigate_Goal_type_support_data_t _Navigate_Goal_message_typesupport_data = {
  {
    0,  // will store the shared library later
    0,  // will store the shared library later
  }
};

static const type_support_map_t _Navigate_Goal_message_typesupport_map = {
  2,
  "test_package",
  &_Navigate_Goal_message_typesupport_ids.typesupport_identifier[0],
  &_Navigate_Goal_message_typesupport_symbol_names.symbol_name[0],
  &_Navigate_Goal_message_typesupport_data.data[0],
};

static const rosidl_message_type_support_t Navigate_Goal_message_type_support_handle = {
  rosidl_typesupport_c__typesupport_identifier,
  reinterpret_cast<const type_support_map_t *>(&_Navigate_Goal_message_typesupport_map),
  rosidl_typesupport_c__get_message_typesupport_handle_function,
};

}  // namespace rosidl_typesupport_c

}  // namespace action

}  // namespace test_package

#ifdef __cplusplus
extern "C"
{
#endif

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_c, test_package, action, Navigate_Goal)() {
  return &::test_package::action::rosidl_typesupport_c::Navigate_Goal_message_type_support_handle;
}

#ifdef __cplusplus
}
#endif

// already included above
// #include "cstddef"
// already included above
// #include "rosidl_runtime_c/message_type_support_struct.h"
// already included above
// #include "test_package/action/detail/navigate__struct.h"
// already included above
// #include "test_package/action/detail/navigate__type_support.h"
// already included above
// #include "rosidl_typesupport_c/identifier.h"
// already included above
// #include "rosidl_typesupport_c/message_type_support_dispatch.h"
// already included above
// #include "rosidl_typesupport_c/type_support_map.h"
// already included above
// #include "rosidl_typesupport_c/visibility_control.h"
// already included above
// #include "rosidl_typesupport_interface/macros.h"

namespace test_package
{

namespace action
{

namespace rosidl_typesupport_c
{

typedef struct _Navigate_Result_type_support_ids_t
{
  const char * typesupport_identifier[2];
} _Navigate_Result_type_support_ids_t;

static const _Navigate_Result_type_support_ids_t _Navigate_Result_message_typesupport_ids = {
  {
    "rosidl_typesupport_fastrtps_c",  // ::rosidl_typesupport_fastrtps_c::typesupport_identifier,
    "rosidl_typesupport_introspection_c",  // ::rosidl_typesupport_introspection_c::typesupport_identifier,
  }
};

typedef struct _Navigate_Result_type_support_symbol_names_t
{
  const char * symbol_name[2];
} _Navigate_Result_type_support_symbol_names_t;

#define STRINGIFY_(s) #s
#define STRINGIFY(s) STRINGIFY_(s)

static const _Navigate_Result_type_support_symbol_names_t _Navigate_Result_message_typesupport_symbol_names = {
  {
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, test_package, action, Navigate_Result)),
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, test_package, action, Navigate_Result)),
  }
};

typedef struct _Navigate_Result_type_support_data_t
{
  void * data[2];
} _Navigate_Result_type_support_data_t;

static _Navigate_Result_type_support_data_t _Navigate_Result_message_typesupport_data = {
  {
    0,  // will store the shared library later
    0,  // will store the shared library later
  }
};

static const type_support_map_t _Navigate_Result_message_typesupport_map = {
  2,
  "test_package",
  &_Navigate_Result_message_typesupport_ids.typesupport_identifier[0],
  &_Navigate_Result_message_typesupport_symbol_names.symbol_name[0],
  &_Navigate_Result_message_typesupport_data.data[0],
};

static const rosidl_message_type_support_t Navigate_Result_message_type_support_handle = {
  rosidl_typesupport_c__typesupport_identifier,
  reinterpret_cast<const type_support_map_t *>(&_Navigate_Result_message_typesupport_map),
  rosidl_typesupport_c__get_message_typesupport_handle_function,
};

}  // namespace rosidl_typesupport_c

}  // namespace action

}  // namespace test_package

#ifdef __cplusplus
extern "C"
{
#endif

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_c, test_package, action, Navigate_Result)() {
  return &::test_package::action::rosidl_typesupport_c::Navigate_Result_message_type_support_handle;
}

#ifdef __cplusplus
}
#endif

// already included above
// #include "cstddef"
// already included above
// #include "rosidl_runtime_c/message_type_support_struct.h"
// already included above
// #include "test_package/action/detail/navigate__struct.h"
// already included above
// #include "test_package/action/detail/navigate__type_support.h"
// already included above
// #include "rosidl_typesupport_c/identifier.h"
// already included above
// #include "rosidl_typesupport_c/message_type_support_dispatch.h"
// already included above
// #include "rosidl_typesupport_c/type_support_map.h"
// already included above
// #include "rosidl_typesupport_c/visibility_control.h"
// already included above
// #include "rosidl_typesupport_interface/macros.h"

namespace test_package
{

namespace action
{

namespace rosidl_typesupport_c
{

typedef struct _Navigate_Feedback_type_support_ids_t
{
  const char * typesupport_identifier[2];
} _Navigate_Feedback_type_support_ids_t;

static const _Navigate_Feedback_type_support_ids_t _Navigate_Feedback_message_typesupport_ids = {
  {
    "rosidl_typesupport_fastrtps_c",  // ::rosidl_typesupport_fastrtps_c::typesupport_identifier,
    "rosidl_typesupport_introspection_c",  // ::rosidl_typesupport_introspection_c::typesupport_identifier,
  }
};

typedef struct _Navigate_Feedback_type_support_symbol_names_t
{
  const char * symbol_name[2];
} _Navigate_Feedback_type_support_symbol_names_t;

#define STRINGIFY_(s) #s
#define STRINGIFY(s) STRINGIFY_(s)

static const _Navigate_Feedback_type_support_symbol_names_t _Navigate_Feedback_message_typesupport_symbol_names = {
  {
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, test_package, action, Navigate_Feedback)),
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, test_package, action, Navigate_Feedback)),
  }
};

typedef struct _Navigate_Feedback_type_support_data_t
{
  void * data[2];
} _Navigate_Feedback_type_support_data_t;

static _Navigate_Feedback_type_support_data_t _Navigate_Feedback_message_typesupport_data = {
  {
    0,  // will store the shared library later
    0,  // will store the shared library later
  }
};

static const type_support_map_t _Navigate_Feedback_message_typesupport_map = {
  2,
  "test_package",
  &_Navigate_Feedback_message_typesupport_ids.typesupport_identifier[0],
  &_Navigate_Feedback_message_typesupport_symbol_names.symbol_name[0],
  &_Navigate_Feedback_message_typesupport_data.data[0],
};

static const rosidl_message_type_support_t Navigate_Feedback_message_type_support_handle = {
  rosidl_typesupport_c__typesupport_identifier,
  reinterpret_cast<const type_support_map_t *>(&_Navigate_Feedback_message_typesupport_map),
  rosidl_typesupport_c__get_message_typesupport_handle_function,
};

}  // namespace rosidl_typesupport_c

}  // namespace action

}  // namespace test_package

#ifdef __cplusplus
extern "C"
{
#endif

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_c, test_package, action, Navigate_Feedback)() {
  return &::test_package::action::rosidl_typesupport_c::Navigate_Feedback_message_type_support_handle;
}

#ifdef __cplusplus
}
#endif

// already included above
// #include "cstddef"
// already included above
// #include "rosidl_runtime_c/message_type_support_struct.h"
// already included above
// #include "test_package/action/detail/navigate__struct.h"
// already included above
// #include "test_package/action/detail/navigate__type_support.h"
// already included above
// #include "rosidl_typesupport_c/identifier.h"
// already included above
// #include "rosidl_typesupport_c/message_type_support_dispatch.h"
// already included above
// #include "rosidl_typesupport_c/type_support_map.h"
// already included above
// #include "rosidl_typesupport_c/visibility_control.h"
// already included above
// #include "rosidl_typesupport_interface/macros.h"

namespace test_package
{

namespace action
{

namespace rosidl_typesupport_c
{

typedef struct _Navigate_SendGoal_Request_type_support_ids_t
{
  const char * typesupport_identifier[2];
} _Navigate_SendGoal_Request_type_support_ids_t;

static const _Navigate_SendGoal_Request_type_support_ids_t _Navigate_SendGoal_Request_message_typesupport_ids = {
  {
    "rosidl_typesupport_fastrtps_c",  // ::rosidl_typesupport_fastrtps_c::typesupport_identifier,
    "rosidl_typesupport_introspection_c",  // ::rosidl_typesupport_introspection_c::typesupport_identifier,
  }
};

typedef struct _Navigate_SendGoal_Request_type_support_symbol_names_t
{
  const char * symbol_name[2];
} _Navigate_SendGoal_Request_type_support_symbol_names_t;

#define STRINGIFY_(s) #s
#define STRINGIFY(s) STRINGIFY_(s)

static const _Navigate_SendGoal_Request_type_support_symbol_names_t _Navigate_SendGoal_Request_message_typesupport_symbol_names = {
  {
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, test_package, action, Navigate_SendGoal_Request)),
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, test_package, action, Navigate_SendGoal_Request)),
  }
};

typedef struct _Navigate_SendGoal_Request_type_support_data_t
{
  void * data[2];
} _Navigate_SendGoal_Request_type_support_data_t;

static _Navigate_SendGoal_Request_type_support_data_t _Navigate_SendGoal_Request_message_typesupport_data = {
  {
    0,  // will store the shared library later
    0,  // will store the shared library later
  }
};

static const type_support_map_t _Navigate_SendGoal_Request_message_typesupport_map = {
  2,
  "test_package",
  &_Navigate_SendGoal_Request_message_typesupport_ids.typesupport_identifier[0],
  &_Navigate_SendGoal_Request_message_typesupport_symbol_names.symbol_name[0],
  &_Navigate_SendGoal_Request_message_typesupport_data.data[0],
};

static const rosidl_message_type_support_t Navigate_SendGoal_Request_message_type_support_handle = {
  rosidl_typesupport_c__typesupport_identifier,
  reinterpret_cast<const type_support_map_t *>(&_Navigate_SendGoal_Request_message_typesupport_map),
  rosidl_typesupport_c__get_message_typesupport_handle_function,
};

}  // namespace rosidl_typesupport_c

}  // namespace action

}  // namespace test_package

#ifdef __cplusplus
extern "C"
{
#endif

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_c, test_package, action, Navigate_SendGoal_Request)() {
  return &::test_package::action::rosidl_typesupport_c::Navigate_SendGoal_Request_message_type_support_handle;
}

#ifdef __cplusplus
}
#endif

// already included above
// #include "cstddef"
// already included above
// #include "rosidl_runtime_c/message_type_support_struct.h"
// already included above
// #include "test_package/action/detail/navigate__struct.h"
// already included above
// #include "test_package/action/detail/navigate__type_support.h"
// already included above
// #include "rosidl_typesupport_c/identifier.h"
// already included above
// #include "rosidl_typesupport_c/message_type_support_dispatch.h"
// already included above
// #include "rosidl_typesupport_c/type_support_map.h"
// already included above
// #include "rosidl_typesupport_c/visibility_control.h"
// already included above
// #include "rosidl_typesupport_interface/macros.h"

namespace test_package
{

namespace action
{

namespace rosidl_typesupport_c
{

typedef struct _Navigate_SendGoal_Response_type_support_ids_t
{
  const char * typesupport_identifier[2];
} _Navigate_SendGoal_Response_type_support_ids_t;

static const _Navigate_SendGoal_Response_type_support_ids_t _Navigate_SendGoal_Response_message_typesupport_ids = {
  {
    "rosidl_typesupport_fastrtps_c",  // ::rosidl_typesupport_fastrtps_c::typesupport_identifier,
    "rosidl_typesupport_introspection_c",  // ::rosidl_typesupport_introspection_c::typesupport_identifier,
  }
};

typedef struct _Navigate_SendGoal_Response_type_support_symbol_names_t
{
  const char * symbol_name[2];
} _Navigate_SendGoal_Response_type_support_symbol_names_t;

#define STRINGIFY_(s) #s
#define STRINGIFY(s) STRINGIFY_(s)

static const _Navigate_SendGoal_Response_type_support_symbol_names_t _Navigate_SendGoal_Response_message_typesupport_symbol_names = {
  {
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, test_package, action, Navigate_SendGoal_Response)),
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, test_package, action, Navigate_SendGoal_Response)),
  }
};

typedef struct _Navigate_SendGoal_Response_type_support_data_t
{
  void * data[2];
} _Navigate_SendGoal_Response_type_support_data_t;

static _Navigate_SendGoal_Response_type_support_data_t _Navigate_SendGoal_Response_message_typesupport_data = {
  {
    0,  // will store the shared library later
    0,  // will store the shared library later
  }
};

static const type_support_map_t _Navigate_SendGoal_Response_message_typesupport_map = {
  2,
  "test_package",
  &_Navigate_SendGoal_Response_message_typesupport_ids.typesupport_identifier[0],
  &_Navigate_SendGoal_Response_message_typesupport_symbol_names.symbol_name[0],
  &_Navigate_SendGoal_Response_message_typesupport_data.data[0],
};

static const rosidl_message_type_support_t Navigate_SendGoal_Response_message_type_support_handle = {
  rosidl_typesupport_c__typesupport_identifier,
  reinterpret_cast<const type_support_map_t *>(&_Navigate_SendGoal_Response_message_typesupport_map),
  rosidl_typesupport_c__get_message_typesupport_handle_function,
};

}  // namespace rosidl_typesupport_c

}  // namespace action

}  // namespace test_package

#ifdef __cplusplus
extern "C"
{
#endif

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_c, test_package, action, Navigate_SendGoal_Response)() {
  return &::test_package::action::rosidl_typesupport_c::Navigate_SendGoal_Response_message_type_support_handle;
}

#ifdef __cplusplus
}
#endif

// already included above
// #include "cstddef"
#include "rosidl_runtime_c/service_type_support_struct.h"
// already included above
// #include "test_package/action/detail/navigate__type_support.h"
// already included above
// #include "rosidl_typesupport_c/identifier.h"
#include "rosidl_typesupport_c/service_type_support_dispatch.h"
// already included above
// #include "rosidl_typesupport_c/type_support_map.h"
// already included above
// #include "rosidl_typesupport_interface/macros.h"

namespace test_package
{

namespace action
{

namespace rosidl_typesupport_c
{

typedef struct _Navigate_SendGoal_type_support_ids_t
{
  const char * typesupport_identifier[2];
} _Navigate_SendGoal_type_support_ids_t;

static const _Navigate_SendGoal_type_support_ids_t _Navigate_SendGoal_service_typesupport_ids = {
  {
    "rosidl_typesupport_fastrtps_c",  // ::rosidl_typesupport_fastrtps_c::typesupport_identifier,
    "rosidl_typesupport_introspection_c",  // ::rosidl_typesupport_introspection_c::typesupport_identifier,
  }
};

typedef struct _Navigate_SendGoal_type_support_symbol_names_t
{
  const char * symbol_name[2];
} _Navigate_SendGoal_type_support_symbol_names_t;

#define STRINGIFY_(s) #s
#define STRINGIFY(s) STRINGIFY_(s)

static const _Navigate_SendGoal_type_support_symbol_names_t _Navigate_SendGoal_service_typesupport_symbol_names = {
  {
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, test_package, action, Navigate_SendGoal)),
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_SYMBOL_NAME(rosidl_typesupport_introspection_c, test_package, action, Navigate_SendGoal)),
  }
};

typedef struct _Navigate_SendGoal_type_support_data_t
{
  void * data[2];
} _Navigate_SendGoal_type_support_data_t;

static _Navigate_SendGoal_type_support_data_t _Navigate_SendGoal_service_typesupport_data = {
  {
    0,  // will store the shared library later
    0,  // will store the shared library later
  }
};

static const type_support_map_t _Navigate_SendGoal_service_typesupport_map = {
  2,
  "test_package",
  &_Navigate_SendGoal_service_typesupport_ids.typesupport_identifier[0],
  &_Navigate_SendGoal_service_typesupport_symbol_names.symbol_name[0],
  &_Navigate_SendGoal_service_typesupport_data.data[0],
};

static const rosidl_service_type_support_t Navigate_SendGoal_service_type_support_handle = {
  rosidl_typesupport_c__typesupport_identifier,
  reinterpret_cast<const type_support_map_t *>(&_Navigate_SendGoal_service_typesupport_map),
  rosidl_typesupport_c__get_service_typesupport_handle_function,
};

}  // namespace rosidl_typesupport_c

}  // namespace action

}  // namespace test_package

#ifdef __cplusplus
extern "C"
{
#endif

const rosidl_service_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_SYMBOL_NAME(rosidl_typesupport_c, test_package, action, Navigate_SendGoal)() {
  return &::test_package::action::rosidl_typesupport_c::Navigate_SendGoal_service_type_support_handle;
}

#ifdef __cplusplus
}
#endif

// already included above
// #include "cstddef"
// already included above
// #include "rosidl_runtime_c/message_type_support_struct.h"
// already included above
// #include "test_package/action/detail/navigate__struct.h"
// already included above
// #include "test_package/action/detail/navigate__type_support.h"
// already included above
// #include "rosidl_typesupport_c/identifier.h"
// already included above
// #include "rosidl_typesupport_c/message_type_support_dispatch.h"
// already included above
// #include "rosidl_typesupport_c/type_support_map.h"
// already included above
// #include "rosidl_typesupport_c/visibility_control.h"
// already included above
// #include "rosidl_typesupport_interface/macros.h"

namespace test_package
{

namespace action
{

namespace rosidl_typesupport_c
{

typedef struct _Navigate_GetResult_Request_type_support_ids_t
{
  const char * typesupport_identifier[2];
} _Navigate_GetResult_Request_type_support_ids_t;

static const _Navigate_GetResult_Request_type_support_ids_t _Navigate_GetResult_Request_message_typesupport_ids = {
  {
    "rosidl_typesupport_fastrtps_c",  // ::rosidl_typesupport_fastrtps_c::typesupport_identifier,
    "rosidl_typesupport_introspection_c",  // ::rosidl_typesupport_introspection_c::typesupport_identifier,
  }
};

typedef struct _Navigate_GetResult_Request_type_support_symbol_names_t
{
  const char * symbol_name[2];
} _Navigate_GetResult_Request_type_support_symbol_names_t;

#define STRINGIFY_(s) #s
#define STRINGIFY(s) STRINGIFY_(s)

static const _Navigate_GetResult_Request_type_support_symbol_names_t _Navigate_GetResult_Request_message_typesupport_symbol_names = {
  {
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, test_package, action, Navigate_GetResult_Request)),
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, test_package, action, Navigate_GetResult_Request)),
  }
};

typedef struct _Navigate_GetResult_Request_type_support_data_t
{
  void * data[2];
} _Navigate_GetResult_Request_type_support_data_t;

static _Navigate_GetResult_Request_type_support_data_t _Navigate_GetResult_Request_message_typesupport_data = {
  {
    0,  // will store the shared library later
    0,  // will store the shared library later
  }
};

static const type_support_map_t _Navigate_GetResult_Request_message_typesupport_map = {
  2,
  "test_package",
  &_Navigate_GetResult_Request_message_typesupport_ids.typesupport_identifier[0],
  &_Navigate_GetResult_Request_message_typesupport_symbol_names.symbol_name[0],
  &_Navigate_GetResult_Request_message_typesupport_data.data[0],
};

static const rosidl_message_type_support_t Navigate_GetResult_Request_message_type_support_handle = {
  rosidl_typesupport_c__typesupport_identifier,
  reinterpret_cast<const type_support_map_t *>(&_Navigate_GetResult_Request_message_typesupport_map),
  rosidl_typesupport_c__get_message_typesupport_handle_function,
};

}  // namespace rosidl_typesupport_c

}  // namespace action

}  // namespace test_package

#ifdef __cplusplus
extern "C"
{
#endif

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_c, test_package, action, Navigate_GetResult_Request)() {
  return &::test_package::action::rosidl_typesupport_c::Navigate_GetResult_Request_message_type_support_handle;
}

#ifdef __cplusplus
}
#endif

// already included above
// #include "cstddef"
// already included above
// #include "rosidl_runtime_c/message_type_support_struct.h"
// already included above
// #include "test_package/action/detail/navigate__struct.h"
// already included above
// #include "test_package/action/detail/navigate__type_support.h"
// already included above
// #include "rosidl_typesupport_c/identifier.h"
// already included above
// #include "rosidl_typesupport_c/message_type_support_dispatch.h"
// already included above
// #include "rosidl_typesupport_c/type_support_map.h"
// already included above
// #include "rosidl_typesupport_c/visibility_control.h"
// already included above
// #include "rosidl_typesupport_interface/macros.h"

namespace test_package
{

namespace action
{

namespace rosidl_typesupport_c
{

typedef struct _Navigate_GetResult_Response_type_support_ids_t
{
  const char * typesupport_identifier[2];
} _Navigate_GetResult_Response_type_support_ids_t;

static const _Navigate_GetResult_Response_type_support_ids_t _Navigate_GetResult_Response_message_typesupport_ids = {
  {
    "rosidl_typesupport_fastrtps_c",  // ::rosidl_typesupport_fastrtps_c::typesupport_identifier,
    "rosidl_typesupport_introspection_c",  // ::rosidl_typesupport_introspection_c::typesupport_identifier,
  }
};

typedef struct _Navigate_GetResult_Response_type_support_symbol_names_t
{
  const char * symbol_name[2];
} _Navigate_GetResult_Response_type_support_symbol_names_t;

#define STRINGIFY_(s) #s
#define STRINGIFY(s) STRINGIFY_(s)

static const _Navigate_GetResult_Response_type_support_symbol_names_t _Navigate_GetResult_Response_message_typesupport_symbol_names = {
  {
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, test_package, action, Navigate_GetResult_Response)),
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, test_package, action, Navigate_GetResult_Response)),
  }
};

typedef struct _Navigate_GetResult_Response_type_support_data_t
{
  void * data[2];
} _Navigate_GetResult_Response_type_support_data_t;

static _Navigate_GetResult_Response_type_support_data_t _Navigate_GetResult_Response_message_typesupport_data = {
  {
    0,  // will store the shared library later
    0,  // will store the shared library later
  }
};

static const type_support_map_t _Navigate_GetResult_Response_message_typesupport_map = {
  2,
  "test_package",
  &_Navigate_GetResult_Response_message_typesupport_ids.typesupport_identifier[0],
  &_Navigate_GetResult_Response_message_typesupport_symbol_names.symbol_name[0],
  &_Navigate_GetResult_Response_message_typesupport_data.data[0],
};

static const rosidl_message_type_support_t Navigate_GetResult_Response_message_type_support_handle = {
  rosidl_typesupport_c__typesupport_identifier,
  reinterpret_cast<const type_support_map_t *>(&_Navigate_GetResult_Response_message_typesupport_map),
  rosidl_typesupport_c__get_message_typesupport_handle_function,
};

}  // namespace rosidl_typesupport_c

}  // namespace action

}  // namespace test_package

#ifdef __cplusplus
extern "C"
{
#endif

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_c, test_package, action, Navigate_GetResult_Response)() {
  return &::test_package::action::rosidl_typesupport_c::Navigate_GetResult_Response_message_type_support_handle;
}

#ifdef __cplusplus
}
#endif

// already included above
// #include "cstddef"
// already included above
// #include "rosidl_runtime_c/service_type_support_struct.h"
// already included above
// #include "test_package/action/detail/navigate__type_support.h"
// already included above
// #include "rosidl_typesupport_c/identifier.h"
// already included above
// #include "rosidl_typesupport_c/service_type_support_dispatch.h"
// already included above
// #include "rosidl_typesupport_c/type_support_map.h"
// already included above
// #include "rosidl_typesupport_interface/macros.h"

namespace test_package
{

namespace action
{

namespace rosidl_typesupport_c
{

typedef struct _Navigate_GetResult_type_support_ids_t
{
  const char * typesupport_identifier[2];
} _Navigate_GetResult_type_support_ids_t;

static const _Navigate_GetResult_type_support_ids_t _Navigate_GetResult_service_typesupport_ids = {
  {
    "rosidl_typesupport_fastrtps_c",  // ::rosidl_typesupport_fastrtps_c::typesupport_identifier,
    "rosidl_typesupport_introspection_c",  // ::rosidl_typesupport_introspection_c::typesupport_identifier,
  }
};

typedef struct _Navigate_GetResult_type_support_symbol_names_t
{
  const char * symbol_name[2];
} _Navigate_GetResult_type_support_symbol_names_t;

#define STRINGIFY_(s) #s
#define STRINGIFY(s) STRINGIFY_(s)

static const _Navigate_GetResult_type_support_symbol_names_t _Navigate_GetResult_service_typesupport_symbol_names = {
  {
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, test_package, action, Navigate_GetResult)),
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_SYMBOL_NAME(rosidl_typesupport_introspection_c, test_package, action, Navigate_GetResult)),
  }
};

typedef struct _Navigate_GetResult_type_support_data_t
{
  void * data[2];
} _Navigate_GetResult_type_support_data_t;

static _Navigate_GetResult_type_support_data_t _Navigate_GetResult_service_typesupport_data = {
  {
    0,  // will store the shared library later
    0,  // will store the shared library later
  }
};

static const type_support_map_t _Navigate_GetResult_service_typesupport_map = {
  2,
  "test_package",
  &_Navigate_GetResult_service_typesupport_ids.typesupport_identifier[0],
  &_Navigate_GetResult_service_typesupport_symbol_names.symbol_name[0],
  &_Navigate_GetResult_service_typesupport_data.data[0],
};

static const rosidl_service_type_support_t Navigate_GetResult_service_type_support_handle = {
  rosidl_typesupport_c__typesupport_identifier,
  reinterpret_cast<const type_support_map_t *>(&_Navigate_GetResult_service_typesupport_map),
  rosidl_typesupport_c__get_service_typesupport_handle_function,
};

}  // namespace rosidl_typesupport_c

}  // namespace action

}  // namespace test_package

#ifdef __cplusplus
extern "C"
{
#endif

const rosidl_service_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_SYMBOL_NAME(rosidl_typesupport_c, test_package, action, Navigate_GetResult)() {
  return &::test_package::action::rosidl_typesupport_c::Navigate_GetResult_service_type_support_handle;
}

#ifdef __cplusplus
}
#endif

// already included above
// #include "cstddef"
// already included above
// #include "rosidl_runtime_c/message_type_support_struct.h"
// already included above
// #include "test_package/action/detail/navigate__struct.h"
// already included above
// #include "test_package/action/detail/navigate__type_support.h"
// already included above
// #include "rosidl_typesupport_c/identifier.h"
// already included above
// #include "rosidl_typesupport_c/message_type_support_dispatch.h"
// already included above
// #include "rosidl_typesupport_c/type_support_map.h"
// already included above
// #include "rosidl_typesupport_c/visibility_control.h"
// already included above
// #include "rosidl_typesupport_interface/macros.h"

namespace test_package
{

namespace action
{

namespace rosidl_typesupport_c
{

typedef struct _Navigate_FeedbackMessage_type_support_ids_t
{
  const char * typesupport_identifier[2];
} _Navigate_FeedbackMessage_type_support_ids_t;

static const _Navigate_FeedbackMessage_type_support_ids_t _Navigate_FeedbackMessage_message_typesupport_ids = {
  {
    "rosidl_typesupport_fastrtps_c",  // ::rosidl_typesupport_fastrtps_c::typesupport_identifier,
    "rosidl_typesupport_introspection_c",  // ::rosidl_typesupport_introspection_c::typesupport_identifier,
  }
};

typedef struct _Navigate_FeedbackMessage_type_support_symbol_names_t
{
  const char * symbol_name[2];
} _Navigate_FeedbackMessage_type_support_symbol_names_t;

#define STRINGIFY_(s) #s
#define STRINGIFY(s) STRINGIFY_(s)

static const _Navigate_FeedbackMessage_type_support_symbol_names_t _Navigate_FeedbackMessage_message_typesupport_symbol_names = {
  {
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, test_package, action, Navigate_FeedbackMessage)),
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, test_package, action, Navigate_FeedbackMessage)),
  }
};

typedef struct _Navigate_FeedbackMessage_type_support_data_t
{
  void * data[2];
} _Navigate_FeedbackMessage_type_support_data_t;

static _Navigate_FeedbackMessage_type_support_data_t _Navigate_FeedbackMessage_message_typesupport_data = {
  {
    0,  // will store the shared library later
    0,  // will store the shared library later
  }
};

static const type_support_map_t _Navigate_FeedbackMessage_message_typesupport_map = {
  2,
  "test_package",
  &_Navigate_FeedbackMessage_message_typesupport_ids.typesupport_identifier[0],
  &_Navigate_FeedbackMessage_message_typesupport_symbol_names.symbol_name[0],
  &_Navigate_FeedbackMessage_message_typesupport_data.data[0],
};

static const rosidl_message_type_support_t Navigate_FeedbackMessage_message_type_support_handle = {
  rosidl_typesupport_c__typesupport_identifier,
  reinterpret_cast<const type_support_map_t *>(&_Navigate_FeedbackMessage_message_typesupport_map),
  rosidl_typesupport_c__get_message_typesupport_handle_function,
};

}  // namespace rosidl_typesupport_c

}  // namespace action

}  // namespace test_package

#ifdef __cplusplus
extern "C"
{
#endif

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_c, test_package, action, Navigate_FeedbackMessage)() {
  return &::test_package::action::rosidl_typesupport_c::Navigate_FeedbackMessage_message_type_support_handle;
}

#ifdef __cplusplus
}
#endif

#include "action_msgs/msg/goal_status_array.h"
#include "action_msgs/srv/cancel_goal.h"
#include "test_package/action/navigate.h"
// already included above
// #include "test_package/action/detail/navigate__type_support.h"

static rosidl_action_type_support_t _test_package__action__Navigate__typesupport_c;

#ifdef __cplusplus
extern "C"
{
#endif

const rosidl_action_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__ACTION_SYMBOL_NAME(
  rosidl_typesupport_c, test_package, action, Navigate)()
{
  // Thread-safe by always writing the same values to the static struct
  _test_package__action__Navigate__typesupport_c.goal_service_type_support =
    ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_SYMBOL_NAME(
    rosidl_typesupport_c, test_package, action, Navigate_SendGoal)();
  _test_package__action__Navigate__typesupport_c.result_service_type_support =
    ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_SYMBOL_NAME(
    rosidl_typesupport_c, test_package, action, Navigate_GetResult)();
  _test_package__action__Navigate__typesupport_c.cancel_service_type_support =
    ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_SYMBOL_NAME(
    rosidl_typesupport_c, action_msgs, srv, CancelGoal)();
  _test_package__action__Navigate__typesupport_c.feedback_message_type_support =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(
    rosidl_typesupport_c, test_package, action, Navigate_FeedbackMessage)();
  _test_package__action__Navigate__typesupport_c.status_message_type_support =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(
    rosidl_typesupport_c, action_msgs, msg, GoalStatusArray)();

  return &_test_package__action__Navigate__typesupport_c;
}

#ifdef __cplusplus
}
#endif
