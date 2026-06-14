// generated from rosidl_typesupport_cpp/resource/idl__type_support.cpp.em
// with input from uv_msgs:srv/RunTask.idl
// generated code does not contain a copyright notice

#include "cstddef"
#include "rosidl_runtime_c/message_type_support_struct.h"
#include "uv_msgs/srv/detail/run_task__functions.h"
#include "uv_msgs/srv/detail/run_task__struct.hpp"
#include "rosidl_typesupport_cpp/identifier.hpp"
#include "rosidl_typesupport_cpp/message_type_support.hpp"
#include "rosidl_typesupport_c/type_support_map.h"
#include "rosidl_typesupport_cpp/message_type_support_dispatch.hpp"
#include "rosidl_typesupport_cpp/visibility_control.h"
#include "rosidl_typesupport_interface/macros.h"

namespace uv_msgs
{

namespace srv
{

namespace rosidl_typesupport_cpp
{

typedef struct _RunTask_Request_type_support_ids_t
{
  const char * typesupport_identifier[2];
} _RunTask_Request_type_support_ids_t;

static const _RunTask_Request_type_support_ids_t _RunTask_Request_message_typesupport_ids = {
  {
    "rosidl_typesupport_fastrtps_cpp",  // ::rosidl_typesupport_fastrtps_cpp::typesupport_identifier,
    "rosidl_typesupport_introspection_cpp",  // ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  }
};

typedef struct _RunTask_Request_type_support_symbol_names_t
{
  const char * symbol_name[2];
} _RunTask_Request_type_support_symbol_names_t;

#define STRINGIFY_(s) #s
#define STRINGIFY(s) STRINGIFY_(s)

static const _RunTask_Request_type_support_symbol_names_t _RunTask_Request_message_typesupport_symbol_names = {
  {
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_cpp, uv_msgs, srv, RunTask_Request)),
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, uv_msgs, srv, RunTask_Request)),
  }
};

typedef struct _RunTask_Request_type_support_data_t
{
  void * data[2];
} _RunTask_Request_type_support_data_t;

static _RunTask_Request_type_support_data_t _RunTask_Request_message_typesupport_data = {
  {
    0,  // will store the shared library later
    0,  // will store the shared library later
  }
};

static const type_support_map_t _RunTask_Request_message_typesupport_map = {
  2,
  "uv_msgs",
  &_RunTask_Request_message_typesupport_ids.typesupport_identifier[0],
  &_RunTask_Request_message_typesupport_symbol_names.symbol_name[0],
  &_RunTask_Request_message_typesupport_data.data[0],
};

static const rosidl_message_type_support_t RunTask_Request_message_type_support_handle = {
  ::rosidl_typesupport_cpp::typesupport_identifier,
  reinterpret_cast<const type_support_map_t *>(&_RunTask_Request_message_typesupport_map),
  ::rosidl_typesupport_cpp::get_message_typesupport_handle_function,
  &uv_msgs__srv__RunTask_Request__get_type_hash,
  &uv_msgs__srv__RunTask_Request__get_type_description,
  &uv_msgs__srv__RunTask_Request__get_type_description_sources,
};

}  // namespace rosidl_typesupport_cpp

}  // namespace srv

}  // namespace uv_msgs

namespace rosidl_typesupport_cpp
{

template<>
ROSIDL_TYPESUPPORT_CPP_PUBLIC
const rosidl_message_type_support_t *
get_message_type_support_handle<uv_msgs::srv::RunTask_Request>()
{
  return &::uv_msgs::srv::rosidl_typesupport_cpp::RunTask_Request_message_type_support_handle;
}

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_CPP_PUBLIC
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_cpp, uv_msgs, srv, RunTask_Request)() {
  return get_message_type_support_handle<uv_msgs::srv::RunTask_Request>();
}

#ifdef __cplusplus
}
#endif
}  // namespace rosidl_typesupport_cpp

// already included above
// #include "cstddef"
// already included above
// #include "rosidl_runtime_c/message_type_support_struct.h"
// already included above
// #include "uv_msgs/srv/detail/run_task__functions.h"
// already included above
// #include "uv_msgs/srv/detail/run_task__struct.hpp"
// already included above
// #include "rosidl_typesupport_cpp/identifier.hpp"
// already included above
// #include "rosidl_typesupport_cpp/message_type_support.hpp"
// already included above
// #include "rosidl_typesupport_c/type_support_map.h"
// already included above
// #include "rosidl_typesupport_cpp/message_type_support_dispatch.hpp"
// already included above
// #include "rosidl_typesupport_cpp/visibility_control.h"
// already included above
// #include "rosidl_typesupport_interface/macros.h"

namespace uv_msgs
{

namespace srv
{

namespace rosidl_typesupport_cpp
{

typedef struct _RunTask_Response_type_support_ids_t
{
  const char * typesupport_identifier[2];
} _RunTask_Response_type_support_ids_t;

static const _RunTask_Response_type_support_ids_t _RunTask_Response_message_typesupport_ids = {
  {
    "rosidl_typesupport_fastrtps_cpp",  // ::rosidl_typesupport_fastrtps_cpp::typesupport_identifier,
    "rosidl_typesupport_introspection_cpp",  // ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  }
};

typedef struct _RunTask_Response_type_support_symbol_names_t
{
  const char * symbol_name[2];
} _RunTask_Response_type_support_symbol_names_t;

#define STRINGIFY_(s) #s
#define STRINGIFY(s) STRINGIFY_(s)

static const _RunTask_Response_type_support_symbol_names_t _RunTask_Response_message_typesupport_symbol_names = {
  {
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_cpp, uv_msgs, srv, RunTask_Response)),
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, uv_msgs, srv, RunTask_Response)),
  }
};

typedef struct _RunTask_Response_type_support_data_t
{
  void * data[2];
} _RunTask_Response_type_support_data_t;

static _RunTask_Response_type_support_data_t _RunTask_Response_message_typesupport_data = {
  {
    0,  // will store the shared library later
    0,  // will store the shared library later
  }
};

static const type_support_map_t _RunTask_Response_message_typesupport_map = {
  2,
  "uv_msgs",
  &_RunTask_Response_message_typesupport_ids.typesupport_identifier[0],
  &_RunTask_Response_message_typesupport_symbol_names.symbol_name[0],
  &_RunTask_Response_message_typesupport_data.data[0],
};

static const rosidl_message_type_support_t RunTask_Response_message_type_support_handle = {
  ::rosidl_typesupport_cpp::typesupport_identifier,
  reinterpret_cast<const type_support_map_t *>(&_RunTask_Response_message_typesupport_map),
  ::rosidl_typesupport_cpp::get_message_typesupport_handle_function,
  &uv_msgs__srv__RunTask_Response__get_type_hash,
  &uv_msgs__srv__RunTask_Response__get_type_description,
  &uv_msgs__srv__RunTask_Response__get_type_description_sources,
};

}  // namespace rosidl_typesupport_cpp

}  // namespace srv

}  // namespace uv_msgs

namespace rosidl_typesupport_cpp
{

template<>
ROSIDL_TYPESUPPORT_CPP_PUBLIC
const rosidl_message_type_support_t *
get_message_type_support_handle<uv_msgs::srv::RunTask_Response>()
{
  return &::uv_msgs::srv::rosidl_typesupport_cpp::RunTask_Response_message_type_support_handle;
}

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_CPP_PUBLIC
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_cpp, uv_msgs, srv, RunTask_Response)() {
  return get_message_type_support_handle<uv_msgs::srv::RunTask_Response>();
}

#ifdef __cplusplus
}
#endif
}  // namespace rosidl_typesupport_cpp

// already included above
// #include "cstddef"
// already included above
// #include "rosidl_runtime_c/message_type_support_struct.h"
// already included above
// #include "uv_msgs/srv/detail/run_task__functions.h"
// already included above
// #include "uv_msgs/srv/detail/run_task__struct.hpp"
// already included above
// #include "rosidl_typesupport_cpp/identifier.hpp"
// already included above
// #include "rosidl_typesupport_cpp/message_type_support.hpp"
// already included above
// #include "rosidl_typesupport_c/type_support_map.h"
// already included above
// #include "rosidl_typesupport_cpp/message_type_support_dispatch.hpp"
// already included above
// #include "rosidl_typesupport_cpp/visibility_control.h"
// already included above
// #include "rosidl_typesupport_interface/macros.h"

namespace uv_msgs
{

namespace srv
{

namespace rosidl_typesupport_cpp
{

typedef struct _RunTask_Event_type_support_ids_t
{
  const char * typesupport_identifier[2];
} _RunTask_Event_type_support_ids_t;

static const _RunTask_Event_type_support_ids_t _RunTask_Event_message_typesupport_ids = {
  {
    "rosidl_typesupport_fastrtps_cpp",  // ::rosidl_typesupport_fastrtps_cpp::typesupport_identifier,
    "rosidl_typesupport_introspection_cpp",  // ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  }
};

typedef struct _RunTask_Event_type_support_symbol_names_t
{
  const char * symbol_name[2];
} _RunTask_Event_type_support_symbol_names_t;

#define STRINGIFY_(s) #s
#define STRINGIFY(s) STRINGIFY_(s)

static const _RunTask_Event_type_support_symbol_names_t _RunTask_Event_message_typesupport_symbol_names = {
  {
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_cpp, uv_msgs, srv, RunTask_Event)),
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, uv_msgs, srv, RunTask_Event)),
  }
};

typedef struct _RunTask_Event_type_support_data_t
{
  void * data[2];
} _RunTask_Event_type_support_data_t;

static _RunTask_Event_type_support_data_t _RunTask_Event_message_typesupport_data = {
  {
    0,  // will store the shared library later
    0,  // will store the shared library later
  }
};

static const type_support_map_t _RunTask_Event_message_typesupport_map = {
  2,
  "uv_msgs",
  &_RunTask_Event_message_typesupport_ids.typesupport_identifier[0],
  &_RunTask_Event_message_typesupport_symbol_names.symbol_name[0],
  &_RunTask_Event_message_typesupport_data.data[0],
};

static const rosidl_message_type_support_t RunTask_Event_message_type_support_handle = {
  ::rosidl_typesupport_cpp::typesupport_identifier,
  reinterpret_cast<const type_support_map_t *>(&_RunTask_Event_message_typesupport_map),
  ::rosidl_typesupport_cpp::get_message_typesupport_handle_function,
  &uv_msgs__srv__RunTask_Event__get_type_hash,
  &uv_msgs__srv__RunTask_Event__get_type_description,
  &uv_msgs__srv__RunTask_Event__get_type_description_sources,
};

}  // namespace rosidl_typesupport_cpp

}  // namespace srv

}  // namespace uv_msgs

namespace rosidl_typesupport_cpp
{

template<>
ROSIDL_TYPESUPPORT_CPP_PUBLIC
const rosidl_message_type_support_t *
get_message_type_support_handle<uv_msgs::srv::RunTask_Event>()
{
  return &::uv_msgs::srv::rosidl_typesupport_cpp::RunTask_Event_message_type_support_handle;
}

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_CPP_PUBLIC
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_cpp, uv_msgs, srv, RunTask_Event)() {
  return get_message_type_support_handle<uv_msgs::srv::RunTask_Event>();
}

#ifdef __cplusplus
}
#endif
}  // namespace rosidl_typesupport_cpp

// already included above
// #include "cstddef"
#include "rosidl_runtime_c/service_type_support_struct.h"
#include "rosidl_typesupport_cpp/service_type_support.hpp"
// already included above
// #include "uv_msgs/srv/detail/run_task__struct.hpp"
// already included above
// #include "rosidl_typesupport_cpp/identifier.hpp"
// already included above
// #include "rosidl_typesupport_c/type_support_map.h"
#include "rosidl_typesupport_cpp/service_type_support_dispatch.hpp"
// already included above
// #include "rosidl_typesupport_cpp/visibility_control.h"
// already included above
// #include "rosidl_typesupport_interface/macros.h"

namespace uv_msgs
{

namespace srv
{

namespace rosidl_typesupport_cpp
{

typedef struct _RunTask_type_support_ids_t
{
  const char * typesupport_identifier[2];
} _RunTask_type_support_ids_t;

static const _RunTask_type_support_ids_t _RunTask_service_typesupport_ids = {
  {
    "rosidl_typesupport_fastrtps_cpp",  // ::rosidl_typesupport_fastrtps_cpp::typesupport_identifier,
    "rosidl_typesupport_introspection_cpp",  // ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  }
};

typedef struct _RunTask_type_support_symbol_names_t
{
  const char * symbol_name[2];
} _RunTask_type_support_symbol_names_t;
#define STRINGIFY_(s) #s
#define STRINGIFY(s) STRINGIFY_(s)

static const _RunTask_type_support_symbol_names_t _RunTask_service_typesupport_symbol_names = {
  {
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_SYMBOL_NAME(rosidl_typesupport_fastrtps_cpp, uv_msgs, srv, RunTask)),
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, uv_msgs, srv, RunTask)),
  }
};

typedef struct _RunTask_type_support_data_t
{
  void * data[2];
} _RunTask_type_support_data_t;

static _RunTask_type_support_data_t _RunTask_service_typesupport_data = {
  {
    0,  // will store the shared library later
    0,  // will store the shared library later
  }
};

static const type_support_map_t _RunTask_service_typesupport_map = {
  2,
  "uv_msgs",
  &_RunTask_service_typesupport_ids.typesupport_identifier[0],
  &_RunTask_service_typesupport_symbol_names.symbol_name[0],
  &_RunTask_service_typesupport_data.data[0],
};

static const rosidl_service_type_support_t RunTask_service_type_support_handle = {
  ::rosidl_typesupport_cpp::typesupport_identifier,
  reinterpret_cast<const type_support_map_t *>(&_RunTask_service_typesupport_map),
  ::rosidl_typesupport_cpp::get_service_typesupport_handle_function,
  ::rosidl_typesupport_cpp::get_message_type_support_handle<uv_msgs::srv::RunTask_Request>(),
  ::rosidl_typesupport_cpp::get_message_type_support_handle<uv_msgs::srv::RunTask_Response>(),
  ::rosidl_typesupport_cpp::get_message_type_support_handle<uv_msgs::srv::RunTask_Event>(),
  &::rosidl_typesupport_cpp::service_create_event_message<uv_msgs::srv::RunTask>,
  &::rosidl_typesupport_cpp::service_destroy_event_message<uv_msgs::srv::RunTask>,
  &uv_msgs__srv__RunTask__get_type_hash,
  &uv_msgs__srv__RunTask__get_type_description,
  &uv_msgs__srv__RunTask__get_type_description_sources,
};

}  // namespace rosidl_typesupport_cpp

}  // namespace srv

}  // namespace uv_msgs

namespace rosidl_typesupport_cpp
{

template<>
ROSIDL_TYPESUPPORT_CPP_PUBLIC
const rosidl_service_type_support_t *
get_service_type_support_handle<uv_msgs::srv::RunTask>()
{
  return &::uv_msgs::srv::rosidl_typesupport_cpp::RunTask_service_type_support_handle;
}

}  // namespace rosidl_typesupport_cpp

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_CPP_PUBLIC
const rosidl_service_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_SYMBOL_NAME(rosidl_typesupport_cpp, uv_msgs, srv, RunTask)() {
  return ::rosidl_typesupport_cpp::get_service_type_support_handle<uv_msgs::srv::RunTask>();
}

#ifdef __cplusplus
}
#endif
