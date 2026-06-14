// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from uv_msgs:srv/RunTask.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "uv_msgs/srv/run_task.h"


#ifndef UV_MSGS__SRV__DETAIL__RUN_TASK__STRUCT_H_
#define UV_MSGS__SRV__DETAIL__RUN_TASK__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'task_name'
#include "rosidl_runtime_c/string.h"

/// Struct defined in srv/RunTask in the package uv_msgs.
typedef struct uv_msgs__srv__RunTask_Request
{
  /// Name of task or path to JSON file
  rosidl_runtime_c__String task_name;
  /// true=start, false=stop
  bool start;
} uv_msgs__srv__RunTask_Request;

// Struct for a sequence of uv_msgs__srv__RunTask_Request.
typedef struct uv_msgs__srv__RunTask_Request__Sequence
{
  uv_msgs__srv__RunTask_Request * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} uv_msgs__srv__RunTask_Request__Sequence;

// Constants defined in the message

// Include directives for member types
// Member 'message'
// already included above
// #include "rosidl_runtime_c/string.h"

/// Struct defined in srv/RunTask in the package uv_msgs.
typedef struct uv_msgs__srv__RunTask_Response
{
  bool success;
  rosidl_runtime_c__String message;
} uv_msgs__srv__RunTask_Response;

// Struct for a sequence of uv_msgs__srv__RunTask_Response.
typedef struct uv_msgs__srv__RunTask_Response__Sequence
{
  uv_msgs__srv__RunTask_Response * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} uv_msgs__srv__RunTask_Response__Sequence;

// Constants defined in the message

// Include directives for member types
// Member 'info'
#include "service_msgs/msg/detail/service_event_info__struct.h"

// constants for array fields with an upper bound
// request
enum
{
  uv_msgs__srv__RunTask_Event__request__MAX_SIZE = 1
};
// response
enum
{
  uv_msgs__srv__RunTask_Event__response__MAX_SIZE = 1
};

/// Struct defined in srv/RunTask in the package uv_msgs.
typedef struct uv_msgs__srv__RunTask_Event
{
  service_msgs__msg__ServiceEventInfo info;
  uv_msgs__srv__RunTask_Request__Sequence request;
  uv_msgs__srv__RunTask_Response__Sequence response;
} uv_msgs__srv__RunTask_Event;

// Struct for a sequence of uv_msgs__srv__RunTask_Event.
typedef struct uv_msgs__srv__RunTask_Event__Sequence
{
  uv_msgs__srv__RunTask_Event * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} uv_msgs__srv__RunTask_Event__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // UV_MSGS__SRV__DETAIL__RUN_TASK__STRUCT_H_
