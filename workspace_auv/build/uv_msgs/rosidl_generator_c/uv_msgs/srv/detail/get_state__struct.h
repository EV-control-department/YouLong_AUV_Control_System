// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from uv_msgs:srv/GetState.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "uv_msgs/srv/get_state.h"


#ifndef UV_MSGS__SRV__DETAIL__GET_STATE__STRUCT_H_
#define UV_MSGS__SRV__DETAIL__GET_STATE__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

/// Struct defined in srv/GetState in the package uv_msgs.
typedef struct uv_msgs__srv__GetState_Request
{
  uint8_t structure_needs_at_least_one_member;
} uv_msgs__srv__GetState_Request;

// Struct for a sequence of uv_msgs__srv__GetState_Request.
typedef struct uv_msgs__srv__GetState_Request__Sequence
{
  uv_msgs__srv__GetState_Request * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} uv_msgs__srv__GetState_Request__Sequence;

// Constants defined in the message

// Include directives for member types
// Member 'state'
#include "uv_msgs/msg/detail/auv_state__struct.h"

/// Struct defined in srv/GetState in the package uv_msgs.
typedef struct uv_msgs__srv__GetState_Response
{
  uv_msgs__msg__AuvState state;
} uv_msgs__srv__GetState_Response;

// Struct for a sequence of uv_msgs__srv__GetState_Response.
typedef struct uv_msgs__srv__GetState_Response__Sequence
{
  uv_msgs__srv__GetState_Response * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} uv_msgs__srv__GetState_Response__Sequence;

// Constants defined in the message

// Include directives for member types
// Member 'info'
#include "service_msgs/msg/detail/service_event_info__struct.h"

// constants for array fields with an upper bound
// request
enum
{
  uv_msgs__srv__GetState_Event__request__MAX_SIZE = 1
};
// response
enum
{
  uv_msgs__srv__GetState_Event__response__MAX_SIZE = 1
};

/// Struct defined in srv/GetState in the package uv_msgs.
typedef struct uv_msgs__srv__GetState_Event
{
  service_msgs__msg__ServiceEventInfo info;
  uv_msgs__srv__GetState_Request__Sequence request;
  uv_msgs__srv__GetState_Response__Sequence response;
} uv_msgs__srv__GetState_Event;

// Struct for a sequence of uv_msgs__srv__GetState_Event.
typedef struct uv_msgs__srv__GetState_Event__Sequence
{
  uv_msgs__srv__GetState_Event * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} uv_msgs__srv__GetState_Event__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // UV_MSGS__SRV__DETAIL__GET_STATE__STRUCT_H_
