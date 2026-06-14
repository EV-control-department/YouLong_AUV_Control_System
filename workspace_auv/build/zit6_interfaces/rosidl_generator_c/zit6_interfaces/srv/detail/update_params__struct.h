// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from zit6_interfaces:srv/UpdateParams.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "zit6_interfaces/srv/update_params.h"


#ifndef ZIT6_INTERFACES__SRV__DETAIL__UPDATE_PARAMS__STRUCT_H_
#define ZIT6_INTERFACES__SRV__DETAIL__UPDATE_PARAMS__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'json'
// Member 'paths'
// Member 'values'
#include "rosidl_runtime_c/string.h"

/// Struct defined in srv/UpdateParams in the package zit6_interfaces.
typedef struct zit6_interfaces__srv__UpdateParams_Request
{
  rosidl_runtime_c__String json;
  rosidl_runtime_c__String__Sequence paths;
  rosidl_runtime_c__String__Sequence values;
} zit6_interfaces__srv__UpdateParams_Request;

// Struct for a sequence of zit6_interfaces__srv__UpdateParams_Request.
typedef struct zit6_interfaces__srv__UpdateParams_Request__Sequence
{
  zit6_interfaces__srv__UpdateParams_Request * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} zit6_interfaces__srv__UpdateParams_Request__Sequence;

// Constants defined in the message

// Include directives for member types
// Member 'message'
// already included above
// #include "rosidl_runtime_c/string.h"

/// Struct defined in srv/UpdateParams in the package zit6_interfaces.
typedef struct zit6_interfaces__srv__UpdateParams_Response
{
  bool success;
  rosidl_runtime_c__String message;
} zit6_interfaces__srv__UpdateParams_Response;

// Struct for a sequence of zit6_interfaces__srv__UpdateParams_Response.
typedef struct zit6_interfaces__srv__UpdateParams_Response__Sequence
{
  zit6_interfaces__srv__UpdateParams_Response * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} zit6_interfaces__srv__UpdateParams_Response__Sequence;

// Constants defined in the message

// Include directives for member types
// Member 'info'
#include "service_msgs/msg/detail/service_event_info__struct.h"

// constants for array fields with an upper bound
// request
enum
{
  zit6_interfaces__srv__UpdateParams_Event__request__MAX_SIZE = 1
};
// response
enum
{
  zit6_interfaces__srv__UpdateParams_Event__response__MAX_SIZE = 1
};

/// Struct defined in srv/UpdateParams in the package zit6_interfaces.
typedef struct zit6_interfaces__srv__UpdateParams_Event
{
  service_msgs__msg__ServiceEventInfo info;
  zit6_interfaces__srv__UpdateParams_Request__Sequence request;
  zit6_interfaces__srv__UpdateParams_Response__Sequence response;
} zit6_interfaces__srv__UpdateParams_Event;

// Struct for a sequence of zit6_interfaces__srv__UpdateParams_Event.
typedef struct zit6_interfaces__srv__UpdateParams_Event__Sequence
{
  zit6_interfaces__srv__UpdateParams_Event * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} zit6_interfaces__srv__UpdateParams_Event__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // ZIT6_INTERFACES__SRV__DETAIL__UPDATE_PARAMS__STRUCT_H_
