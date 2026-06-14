// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from uv_msgs:msg/PidGains.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "uv_msgs/msg/pid_gains.h"


#ifndef UV_MSGS__MSG__DETAIL__PID_GAINS__STRUCT_H_
#define UV_MSGS__MSG__DETAIL__PID_GAINS__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

// Constants defined in the message

// Include directives for member types
// Member 'name'
#include "rosidl_runtime_c/string.h"

/// Struct defined in msg/PidGains in the package uv_msgs.
/**
  * PID controller gains
 */
typedef struct uv_msgs__msg__PidGains
{
  rosidl_runtime_c__String name;
  float kp;
  float ki;
  float kd;
  float i_limit;
  float output_limit;
  float feedforward;
} uv_msgs__msg__PidGains;

// Struct for a sequence of uv_msgs__msg__PidGains.
typedef struct uv_msgs__msg__PidGains__Sequence
{
  uv_msgs__msg__PidGains * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} uv_msgs__msg__PidGains__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // UV_MSGS__MSG__DETAIL__PID_GAINS__STRUCT_H_
