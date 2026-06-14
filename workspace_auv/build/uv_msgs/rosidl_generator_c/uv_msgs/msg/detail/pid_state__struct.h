// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from uv_msgs:msg/PidState.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "uv_msgs/msg/pid_state.h"


#ifndef UV_MSGS__MSG__DETAIL__PID_STATE__STRUCT_H_
#define UV_MSGS__MSG__DETAIL__PID_STATE__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

// Constants defined in the message

// Include directives for member types
// Member 'speed_mag'
// Member 'vel_x'
// Member 'vel_y'
// Member 'pos_z'
// Member 'yaw'
// Member 'yaw_rate'
#include "uv_msgs/msg/detail/pid_gains__struct.h"

/// Struct defined in msg/PidState in the package uv_msgs.
/**
  * PID controller telemetry (published for debugging/tuning)
 */
typedef struct uv_msgs__msg__PidState
{
  /// Outer loop gains
  uv_msgs__msg__PidGains speed_mag;
  /// Inner loop X gains
  uv_msgs__msg__PidGains vel_x;
  /// Inner loop Y gains
  uv_msgs__msg__PidGains vel_y;
  /// Z axis gains
  uv_msgs__msg__PidGains pos_z;
  /// Yaw position loop
  uv_msgs__msg__PidGains yaw;
  /// Yaw rate loop
  uv_msgs__msg__PidGains yaw_rate;
  float error_length;
  float error_angle_deg;
  float speed_cmd;
  float vx_body_cmd;
  float vy_body_cmd;
} uv_msgs__msg__PidState;

// Struct for a sequence of uv_msgs__msg__PidState.
typedef struct uv_msgs__msg__PidState__Sequence
{
  uv_msgs__msg__PidState * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} uv_msgs__msg__PidState__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // UV_MSGS__MSG__DETAIL__PID_STATE__STRUCT_H_
