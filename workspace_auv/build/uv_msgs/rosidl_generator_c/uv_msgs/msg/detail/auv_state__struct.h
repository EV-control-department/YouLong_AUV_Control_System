// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from uv_msgs:msg/AuvState.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "uv_msgs/msg/auv_state.h"


#ifndef UV_MSGS__MSG__DETAIL__AUV_STATE__STRUCT_H_
#define UV_MSGS__MSG__DETAIL__AUV_STATE__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

// Constants defined in the message

/// Struct defined in msg/AuvState in the package uv_msgs.
/**
  * Comprehensive robot state published by sim_bridge (or real MCU bridge)
 */
typedef struct uv_msgs__msg__AuvState
{
  /// Position in world NED frame
  /// North (m)
  float pos_x;
  /// East (m)
  float pos_y;
  /// Down (m, positive = deeper)
  float pos_z;
  /// Yaw in radians (NED frame)
  float yaw;
  /// Velocity in body frame (FRD: Forward-Right-Down)
  /// Forward (m/s)
  float vel_x;
  /// Right (m/s)
  float vel_y;
  /// Down (m/s)
  float vel_z;
  /// Yaw rate (rad/s)
  float yaw_rate;
  /// Velocity in world frame (for external use)
  /// North (m/s)
  float vel_world_x;
  /// East (m/s)
  float vel_world_y;
  /// Status
  bool armed;
  /// 0=none, 1=pos, 2=vel, 3=force
  uint8_t control_mode;
  float battery_voltage;
  uint32_t error_flags;
  float cycle_time_ms;
  /// Current target (for monitoring)
  float target_x;
  float target_y;
  float target_z;
  float target_yaw;
} uv_msgs__msg__AuvState;

// Struct for a sequence of uv_msgs__msg__AuvState.
typedef struct uv_msgs__msg__AuvState__Sequence
{
  uv_msgs__msg__AuvState * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} uv_msgs__msg__AuvState__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // UV_MSGS__MSG__DETAIL__AUV_STATE__STRUCT_H_
