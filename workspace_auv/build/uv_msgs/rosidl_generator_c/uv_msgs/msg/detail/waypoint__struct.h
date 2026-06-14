// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from uv_msgs:msg/Waypoint.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "uv_msgs/msg/waypoint.h"


#ifndef UV_MSGS__MSG__DETAIL__WAYPOINT__STRUCT_H_
#define UV_MSGS__MSG__DETAIL__WAYPOINT__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

// Constants defined in the message

/// Struct defined in msg/Waypoint in the package uv_msgs.
/**
  * Single navigation waypoint
 */
typedef struct uv_msgs__msg__Waypoint
{
  /// NED North (m)
  float x;
  /// NED East (m)
  float y;
  /// Depth (m)
  float z;
  /// Heading (rad)
  float yaw;
  /// Target speed (m/s)
  float speed;
} uv_msgs__msg__Waypoint;

// Struct for a sequence of uv_msgs__msg__Waypoint.
typedef struct uv_msgs__msg__Waypoint__Sequence
{
  uv_msgs__msg__Waypoint * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} uv_msgs__msg__Waypoint__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // UV_MSGS__MSG__DETAIL__WAYPOINT__STRUCT_H_
