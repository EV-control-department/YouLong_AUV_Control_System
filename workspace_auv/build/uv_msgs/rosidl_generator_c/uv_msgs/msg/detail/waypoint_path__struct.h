// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from uv_msgs:msg/WaypointPath.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "uv_msgs/msg/waypoint_path.h"


#ifndef UV_MSGS__MSG__DETAIL__WAYPOINT_PATH__STRUCT_H_
#define UV_MSGS__MSG__DETAIL__WAYPOINT_PATH__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

// Constants defined in the message

// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__struct.h"
// Member 'waypoints'
#include "uv_msgs/msg/detail/waypoint__struct.h"

/// Struct defined in msg/WaypointPath in the package uv_msgs.
/**
  * Array of waypoints for path planning
 */
typedef struct uv_msgs__msg__WaypointPath
{
  std_msgs__msg__Header header;
  uv_msgs__msg__Waypoint__Sequence waypoints;
} uv_msgs__msg__WaypointPath;

// Struct for a sequence of uv_msgs__msg__WaypointPath.
typedef struct uv_msgs__msg__WaypointPath__Sequence
{
  uv_msgs__msg__WaypointPath * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} uv_msgs__msg__WaypointPath__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // UV_MSGS__MSG__DETAIL__WAYPOINT_PATH__STRUCT_H_
