// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from uv_msgs:msg/ObjectPosition.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "uv_msgs/msg/object_position.h"


#ifndef UV_MSGS__MSG__DETAIL__OBJECT_POSITION__STRUCT_H_
#define UV_MSGS__MSG__DETAIL__OBJECT_POSITION__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

// Constants defined in the message

// Include directives for member types
// Member 'class_name'
#include "rosidl_runtime_c/string.h"

/// Struct defined in msg/ObjectPosition in the package uv_msgs.
/**
  * World position of a tracked object
 */
typedef struct uv_msgs__msg__ObjectPosition
{
  uint16_t class_id;
  rosidl_runtime_c__String class_name;
  /// NED North (m)
  float world_x;
  /// NED East (m)
  float world_y;
  /// Depth (m)
  float world_z;
  /// Position estimate confidence
  float confidence;
  uint32_t num_observations;
} uv_msgs__msg__ObjectPosition;

// Struct for a sequence of uv_msgs__msg__ObjectPosition.
typedef struct uv_msgs__msg__ObjectPosition__Sequence
{
  uv_msgs__msg__ObjectPosition * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} uv_msgs__msg__ObjectPosition__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // UV_MSGS__MSG__DETAIL__OBJECT_POSITION__STRUCT_H_
