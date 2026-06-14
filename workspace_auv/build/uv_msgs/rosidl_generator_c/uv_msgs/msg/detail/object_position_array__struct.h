// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from uv_msgs:msg/ObjectPositionArray.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "uv_msgs/msg/object_position_array.h"


#ifndef UV_MSGS__MSG__DETAIL__OBJECT_POSITION_ARRAY__STRUCT_H_
#define UV_MSGS__MSG__DETAIL__OBJECT_POSITION_ARRAY__STRUCT_H_

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
// Member 'objects'
#include "uv_msgs/msg/detail/object_position__struct.h"

/// Struct defined in msg/ObjectPositionArray in the package uv_msgs.
/**
  * Array of tracked object positions
 */
typedef struct uv_msgs__msg__ObjectPositionArray
{
  std_msgs__msg__Header header;
  uv_msgs__msg__ObjectPosition__Sequence objects;
} uv_msgs__msg__ObjectPositionArray;

// Struct for a sequence of uv_msgs__msg__ObjectPositionArray.
typedef struct uv_msgs__msg__ObjectPositionArray__Sequence
{
  uv_msgs__msg__ObjectPositionArray * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} uv_msgs__msg__ObjectPositionArray__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // UV_MSGS__MSG__DETAIL__OBJECT_POSITION_ARRAY__STRUCT_H_
