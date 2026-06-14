// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from uv_msgs:msg/DetectionArray.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "uv_msgs/msg/detection_array.h"


#ifndef UV_MSGS__MSG__DETAIL__DETECTION_ARRAY__STRUCT_H_
#define UV_MSGS__MSG__DETAIL__DETECTION_ARRAY__STRUCT_H_

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
// Member 'camera_name'
#include "rosidl_runtime_c/string.h"
// Member 'detections'
#include "uv_msgs/msg/detail/detection__struct.h"

/// Struct defined in msg/DetectionArray in the package uv_msgs.
/**
  * Array of detections from one camera frame
 */
typedef struct uv_msgs__msg__DetectionArray
{
  std_msgs__msg__Header header;
  /// "front" or "down"
  rosidl_runtime_c__String camera_name;
  uv_msgs__msg__Detection__Sequence detections;
} uv_msgs__msg__DetectionArray;

// Struct for a sequence of uv_msgs__msg__DetectionArray.
typedef struct uv_msgs__msg__DetectionArray__Sequence
{
  uv_msgs__msg__DetectionArray * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} uv_msgs__msg__DetectionArray__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // UV_MSGS__MSG__DETAIL__DETECTION_ARRAY__STRUCT_H_
