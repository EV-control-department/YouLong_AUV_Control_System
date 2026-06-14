// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from uv_msgs:msg/Detection.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "uv_msgs/msg/detection.h"


#ifndef UV_MSGS__MSG__DETAIL__DETECTION__STRUCT_H_
#define UV_MSGS__MSG__DETAIL__DETECTION__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

// Constants defined in the message

/// Struct defined in msg/Detection in the package uv_msgs.
/**
  * Single object detection result
 */
typedef struct uv_msgs__msg__Detection
{
  uint16_t class_id;
  float confidence;
  /// Center pixel X
  float pixel_x;
  /// Center pixel Y
  float pixel_y;
  /// Bounding box top-left X
  float bbox_x1;
  /// Bounding box top-left Y
  float bbox_y1;
  /// Bounding box bottom-right X
  float bbox_x2;
  /// Bounding box bottom-right Y
  float bbox_y2;
} uv_msgs__msg__Detection;

// Struct for a sequence of uv_msgs__msg__Detection.
typedef struct uv_msgs__msg__Detection__Sequence
{
  uv_msgs__msg__Detection * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} uv_msgs__msg__Detection__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // UV_MSGS__MSG__DETAIL__DETECTION__STRUCT_H_
