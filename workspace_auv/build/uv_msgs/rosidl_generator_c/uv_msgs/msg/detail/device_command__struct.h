// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from uv_msgs:msg/DeviceCommand.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "uv_msgs/msg/device_command.h"


#ifndef UV_MSGS__MSG__DETAIL__DEVICE_COMMAND__STRUCT_H_
#define UV_MSGS__MSG__DETAIL__DEVICE_COMMAND__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

// Constants defined in the message

/// Constant 'DEVICE_LED'.
enum
{
  uv_msgs__msg__DeviceCommand__DEVICE_LED = 0
};

/// Constant 'DEVICE_ARM'.
enum
{
  uv_msgs__msg__DeviceCommand__DEVICE_ARM = 1
};

/// Constant 'DEVICE_SERVO'.
enum
{
  uv_msgs__msg__DeviceCommand__DEVICE_SERVO = 2
};

/// Struct defined in msg/DeviceCommand in the package uv_msgs.
/**
  * Device command for LED, arm, servo, etc.
 */
typedef struct uv_msgs__msg__DeviceCommand
{
  uint8_t device_type;
  /// Device-specific command code
  uint8_t command;
  /// Device-specific value
  float value;
} uv_msgs__msg__DeviceCommand;

// Struct for a sequence of uv_msgs__msg__DeviceCommand.
typedef struct uv_msgs__msg__DeviceCommand__Sequence
{
  uv_msgs__msg__DeviceCommand * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} uv_msgs__msg__DeviceCommand__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // UV_MSGS__MSG__DETAIL__DEVICE_COMMAND__STRUCT_H_
