// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from uv_msgs:msg/TaskStatus.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "uv_msgs/msg/task_status.h"


#ifndef UV_MSGS__MSG__DETAIL__TASK_STATUS__STRUCT_H_
#define UV_MSGS__MSG__DETAIL__TASK_STATUS__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

// Constants defined in the message

/// Constant 'STATUS_IDLE'.
enum
{
  uv_msgs__msg__TaskStatus__STATUS_IDLE = 0
};

/// Constant 'STATUS_RUNNING'.
enum
{
  uv_msgs__msg__TaskStatus__STATUS_RUNNING = 1
};

/// Constant 'STATUS_PAUSED'.
enum
{
  uv_msgs__msg__TaskStatus__STATUS_PAUSED = 2
};

/// Constant 'STATUS_DONE'.
enum
{
  uv_msgs__msg__TaskStatus__STATUS_DONE = 3
};

/// Constant 'STATUS_ERROR'.
enum
{
  uv_msgs__msg__TaskStatus__STATUS_ERROR = 4
};

// Include directives for member types
// Member 'current_task_name'
// Member 'error_message'
#include "rosidl_runtime_c/string.h"

/// Struct defined in msg/TaskStatus in the package uv_msgs.
/**
  * Task execution status
 */
typedef struct uv_msgs__msg__TaskStatus
{
  uint8_t status;
  uint32_t current_task_index;
  uint32_t total_tasks;
  rosidl_runtime_c__String current_task_name;
  rosidl_runtime_c__String error_message;
} uv_msgs__msg__TaskStatus;

// Struct for a sequence of uv_msgs__msg__TaskStatus.
typedef struct uv_msgs__msg__TaskStatus__Sequence
{
  uv_msgs__msg__TaskStatus * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} uv_msgs__msg__TaskStatus__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // UV_MSGS__MSG__DETAIL__TASK_STATUS__STRUCT_H_
