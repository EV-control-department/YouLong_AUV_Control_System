// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from uv_msgs:msg/MotionCommand.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "uv_msgs/msg/motion_command.h"


#ifndef UV_MSGS__MSG__DETAIL__MOTION_COMMAND__STRUCT_H_
#define UV_MSGS__MSG__DETAIL__MOTION_COMMAND__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

// Constants defined in the message

/// Constant 'MODE_POS_WORLD'.
/**
  * Absolute world NED position
 */
enum
{
  uv_msgs__msg__MotionCommand__MODE_POS_WORLD = 0
};

/// Constant 'MODE_POS_BODY'.
/**
  * Absolute body-frame position
 */
enum
{
  uv_msgs__msg__MotionCommand__MODE_POS_BODY = 1
};

/// Constant 'MODE_VEL_WORLD'.
/**
  * World-frame velocity
 */
enum
{
  uv_msgs__msg__MotionCommand__MODE_VEL_WORLD = 2
};

/// Constant 'MODE_VEL_BODY'.
/**
  * Body-frame velocity
 */
enum
{
  uv_msgs__msg__MotionCommand__MODE_VEL_BODY = 3
};

/// Constant 'MODE_FORCE_BODY'.
/**
  * Direct body-frame force
 */
enum
{
  uv_msgs__msg__MotionCommand__MODE_FORCE_BODY = 4
};

/// Constant 'MODE_POS_WORLD_STEP'.
/**
  * Incremental world position step
 */
enum
{
  uv_msgs__msg__MotionCommand__MODE_POS_WORLD_STEP = 5
};

/// Constant 'MODE_POS_BODY_STEP'.
/**
  * Incremental body position step
 */
enum
{
  uv_msgs__msg__MotionCommand__MODE_POS_BODY_STEP = 6
};

/// Constant 'MODE_VEL_WORLD_STEP'.
/**
  * Incremental world velocity step
 */
enum
{
  uv_msgs__msg__MotionCommand__MODE_VEL_WORLD_STEP = 7
};

/// Constant 'MODE_VEL_BODY_STEP'.
/**
  * Incremental body velocity step
 */
enum
{
  uv_msgs__msg__MotionCommand__MODE_VEL_BODY_STEP = 8
};

/// Struct defined in msg/MotionCommand in the package uv_msgs.
/**
  * Motion command from control layer to sim_bridge
  * Replaces ZitSetpoint with a cleaner interface
 */
typedef struct uv_msgs__msg__MotionCommand
{
  /// One of the MODE_* constants
  uint8_t mode;
  /// Bitmask: bit0=x, bit1=y, bit2=z, bit3=yaw (1=ignore axis)
  uint8_t type_mask;
  float x;
  float y;
  float z;
  /// Radians
  float yaw;
} uv_msgs__msg__MotionCommand;

// Struct for a sequence of uv_msgs__msg__MotionCommand.
typedef struct uv_msgs__msg__MotionCommand__Sequence
{
  uv_msgs__msg__MotionCommand * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} uv_msgs__msg__MotionCommand__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // UV_MSGS__MSG__DETAIL__MOTION_COMMAND__STRUCT_H_
