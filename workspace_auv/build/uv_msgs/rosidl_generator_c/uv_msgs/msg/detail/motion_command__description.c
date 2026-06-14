// generated from rosidl_generator_c/resource/idl__description.c.em
// with input from uv_msgs:msg/MotionCommand.idl
// generated code does not contain a copyright notice

#include "uv_msgs/msg/detail/motion_command__functions.h"

ROSIDL_GENERATOR_C_PUBLIC_uv_msgs
const rosidl_type_hash_t *
uv_msgs__msg__MotionCommand__get_type_hash(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0x6f, 0xed, 0x03, 0xc0, 0x5b, 0x10, 0x7a, 0x42,
      0xcd, 0x8f, 0x91, 0x93, 0xbd, 0xd1, 0x6b, 0xa5,
      0xb8, 0x23, 0xed, 0x5c, 0x4f, 0x2d, 0x6f, 0xc0,
      0xf9, 0x1c, 0x12, 0x05, 0x6c, 0xfa, 0x85, 0x4e,
    }};
  return &hash;
}

#include <assert.h>
#include <string.h>

// Include directives for referenced types

// Hashes for external referenced types
#ifndef NDEBUG
#endif

static char uv_msgs__msg__MotionCommand__TYPE_NAME[] = "uv_msgs/msg/MotionCommand";

// Define type names, field names, and default values
static char uv_msgs__msg__MotionCommand__FIELD_NAME__mode[] = "mode";
static char uv_msgs__msg__MotionCommand__FIELD_NAME__type_mask[] = "type_mask";
static char uv_msgs__msg__MotionCommand__FIELD_NAME__x[] = "x";
static char uv_msgs__msg__MotionCommand__FIELD_NAME__y[] = "y";
static char uv_msgs__msg__MotionCommand__FIELD_NAME__z[] = "z";
static char uv_msgs__msg__MotionCommand__FIELD_NAME__yaw[] = "yaw";

static rosidl_runtime_c__type_description__Field uv_msgs__msg__MotionCommand__FIELDS[] = {
  {
    {uv_msgs__msg__MotionCommand__FIELD_NAME__mode, 4, 4},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT8,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {uv_msgs__msg__MotionCommand__FIELD_NAME__type_mask, 9, 9},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT8,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {uv_msgs__msg__MotionCommand__FIELD_NAME__x, 1, 1},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {uv_msgs__msg__MotionCommand__FIELD_NAME__y, 1, 1},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {uv_msgs__msg__MotionCommand__FIELD_NAME__z, 1, 1},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {uv_msgs__msg__MotionCommand__FIELD_NAME__yaw, 3, 3},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
};

const rosidl_runtime_c__type_description__TypeDescription *
uv_msgs__msg__MotionCommand__get_type_description(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {uv_msgs__msg__MotionCommand__TYPE_NAME, 25, 25},
      {uv_msgs__msg__MotionCommand__FIELDS, 6, 6},
    },
    {NULL, 0, 0},
  };
  if (!constructed) {
    constructed = true;
  }
  return &description;
}

static char toplevel_type_raw_source[] =
  "# Motion command from control layer to sim_bridge\n"
  "# Replaces ZitSetpoint with a cleaner interface\n"
  "\n"
  "uint8 MODE_POS_WORLD      = 0   # Absolute world NED position\n"
  "uint8 MODE_POS_BODY       = 1   # Absolute body-frame position\n"
  "uint8 MODE_VEL_WORLD      = 2   # World-frame velocity\n"
  "uint8 MODE_VEL_BODY       = 3   # Body-frame velocity\n"
  "uint8 MODE_FORCE_BODY     = 4   # Direct body-frame force\n"
  "uint8 MODE_POS_WORLD_STEP = 5   # Incremental world position step\n"
  "uint8 MODE_POS_BODY_STEP  = 6   # Incremental body position step\n"
  "uint8 MODE_VEL_WORLD_STEP = 7   # Incremental world velocity step\n"
  "uint8 MODE_VEL_BODY_STEP  = 8   # Incremental body velocity step\n"
  "\n"
  "uint8 mode            # One of the MODE_* constants\n"
  "uint8 type_mask       # Bitmask: bit0=x, bit1=y, bit2=z, bit3=yaw (1=ignore axis)\n"
  "\n"
  "float32 x\n"
  "float32 y\n"
  "float32 z\n"
  "float32 yaw           # Radians";

static char msg_encoding[] = "msg";

// Define all individual source functions

const rosidl_runtime_c__type_description__TypeSource *
uv_msgs__msg__MotionCommand__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {uv_msgs__msg__MotionCommand__TYPE_NAME, 25, 25},
    {msg_encoding, 3, 3},
    {toplevel_type_raw_source, 851, 851},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
uv_msgs__msg__MotionCommand__get_type_description_sources(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[1];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 1, 1};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *uv_msgs__msg__MotionCommand__get_individual_type_description_source(NULL),
    constructed = true;
  }
  return &source_sequence;
}
