// generated from rosidl_generator_c/resource/idl__description.c.em
// with input from uv_msgs:msg/AuvState.idl
// generated code does not contain a copyright notice

#include "uv_msgs/msg/detail/auv_state__functions.h"

ROSIDL_GENERATOR_C_PUBLIC_uv_msgs
const rosidl_type_hash_t *
uv_msgs__msg__AuvState__get_type_hash(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0xbd, 0x23, 0xe7, 0x38, 0x3b, 0xcb, 0x95, 0x9e,
      0x1e, 0xa3, 0x9b, 0x80, 0xda, 0x8b, 0x7c, 0x79,
      0x8f, 0x0c, 0xa6, 0xbb, 0x3c, 0x5a, 0xc5, 0xb2,
      0xe2, 0x35, 0x12, 0x64, 0xaa, 0xd8, 0x1a, 0xe5,
    }};
  return &hash;
}

#include <assert.h>
#include <string.h>

// Include directives for referenced types

// Hashes for external referenced types
#ifndef NDEBUG
#endif

static char uv_msgs__msg__AuvState__TYPE_NAME[] = "uv_msgs/msg/AuvState";

// Define type names, field names, and default values
static char uv_msgs__msg__AuvState__FIELD_NAME__pos_x[] = "pos_x";
static char uv_msgs__msg__AuvState__FIELD_NAME__pos_y[] = "pos_y";
static char uv_msgs__msg__AuvState__FIELD_NAME__pos_z[] = "pos_z";
static char uv_msgs__msg__AuvState__FIELD_NAME__yaw[] = "yaw";
static char uv_msgs__msg__AuvState__FIELD_NAME__vel_x[] = "vel_x";
static char uv_msgs__msg__AuvState__FIELD_NAME__vel_y[] = "vel_y";
static char uv_msgs__msg__AuvState__FIELD_NAME__vel_z[] = "vel_z";
static char uv_msgs__msg__AuvState__FIELD_NAME__yaw_rate[] = "yaw_rate";
static char uv_msgs__msg__AuvState__FIELD_NAME__vel_world_x[] = "vel_world_x";
static char uv_msgs__msg__AuvState__FIELD_NAME__vel_world_y[] = "vel_world_y";
static char uv_msgs__msg__AuvState__FIELD_NAME__armed[] = "armed";
static char uv_msgs__msg__AuvState__FIELD_NAME__control_mode[] = "control_mode";
static char uv_msgs__msg__AuvState__FIELD_NAME__battery_voltage[] = "battery_voltage";
static char uv_msgs__msg__AuvState__FIELD_NAME__error_flags[] = "error_flags";
static char uv_msgs__msg__AuvState__FIELD_NAME__cycle_time_ms[] = "cycle_time_ms";
static char uv_msgs__msg__AuvState__FIELD_NAME__target_x[] = "target_x";
static char uv_msgs__msg__AuvState__FIELD_NAME__target_y[] = "target_y";
static char uv_msgs__msg__AuvState__FIELD_NAME__target_z[] = "target_z";
static char uv_msgs__msg__AuvState__FIELD_NAME__target_yaw[] = "target_yaw";

static rosidl_runtime_c__type_description__Field uv_msgs__msg__AuvState__FIELDS[] = {
  {
    {uv_msgs__msg__AuvState__FIELD_NAME__pos_x, 5, 5},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {uv_msgs__msg__AuvState__FIELD_NAME__pos_y, 5, 5},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {uv_msgs__msg__AuvState__FIELD_NAME__pos_z, 5, 5},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {uv_msgs__msg__AuvState__FIELD_NAME__yaw, 3, 3},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {uv_msgs__msg__AuvState__FIELD_NAME__vel_x, 5, 5},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {uv_msgs__msg__AuvState__FIELD_NAME__vel_y, 5, 5},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {uv_msgs__msg__AuvState__FIELD_NAME__vel_z, 5, 5},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {uv_msgs__msg__AuvState__FIELD_NAME__yaw_rate, 8, 8},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {uv_msgs__msg__AuvState__FIELD_NAME__vel_world_x, 11, 11},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {uv_msgs__msg__AuvState__FIELD_NAME__vel_world_y, 11, 11},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {uv_msgs__msg__AuvState__FIELD_NAME__armed, 5, 5},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_BOOLEAN,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {uv_msgs__msg__AuvState__FIELD_NAME__control_mode, 12, 12},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT8,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {uv_msgs__msg__AuvState__FIELD_NAME__battery_voltage, 15, 15},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {uv_msgs__msg__AuvState__FIELD_NAME__error_flags, 11, 11},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT32,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {uv_msgs__msg__AuvState__FIELD_NAME__cycle_time_ms, 13, 13},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {uv_msgs__msg__AuvState__FIELD_NAME__target_x, 8, 8},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {uv_msgs__msg__AuvState__FIELD_NAME__target_y, 8, 8},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {uv_msgs__msg__AuvState__FIELD_NAME__target_z, 8, 8},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {uv_msgs__msg__AuvState__FIELD_NAME__target_yaw, 10, 10},
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
uv_msgs__msg__AuvState__get_type_description(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {uv_msgs__msg__AuvState__TYPE_NAME, 20, 20},
      {uv_msgs__msg__AuvState__FIELDS, 19, 19},
    },
    {NULL, 0, 0},
  };
  if (!constructed) {
    constructed = true;
  }
  return &description;
}

static char toplevel_type_raw_source[] =
  "# Comprehensive robot state published by sim_bridge (or real MCU bridge)\n"
  "\n"
  "# Position in world NED frame\n"
  "float32 pos_x         # North (m)\n"
  "float32 pos_y         # East (m)\n"
  "float32 pos_z         # Down (m, positive = deeper)\n"
  "float32 yaw           # Yaw in radians (NED frame)\n"
  "\n"
  "# Velocity in body frame (FRD: Forward-Right-Down)\n"
  "float32 vel_x         # Forward (m/s)\n"
  "float32 vel_y         # Right (m/s)\n"
  "float32 vel_z         # Down (m/s)\n"
  "float32 yaw_rate      # Yaw rate (rad/s)\n"
  "\n"
  "# Velocity in world frame (for external use)\n"
  "float32 vel_world_x   # North (m/s)\n"
  "float32 vel_world_y   # East (m/s)\n"
  "\n"
  "# Status\n"
  "bool armed\n"
  "uint8 control_mode    # 0=none, 1=pos, 2=vel, 3=force\n"
  "float32 battery_voltage\n"
  "uint32 error_flags\n"
  "float32 cycle_time_ms\n"
  "\n"
  "# Current target (for monitoring)\n"
  "float32 target_x\n"
  "float32 target_y\n"
  "float32 target_z\n"
  "float32 target_yaw";

static char msg_encoding[] = "msg";

// Define all individual source functions

const rosidl_runtime_c__type_description__TypeSource *
uv_msgs__msg__AuvState__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {uv_msgs__msg__AuvState__TYPE_NAME, 20, 20},
    {msg_encoding, 3, 3},
    {toplevel_type_raw_source, 838, 838},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
uv_msgs__msg__AuvState__get_type_description_sources(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[1];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 1, 1};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *uv_msgs__msg__AuvState__get_individual_type_description_source(NULL),
    constructed = true;
  }
  return &source_sequence;
}
