// generated from rosidl_generator_c/resource/idl__description.c.em
// with input from uv_msgs:msg/PidState.idl
// generated code does not contain a copyright notice

#include "uv_msgs/msg/detail/pid_state__functions.h"

ROSIDL_GENERATOR_C_PUBLIC_uv_msgs
const rosidl_type_hash_t *
uv_msgs__msg__PidState__get_type_hash(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0x54, 0xad, 0x8d, 0x13, 0x4c, 0x89, 0x78, 0x9e,
      0x12, 0xf2, 0x91, 0x8b, 0x79, 0xdc, 0xf4, 0x8c,
      0xe3, 0x84, 0x41, 0xe6, 0xe3, 0xdf, 0x9b, 0x0d,
      0xb7, 0x0e, 0x62, 0x03, 0x88, 0xff, 0x7e, 0xc1,
    }};
  return &hash;
}

#include <assert.h>
#include <string.h>

// Include directives for referenced types
#include "uv_msgs/msg/detail/pid_gains__functions.h"

// Hashes for external referenced types
#ifndef NDEBUG
static const rosidl_type_hash_t uv_msgs__msg__PidGains__EXPECTED_HASH = {1, {
    0x0b, 0xad, 0x6e, 0x07, 0xa9, 0xb5, 0xe2, 0x83,
    0xac, 0xe5, 0xda, 0x50, 0x71, 0xc2, 0x7f, 0x30,
    0x25, 0x4e, 0x8b, 0x9c, 0x26, 0x74, 0x02, 0xec,
    0xa5, 0x35, 0xde, 0x6c, 0x50, 0xb8, 0x9f, 0x96,
  }};
#endif

static char uv_msgs__msg__PidState__TYPE_NAME[] = "uv_msgs/msg/PidState";
static char uv_msgs__msg__PidGains__TYPE_NAME[] = "uv_msgs/msg/PidGains";

// Define type names, field names, and default values
static char uv_msgs__msg__PidState__FIELD_NAME__speed_mag[] = "speed_mag";
static char uv_msgs__msg__PidState__FIELD_NAME__vel_x[] = "vel_x";
static char uv_msgs__msg__PidState__FIELD_NAME__vel_y[] = "vel_y";
static char uv_msgs__msg__PidState__FIELD_NAME__pos_z[] = "pos_z";
static char uv_msgs__msg__PidState__FIELD_NAME__yaw[] = "yaw";
static char uv_msgs__msg__PidState__FIELD_NAME__yaw_rate[] = "yaw_rate";
static char uv_msgs__msg__PidState__FIELD_NAME__error_length[] = "error_length";
static char uv_msgs__msg__PidState__FIELD_NAME__error_angle_deg[] = "error_angle_deg";
static char uv_msgs__msg__PidState__FIELD_NAME__speed_cmd[] = "speed_cmd";
static char uv_msgs__msg__PidState__FIELD_NAME__vx_body_cmd[] = "vx_body_cmd";
static char uv_msgs__msg__PidState__FIELD_NAME__vy_body_cmd[] = "vy_body_cmd";

static rosidl_runtime_c__type_description__Field uv_msgs__msg__PidState__FIELDS[] = {
  {
    {uv_msgs__msg__PidState__FIELD_NAME__speed_mag, 9, 9},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE,
      0,
      0,
      {uv_msgs__msg__PidGains__TYPE_NAME, 20, 20},
    },
    {NULL, 0, 0},
  },
  {
    {uv_msgs__msg__PidState__FIELD_NAME__vel_x, 5, 5},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE,
      0,
      0,
      {uv_msgs__msg__PidGains__TYPE_NAME, 20, 20},
    },
    {NULL, 0, 0},
  },
  {
    {uv_msgs__msg__PidState__FIELD_NAME__vel_y, 5, 5},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE,
      0,
      0,
      {uv_msgs__msg__PidGains__TYPE_NAME, 20, 20},
    },
    {NULL, 0, 0},
  },
  {
    {uv_msgs__msg__PidState__FIELD_NAME__pos_z, 5, 5},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE,
      0,
      0,
      {uv_msgs__msg__PidGains__TYPE_NAME, 20, 20},
    },
    {NULL, 0, 0},
  },
  {
    {uv_msgs__msg__PidState__FIELD_NAME__yaw, 3, 3},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE,
      0,
      0,
      {uv_msgs__msg__PidGains__TYPE_NAME, 20, 20},
    },
    {NULL, 0, 0},
  },
  {
    {uv_msgs__msg__PidState__FIELD_NAME__yaw_rate, 8, 8},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE,
      0,
      0,
      {uv_msgs__msg__PidGains__TYPE_NAME, 20, 20},
    },
    {NULL, 0, 0},
  },
  {
    {uv_msgs__msg__PidState__FIELD_NAME__error_length, 12, 12},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {uv_msgs__msg__PidState__FIELD_NAME__error_angle_deg, 15, 15},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {uv_msgs__msg__PidState__FIELD_NAME__speed_cmd, 9, 9},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {uv_msgs__msg__PidState__FIELD_NAME__vx_body_cmd, 11, 11},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {uv_msgs__msg__PidState__FIELD_NAME__vy_body_cmd, 11, 11},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
};

static rosidl_runtime_c__type_description__IndividualTypeDescription uv_msgs__msg__PidState__REFERENCED_TYPE_DESCRIPTIONS[] = {
  {
    {uv_msgs__msg__PidGains__TYPE_NAME, 20, 20},
    {NULL, 0, 0},
  },
};

const rosidl_runtime_c__type_description__TypeDescription *
uv_msgs__msg__PidState__get_type_description(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {uv_msgs__msg__PidState__TYPE_NAME, 20, 20},
      {uv_msgs__msg__PidState__FIELDS, 11, 11},
    },
    {uv_msgs__msg__PidState__REFERENCED_TYPE_DESCRIPTIONS, 1, 1},
  };
  if (!constructed) {
    assert(0 == memcmp(&uv_msgs__msg__PidGains__EXPECTED_HASH, uv_msgs__msg__PidGains__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[0].fields = uv_msgs__msg__PidGains__get_type_description(NULL)->type_description.fields;
    constructed = true;
  }
  return &description;
}

static char toplevel_type_raw_source[] =
  "# PID controller telemetry (published for debugging/tuning)\n"
  "\n"
  "PidGains speed_mag    # Outer loop gains\n"
  "PidGains vel_x        # Inner loop X gains\n"
  "PidGains vel_y        # Inner loop Y gains\n"
  "PidGains pos_z        # Z axis gains\n"
  "PidGains yaw          # Yaw position loop\n"
  "PidGains yaw_rate     # Yaw rate loop\n"
  "\n"
  "float32 error_length\n"
  "float32 error_angle_deg\n"
  "float32 speed_cmd\n"
  "float32 vx_body_cmd\n"
  "float32 vy_body_cmd";

static char msg_encoding[] = "msg";

// Define all individual source functions

const rosidl_runtime_c__type_description__TypeSource *
uv_msgs__msg__PidState__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {uv_msgs__msg__PidState__TYPE_NAME, 20, 20},
    {msg_encoding, 3, 3},
    {toplevel_type_raw_source, 409, 409},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
uv_msgs__msg__PidState__get_type_description_sources(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[2];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 2, 2};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *uv_msgs__msg__PidState__get_individual_type_description_source(NULL),
    sources[1] = *uv_msgs__msg__PidGains__get_individual_type_description_source(NULL);
    constructed = true;
  }
  return &source_sequence;
}
