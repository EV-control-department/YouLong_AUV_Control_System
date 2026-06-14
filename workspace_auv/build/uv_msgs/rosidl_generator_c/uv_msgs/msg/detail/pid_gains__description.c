// generated from rosidl_generator_c/resource/idl__description.c.em
// with input from uv_msgs:msg/PidGains.idl
// generated code does not contain a copyright notice

#include "uv_msgs/msg/detail/pid_gains__functions.h"

ROSIDL_GENERATOR_C_PUBLIC_uv_msgs
const rosidl_type_hash_t *
uv_msgs__msg__PidGains__get_type_hash(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0x0b, 0xad, 0x6e, 0x07, 0xa9, 0xb5, 0xe2, 0x83,
      0xac, 0xe5, 0xda, 0x50, 0x71, 0xc2, 0x7f, 0x30,
      0x25, 0x4e, 0x8b, 0x9c, 0x26, 0x74, 0x02, 0xec,
      0xa5, 0x35, 0xde, 0x6c, 0x50, 0xb8, 0x9f, 0x96,
    }};
  return &hash;
}

#include <assert.h>
#include <string.h>

// Include directives for referenced types

// Hashes for external referenced types
#ifndef NDEBUG
#endif

static char uv_msgs__msg__PidGains__TYPE_NAME[] = "uv_msgs/msg/PidGains";

// Define type names, field names, and default values
static char uv_msgs__msg__PidGains__FIELD_NAME__name[] = "name";
static char uv_msgs__msg__PidGains__FIELD_NAME__kp[] = "kp";
static char uv_msgs__msg__PidGains__FIELD_NAME__ki[] = "ki";
static char uv_msgs__msg__PidGains__FIELD_NAME__kd[] = "kd";
static char uv_msgs__msg__PidGains__FIELD_NAME__i_limit[] = "i_limit";
static char uv_msgs__msg__PidGains__FIELD_NAME__output_limit[] = "output_limit";
static char uv_msgs__msg__PidGains__FIELD_NAME__feedforward[] = "feedforward";

static rosidl_runtime_c__type_description__Field uv_msgs__msg__PidGains__FIELDS[] = {
  {
    {uv_msgs__msg__PidGains__FIELD_NAME__name, 4, 4},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_STRING,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {uv_msgs__msg__PidGains__FIELD_NAME__kp, 2, 2},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {uv_msgs__msg__PidGains__FIELD_NAME__ki, 2, 2},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {uv_msgs__msg__PidGains__FIELD_NAME__kd, 2, 2},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {uv_msgs__msg__PidGains__FIELD_NAME__i_limit, 7, 7},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {uv_msgs__msg__PidGains__FIELD_NAME__output_limit, 12, 12},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {uv_msgs__msg__PidGains__FIELD_NAME__feedforward, 11, 11},
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
uv_msgs__msg__PidGains__get_type_description(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {uv_msgs__msg__PidGains__TYPE_NAME, 20, 20},
      {uv_msgs__msg__PidGains__FIELDS, 7, 7},
    },
    {NULL, 0, 0},
  };
  if (!constructed) {
    constructed = true;
  }
  return &description;
}

static char toplevel_type_raw_source[] =
  "# PID controller gains\n"
  "\n"
  "string name\n"
  "float32 kp\n"
  "float32 ki\n"
  "float32 kd\n"
  "float32 i_limit\n"
  "float32 output_limit\n"
  "float32 feedforward";

static char msg_encoding[] = "msg";

// Define all individual source functions

const rosidl_runtime_c__type_description__TypeSource *
uv_msgs__msg__PidGains__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {uv_msgs__msg__PidGains__TYPE_NAME, 20, 20},
    {msg_encoding, 3, 3},
    {toplevel_type_raw_source, 126, 126},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
uv_msgs__msg__PidGains__get_type_description_sources(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[1];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 1, 1};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *uv_msgs__msg__PidGains__get_individual_type_description_source(NULL),
    constructed = true;
  }
  return &source_sequence;
}
