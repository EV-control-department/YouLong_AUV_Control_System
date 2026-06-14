// generated from rosidl_generator_c/resource/idl__description.c.em
// with input from uv_msgs:msg/Waypoint.idl
// generated code does not contain a copyright notice

#include "uv_msgs/msg/detail/waypoint__functions.h"

ROSIDL_GENERATOR_C_PUBLIC_uv_msgs
const rosidl_type_hash_t *
uv_msgs__msg__Waypoint__get_type_hash(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0x10, 0x16, 0xba, 0x7a, 0x48, 0x7e, 0x21, 0xfc,
      0x67, 0xab, 0x02, 0x2b, 0xdd, 0xea, 0xbf, 0x1b,
      0xac, 0x2e, 0x35, 0xb7, 0x34, 0xc5, 0xc9, 0x32,
      0xa2, 0x8a, 0x2a, 0x17, 0x8c, 0xa8, 0xec, 0xba,
    }};
  return &hash;
}

#include <assert.h>
#include <string.h>

// Include directives for referenced types

// Hashes for external referenced types
#ifndef NDEBUG
#endif

static char uv_msgs__msg__Waypoint__TYPE_NAME[] = "uv_msgs/msg/Waypoint";

// Define type names, field names, and default values
static char uv_msgs__msg__Waypoint__FIELD_NAME__x[] = "x";
static char uv_msgs__msg__Waypoint__FIELD_NAME__y[] = "y";
static char uv_msgs__msg__Waypoint__FIELD_NAME__z[] = "z";
static char uv_msgs__msg__Waypoint__FIELD_NAME__yaw[] = "yaw";
static char uv_msgs__msg__Waypoint__FIELD_NAME__speed[] = "speed";

static rosidl_runtime_c__type_description__Field uv_msgs__msg__Waypoint__FIELDS[] = {
  {
    {uv_msgs__msg__Waypoint__FIELD_NAME__x, 1, 1},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {uv_msgs__msg__Waypoint__FIELD_NAME__y, 1, 1},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {uv_msgs__msg__Waypoint__FIELD_NAME__z, 1, 1},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {uv_msgs__msg__Waypoint__FIELD_NAME__yaw, 3, 3},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {uv_msgs__msg__Waypoint__FIELD_NAME__speed, 5, 5},
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
uv_msgs__msg__Waypoint__get_type_description(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {uv_msgs__msg__Waypoint__TYPE_NAME, 20, 20},
      {uv_msgs__msg__Waypoint__FIELDS, 5, 5},
    },
    {NULL, 0, 0},
  };
  if (!constructed) {
    constructed = true;
  }
  return &description;
}

static char toplevel_type_raw_source[] =
  "# Single navigation waypoint\n"
  "\n"
  "float32 x            # NED North (m)\n"
  "float32 y            # NED East (m)\n"
  "float32 z            # Depth (m)\n"
  "float32 yaw          # Heading (rad)\n"
  "float32 speed        # Target speed (m/s)";

static char msg_encoding[] = "msg";

// Define all individual source functions

const rosidl_runtime_c__type_description__TypeSource *
uv_msgs__msg__Waypoint__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {uv_msgs__msg__Waypoint__TYPE_NAME, 20, 20},
    {msg_encoding, 3, 3},
    {toplevel_type_raw_source, 215, 215},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
uv_msgs__msg__Waypoint__get_type_description_sources(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[1];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 1, 1};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *uv_msgs__msg__Waypoint__get_individual_type_description_source(NULL),
    constructed = true;
  }
  return &source_sequence;
}
