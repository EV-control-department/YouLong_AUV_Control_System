// generated from rosidl_generator_c/resource/idl__description.c.em
// with input from uv_msgs:msg/ObjectPosition.idl
// generated code does not contain a copyright notice

#include "uv_msgs/msg/detail/object_position__functions.h"

ROSIDL_GENERATOR_C_PUBLIC_uv_msgs
const rosidl_type_hash_t *
uv_msgs__msg__ObjectPosition__get_type_hash(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0x9a, 0xd6, 0x16, 0x56, 0x26, 0xea, 0x45, 0x08,
      0xdd, 0x8e, 0xb4, 0x87, 0x66, 0x50, 0xf5, 0xa2,
      0xa4, 0xeb, 0x3e, 0x86, 0x8d, 0x2e, 0x1f, 0xc4,
      0xe1, 0x4e, 0xc0, 0x9f, 0x42, 0xc0, 0x72, 0x04,
    }};
  return &hash;
}

#include <assert.h>
#include <string.h>

// Include directives for referenced types

// Hashes for external referenced types
#ifndef NDEBUG
#endif

static char uv_msgs__msg__ObjectPosition__TYPE_NAME[] = "uv_msgs/msg/ObjectPosition";

// Define type names, field names, and default values
static char uv_msgs__msg__ObjectPosition__FIELD_NAME__class_id[] = "class_id";
static char uv_msgs__msg__ObjectPosition__FIELD_NAME__class_name[] = "class_name";
static char uv_msgs__msg__ObjectPosition__FIELD_NAME__world_x[] = "world_x";
static char uv_msgs__msg__ObjectPosition__FIELD_NAME__world_y[] = "world_y";
static char uv_msgs__msg__ObjectPosition__FIELD_NAME__world_z[] = "world_z";
static char uv_msgs__msg__ObjectPosition__FIELD_NAME__confidence[] = "confidence";
static char uv_msgs__msg__ObjectPosition__FIELD_NAME__num_observations[] = "num_observations";

static rosidl_runtime_c__type_description__Field uv_msgs__msg__ObjectPosition__FIELDS[] = {
  {
    {uv_msgs__msg__ObjectPosition__FIELD_NAME__class_id, 8, 8},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT16,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {uv_msgs__msg__ObjectPosition__FIELD_NAME__class_name, 10, 10},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_STRING,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {uv_msgs__msg__ObjectPosition__FIELD_NAME__world_x, 7, 7},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {uv_msgs__msg__ObjectPosition__FIELD_NAME__world_y, 7, 7},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {uv_msgs__msg__ObjectPosition__FIELD_NAME__world_z, 7, 7},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {uv_msgs__msg__ObjectPosition__FIELD_NAME__confidence, 10, 10},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {uv_msgs__msg__ObjectPosition__FIELD_NAME__num_observations, 16, 16},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT32,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
};

const rosidl_runtime_c__type_description__TypeDescription *
uv_msgs__msg__ObjectPosition__get_type_description(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {uv_msgs__msg__ObjectPosition__TYPE_NAME, 26, 26},
      {uv_msgs__msg__ObjectPosition__FIELDS, 7, 7},
    },
    {NULL, 0, 0},
  };
  if (!constructed) {
    constructed = true;
  }
  return &description;
}

static char toplevel_type_raw_source[] =
  "# World position of a tracked object\n"
  "\n"
  "uint16 class_id\n"
  "string class_name\n"
  "float32 world_x       # NED North (m)\n"
  "float32 world_y       # NED East (m)\n"
  "float32 world_z       # Depth (m)\n"
  "float32 confidence    # Position estimate confidence\n"
  "uint32 num_observations";

static char msg_encoding[] = "msg";

// Define all individual source functions

const rosidl_runtime_c__type_description__TypeSource *
uv_msgs__msg__ObjectPosition__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {uv_msgs__msg__ObjectPosition__TYPE_NAME, 26, 26},
    {msg_encoding, 3, 3},
    {toplevel_type_raw_source, 258, 258},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
uv_msgs__msg__ObjectPosition__get_type_description_sources(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[1];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 1, 1};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *uv_msgs__msg__ObjectPosition__get_individual_type_description_source(NULL),
    constructed = true;
  }
  return &source_sequence;
}
