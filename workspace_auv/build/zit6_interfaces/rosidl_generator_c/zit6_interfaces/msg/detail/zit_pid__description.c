// generated from rosidl_generator_c/resource/idl__description.c.em
// with input from zit6_interfaces:msg/ZitPid.idl
// generated code does not contain a copyright notice

#include "zit6_interfaces/msg/detail/zit_pid__functions.h"

ROSIDL_GENERATOR_C_PUBLIC_zit6_interfaces
const rosidl_type_hash_t *
zit6_interfaces__msg__ZitPid__get_type_hash(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0x8e, 0x4e, 0x85, 0xee, 0xb5, 0xdd, 0xb5, 0x01,
      0xc2, 0x55, 0x64, 0xd5, 0x52, 0x1e, 0xdb, 0xed,
      0xb0, 0xe3, 0xf8, 0x78, 0xfb, 0x18, 0xdd, 0x7b,
      0x05, 0xd5, 0x09, 0x82, 0xdd, 0x83, 0x7c, 0x03,
    }};
  return &hash;
}

#include <assert.h>
#include <string.h>

// Include directives for referenced types

// Hashes for external referenced types
#ifndef NDEBUG
#endif

static char zit6_interfaces__msg__ZitPid__TYPE_NAME[] = "zit6_interfaces/msg/ZitPid";

// Define type names, field names, and default values
static char zit6_interfaces__msg__ZitPid__FIELD_NAME__axis[] = "axis";
static char zit6_interfaces__msg__ZitPid__FIELD_NAME__is_pos_ring[] = "is_pos_ring";
static char zit6_interfaces__msg__ZitPid__FIELD_NAME__kp[] = "kp";
static char zit6_interfaces__msg__ZitPid__FIELD_NAME__ki[] = "ki";
static char zit6_interfaces__msg__ZitPid__FIELD_NAME__kd[] = "kd";
static char zit6_interfaces__msg__ZitPid__FIELD_NAME__i_limit[] = "i_limit";
static char zit6_interfaces__msg__ZitPid__FIELD_NAME__out_limit[] = "out_limit";
static char zit6_interfaces__msg__ZitPid__FIELD_NAME__max_v[] = "max_v";
static char zit6_interfaces__msg__ZitPid__FIELD_NAME__max_a[] = "max_a";

static rosidl_runtime_c__type_description__Field zit6_interfaces__msg__ZitPid__FIELDS[] = {
  {
    {zit6_interfaces__msg__ZitPid__FIELD_NAME__axis, 4, 4},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT8,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {zit6_interfaces__msg__ZitPid__FIELD_NAME__is_pos_ring, 11, 11},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_BOOLEAN,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {zit6_interfaces__msg__ZitPid__FIELD_NAME__kp, 2, 2},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {zit6_interfaces__msg__ZitPid__FIELD_NAME__ki, 2, 2},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {zit6_interfaces__msg__ZitPid__FIELD_NAME__kd, 2, 2},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {zit6_interfaces__msg__ZitPid__FIELD_NAME__i_limit, 7, 7},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {zit6_interfaces__msg__ZitPid__FIELD_NAME__out_limit, 9, 9},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {zit6_interfaces__msg__ZitPid__FIELD_NAME__max_v, 5, 5},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {zit6_interfaces__msg__ZitPid__FIELD_NAME__max_a, 5, 5},
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
zit6_interfaces__msg__ZitPid__get_type_description(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {zit6_interfaces__msg__ZitPid__TYPE_NAME, 26, 26},
      {zit6_interfaces__msg__ZitPid__FIELDS, 9, 9},
    },
    {NULL, 0, 0},
  };
  if (!constructed) {
    constructed = true;
  }
  return &description;
}

static char toplevel_type_raw_source[] =
  "uint8 axis # 0:X, 1:Y, 2:Z, 3:Yaw\n"
  "bool is_pos_ring # true: \\xe4\\xbd\\x8d\\xe7\\xbd\\xae\\xe7\\x8e\\xaf(\\xe5\\x90\\xab\\xe8\\xa7\\x84\\xe5\\x88\\x92\\xe5\\x99\\xa8), false: \\xe9\\x80\\x9f\\xe5\\xba\\xa6\\xe7\\x8e\\xaf\n"
  "float32 kp\n"
  "float32 ki\n"
  "float32 kd\n"
  "float32 i_limit\n"
  "float32 out_limit\n"
  "\n"
  "# \\xe8\\xa7\\x84\\xe5\\x88\\x92\\xe5\\x99\\xa8\\xe5\\x8f\\x82\\xe6\\x95\\xb0 (\\xe4\\xbb\\x85\\xe5\\x9c\\xa8 is_pos_ring \\xe4\\xb8\\xba true \\xe6\\x97\\xb6\\xe6\\x9c\\x89\\xe6\\x95\\x88)\n"
  "float32 max_v\n"
  "float32 max_a";

static char msg_encoding[] = "msg";

// Define all individual source functions

const rosidl_runtime_c__type_description__TypeSource *
zit6_interfaces__msg__ZitPid__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {zit6_interfaces__msg__ZitPid__TYPE_NAME, 26, 26},
    {msg_encoding, 3, 3},
    {toplevel_type_raw_source, 213, 213},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
zit6_interfaces__msg__ZitPid__get_type_description_sources(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[1];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 1, 1};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *zit6_interfaces__msg__ZitPid__get_individual_type_description_source(NULL),
    constructed = true;
  }
  return &source_sequence;
}
