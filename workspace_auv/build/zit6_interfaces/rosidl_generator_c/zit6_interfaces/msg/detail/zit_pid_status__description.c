// generated from rosidl_generator_c/resource/idl__description.c.em
// with input from zit6_interfaces:msg/ZitPidStatus.idl
// generated code does not contain a copyright notice

#include "zit6_interfaces/msg/detail/zit_pid_status__functions.h"

ROSIDL_GENERATOR_C_PUBLIC_zit6_interfaces
const rosidl_type_hash_t *
zit6_interfaces__msg__ZitPidStatus__get_type_hash(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0x87, 0xe7, 0x53, 0x2c, 0x63, 0x0a, 0x03, 0x83,
      0x84, 0xcf, 0xc5, 0xb2, 0xd8, 0x8c, 0x6f, 0x48,
      0x52, 0x82, 0x20, 0xf4, 0xb7, 0xb8, 0xc1, 0xa9,
      0xc7, 0x57, 0xc7, 0xa4, 0x9b, 0x08, 0xd1, 0x6a,
    }};
  return &hash;
}

#include <assert.h>
#include <string.h>

// Include directives for referenced types

// Hashes for external referenced types
#ifndef NDEBUG
#endif

static char zit6_interfaces__msg__ZitPidStatus__TYPE_NAME[] = "zit6_interfaces/msg/ZitPidStatus";

// Define type names, field names, and default values
static char zit6_interfaces__msg__ZitPidStatus__FIELD_NAME__pos_kp[] = "pos_kp";
static char zit6_interfaces__msg__ZitPidStatus__FIELD_NAME__pos_max_v[] = "pos_max_v";
static char zit6_interfaces__msg__ZitPidStatus__FIELD_NAME__pos_max_a[] = "pos_max_a";
static char zit6_interfaces__msg__ZitPidStatus__FIELD_NAME__pos_out_limit[] = "pos_out_limit";
static char zit6_interfaces__msg__ZitPidStatus__FIELD_NAME__vel_kp[] = "vel_kp";
static char zit6_interfaces__msg__ZitPidStatus__FIELD_NAME__vel_ki[] = "vel_ki";
static char zit6_interfaces__msg__ZitPidStatus__FIELD_NAME__vel_kd[] = "vel_kd";
static char zit6_interfaces__msg__ZitPidStatus__FIELD_NAME__vel_i_limit[] = "vel_i_limit";
static char zit6_interfaces__msg__ZitPidStatus__FIELD_NAME__vel_out_limit[] = "vel_out_limit";

static rosidl_runtime_c__type_description__Field zit6_interfaces__msg__ZitPidStatus__FIELDS[] = {
  {
    {zit6_interfaces__msg__ZitPidStatus__FIELD_NAME__pos_kp, 6, 6},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT_ARRAY,
      4,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {zit6_interfaces__msg__ZitPidStatus__FIELD_NAME__pos_max_v, 9, 9},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT_ARRAY,
      4,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {zit6_interfaces__msg__ZitPidStatus__FIELD_NAME__pos_max_a, 9, 9},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT_ARRAY,
      4,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {zit6_interfaces__msg__ZitPidStatus__FIELD_NAME__pos_out_limit, 13, 13},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT_ARRAY,
      4,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {zit6_interfaces__msg__ZitPidStatus__FIELD_NAME__vel_kp, 6, 6},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT_ARRAY,
      4,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {zit6_interfaces__msg__ZitPidStatus__FIELD_NAME__vel_ki, 6, 6},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT_ARRAY,
      4,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {zit6_interfaces__msg__ZitPidStatus__FIELD_NAME__vel_kd, 6, 6},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT_ARRAY,
      4,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {zit6_interfaces__msg__ZitPidStatus__FIELD_NAME__vel_i_limit, 11, 11},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT_ARRAY,
      4,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {zit6_interfaces__msg__ZitPidStatus__FIELD_NAME__vel_out_limit, 13, 13},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT_ARRAY,
      4,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
};

const rosidl_runtime_c__type_description__TypeDescription *
zit6_interfaces__msg__ZitPidStatus__get_type_description(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {zit6_interfaces__msg__ZitPidStatus__TYPE_NAME, 32, 32},
      {zit6_interfaces__msg__ZitPidStatus__FIELDS, 9, 9},
    },
    {NULL, 0, 0},
  };
  if (!constructed) {
    constructed = true;
  }
  return &description;
}

static char toplevel_type_raw_source[] =
  "# \\xe5\\x85\\xa8\\xe8\\xbd\\xb4 PID \\xe4\\xb8\\x8e \\xe8\\xa7\\x84\\xe5\\x88\\x92\\xe5\\x99\\xa8\\xe7\\x8a\\xb6\\xe6\\x80\\x81\\xe6\\xb1\\x87\\xe6\\x80\\xbb\\xe5\\x8f\\x8d\\xe9\\xa6\\x88 (1Hz)\n"
  "\n"
  "# \\xe4\\xbd\\x8d\\xe7\\xbd\\xae\\xe7\\x8e\\xaf P \\xe5\\x8f\\x82\\xe6\\x95\\xb0\\xe4\\xb8\\x8e\\xe8\\xa7\\x84\\xe5\\x88\\x92\\xe5\\x99\\xa8\\xe9\\x99\\x90\\xe5\\x80\\xbc [X, Y, Z, Yaw]\n"
  "float32[4] pos_kp\n"
  "float32[4] pos_max_v\n"
  "float32[4] pos_max_a\n"
  "float32[4] pos_out_limit\n"
  "\n"
  "# \\xe9\\x80\\x9f\\xe5\\xba\\xa6\\xe7\\x8e\\xaf PID \\xe5\\x8f\\x82\\xe6\\x95\\xb0 [X, Y, Z, Yaw]\n"
  "float32[4] vel_kp\n"
  "float32[4] vel_ki\n"
  "float32[4] vel_kd\n"
  "float32[4] vel_i_limit\n"
  "float32[4] vel_out_limit";

static char msg_encoding[] = "msg";

// Define all individual source functions

const rosidl_runtime_c__type_description__TypeSource *
zit6_interfaces__msg__ZitPidStatus__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {zit6_interfaces__msg__ZitPidStatus__TYPE_NAME, 32, 32},
    {msg_encoding, 3, 3},
    {toplevel_type_raw_source, 276, 276},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
zit6_interfaces__msg__ZitPidStatus__get_type_description_sources(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[1];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 1, 1};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *zit6_interfaces__msg__ZitPidStatus__get_individual_type_description_source(NULL),
    constructed = true;
  }
  return &source_sequence;
}
