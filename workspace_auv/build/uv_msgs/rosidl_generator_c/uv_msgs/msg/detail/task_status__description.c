// generated from rosidl_generator_c/resource/idl__description.c.em
// with input from uv_msgs:msg/TaskStatus.idl
// generated code does not contain a copyright notice

#include "uv_msgs/msg/detail/task_status__functions.h"

ROSIDL_GENERATOR_C_PUBLIC_uv_msgs
const rosidl_type_hash_t *
uv_msgs__msg__TaskStatus__get_type_hash(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0x2c, 0x62, 0x56, 0x35, 0x56, 0x4f, 0x44, 0x40,
      0x1b, 0xb6, 0x61, 0x97, 0xb8, 0xd5, 0x5c, 0x2f,
      0xe9, 0x79, 0xb1, 0x86, 0x28, 0xfc, 0xec, 0x49,
      0xab, 0x30, 0xaf, 0x56, 0x56, 0x02, 0xc1, 0xf3,
    }};
  return &hash;
}

#include <assert.h>
#include <string.h>

// Include directives for referenced types

// Hashes for external referenced types
#ifndef NDEBUG
#endif

static char uv_msgs__msg__TaskStatus__TYPE_NAME[] = "uv_msgs/msg/TaskStatus";

// Define type names, field names, and default values
static char uv_msgs__msg__TaskStatus__FIELD_NAME__status[] = "status";
static char uv_msgs__msg__TaskStatus__FIELD_NAME__current_task_index[] = "current_task_index";
static char uv_msgs__msg__TaskStatus__FIELD_NAME__total_tasks[] = "total_tasks";
static char uv_msgs__msg__TaskStatus__FIELD_NAME__current_task_name[] = "current_task_name";
static char uv_msgs__msg__TaskStatus__FIELD_NAME__error_message[] = "error_message";

static rosidl_runtime_c__type_description__Field uv_msgs__msg__TaskStatus__FIELDS[] = {
  {
    {uv_msgs__msg__TaskStatus__FIELD_NAME__status, 6, 6},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT8,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {uv_msgs__msg__TaskStatus__FIELD_NAME__current_task_index, 18, 18},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT32,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {uv_msgs__msg__TaskStatus__FIELD_NAME__total_tasks, 11, 11},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT32,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {uv_msgs__msg__TaskStatus__FIELD_NAME__current_task_name, 17, 17},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_STRING,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {uv_msgs__msg__TaskStatus__FIELD_NAME__error_message, 13, 13},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_STRING,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
};

const rosidl_runtime_c__type_description__TypeDescription *
uv_msgs__msg__TaskStatus__get_type_description(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {uv_msgs__msg__TaskStatus__TYPE_NAME, 22, 22},
      {uv_msgs__msg__TaskStatus__FIELDS, 5, 5},
    },
    {NULL, 0, 0},
  };
  if (!constructed) {
    constructed = true;
  }
  return &description;
}

static char toplevel_type_raw_source[] =
  "# Task execution status\n"
  "\n"
  "uint8 STATUS_IDLE     = 0\n"
  "uint8 STATUS_RUNNING  = 1\n"
  "uint8 STATUS_PAUSED   = 2\n"
  "uint8 STATUS_DONE     = 3\n"
  "uint8 STATUS_ERROR    = 4\n"
  "\n"
  "uint8 status\n"
  "uint32 current_task_index\n"
  "uint32 total_tasks\n"
  "string current_task_name\n"
  "string error_message";

static char msg_encoding[] = "msg";

// Define all individual source functions

const rosidl_runtime_c__type_description__TypeSource *
uv_msgs__msg__TaskStatus__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {uv_msgs__msg__TaskStatus__TYPE_NAME, 22, 22},
    {msg_encoding, 3, 3},
    {toplevel_type_raw_source, 260, 260},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
uv_msgs__msg__TaskStatus__get_type_description_sources(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[1];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 1, 1};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *uv_msgs__msg__TaskStatus__get_individual_type_description_source(NULL),
    constructed = true;
  }
  return &source_sequence;
}
