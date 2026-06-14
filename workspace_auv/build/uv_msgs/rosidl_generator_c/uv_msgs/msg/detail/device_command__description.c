// generated from rosidl_generator_c/resource/idl__description.c.em
// with input from uv_msgs:msg/DeviceCommand.idl
// generated code does not contain a copyright notice

#include "uv_msgs/msg/detail/device_command__functions.h"

ROSIDL_GENERATOR_C_PUBLIC_uv_msgs
const rosidl_type_hash_t *
uv_msgs__msg__DeviceCommand__get_type_hash(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0x2f, 0x22, 0x65, 0x51, 0x00, 0x55, 0x3a, 0x1a,
      0x0f, 0xdf, 0xd4, 0xb9, 0x4e, 0xc8, 0xc8, 0xa1,
      0xdc, 0x04, 0x2d, 0x14, 0x5f, 0x89, 0x98, 0xd4,
      0xc0, 0x23, 0x2a, 0x1c, 0x7f, 0x8a, 0xd4, 0xd4,
    }};
  return &hash;
}

#include <assert.h>
#include <string.h>

// Include directives for referenced types

// Hashes for external referenced types
#ifndef NDEBUG
#endif

static char uv_msgs__msg__DeviceCommand__TYPE_NAME[] = "uv_msgs/msg/DeviceCommand";

// Define type names, field names, and default values
static char uv_msgs__msg__DeviceCommand__FIELD_NAME__device_type[] = "device_type";
static char uv_msgs__msg__DeviceCommand__FIELD_NAME__command[] = "command";
static char uv_msgs__msg__DeviceCommand__FIELD_NAME__value[] = "value";

static rosidl_runtime_c__type_description__Field uv_msgs__msg__DeviceCommand__FIELDS[] = {
  {
    {uv_msgs__msg__DeviceCommand__FIELD_NAME__device_type, 11, 11},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT8,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {uv_msgs__msg__DeviceCommand__FIELD_NAME__command, 7, 7},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT8,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {uv_msgs__msg__DeviceCommand__FIELD_NAME__value, 5, 5},
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
uv_msgs__msg__DeviceCommand__get_type_description(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {uv_msgs__msg__DeviceCommand__TYPE_NAME, 25, 25},
      {uv_msgs__msg__DeviceCommand__FIELDS, 3, 3},
    },
    {NULL, 0, 0},
  };
  if (!constructed) {
    constructed = true;
  }
  return &description;
}

static char toplevel_type_raw_source[] =
  "# Device command for LED, arm, servo, etc.\n"
  "\n"
  "uint8 DEVICE_LED    = 0\n"
  "uint8 DEVICE_ARM    = 1\n"
  "uint8 DEVICE_SERVO  = 2\n"
  "\n"
  "uint8 device_type\n"
  "uint8 command       # Device-specific command code\n"
  "float32 value       # Device-specific value";

static char msg_encoding[] = "msg";

// Define all individual source functions

const rosidl_runtime_c__type_description__TypeSource *
uv_msgs__msg__DeviceCommand__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {uv_msgs__msg__DeviceCommand__TYPE_NAME, 25, 25},
    {msg_encoding, 3, 3},
    {toplevel_type_raw_source, 230, 230},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
uv_msgs__msg__DeviceCommand__get_type_description_sources(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[1];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 1, 1};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *uv_msgs__msg__DeviceCommand__get_individual_type_description_source(NULL),
    constructed = true;
  }
  return &source_sequence;
}
