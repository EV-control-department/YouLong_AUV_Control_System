// generated from rosidl_generator_c/resource/idl__description.c.em
// with input from uv_msgs:msg/Detection.idl
// generated code does not contain a copyright notice

#include "uv_msgs/msg/detail/detection__functions.h"

ROSIDL_GENERATOR_C_PUBLIC_uv_msgs
const rosidl_type_hash_t *
uv_msgs__msg__Detection__get_type_hash(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0x8d, 0x72, 0xbd, 0xb2, 0x1b, 0xe7, 0x30, 0xbe,
      0x1e, 0x75, 0xdc, 0x41, 0x8c, 0x35, 0x48, 0x4b,
      0xb5, 0xd0, 0xaf, 0xca, 0x94, 0x9a, 0x8e, 0x3f,
      0x3c, 0x8b, 0xd4, 0x2b, 0xa1, 0x72, 0x47, 0x3d,
    }};
  return &hash;
}

#include <assert.h>
#include <string.h>

// Include directives for referenced types

// Hashes for external referenced types
#ifndef NDEBUG
#endif

static char uv_msgs__msg__Detection__TYPE_NAME[] = "uv_msgs/msg/Detection";

// Define type names, field names, and default values
static char uv_msgs__msg__Detection__FIELD_NAME__class_id[] = "class_id";
static char uv_msgs__msg__Detection__FIELD_NAME__confidence[] = "confidence";
static char uv_msgs__msg__Detection__FIELD_NAME__pixel_x[] = "pixel_x";
static char uv_msgs__msg__Detection__FIELD_NAME__pixel_y[] = "pixel_y";
static char uv_msgs__msg__Detection__FIELD_NAME__bbox_x1[] = "bbox_x1";
static char uv_msgs__msg__Detection__FIELD_NAME__bbox_y1[] = "bbox_y1";
static char uv_msgs__msg__Detection__FIELD_NAME__bbox_x2[] = "bbox_x2";
static char uv_msgs__msg__Detection__FIELD_NAME__bbox_y2[] = "bbox_y2";

static rosidl_runtime_c__type_description__Field uv_msgs__msg__Detection__FIELDS[] = {
  {
    {uv_msgs__msg__Detection__FIELD_NAME__class_id, 8, 8},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT16,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {uv_msgs__msg__Detection__FIELD_NAME__confidence, 10, 10},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {uv_msgs__msg__Detection__FIELD_NAME__pixel_x, 7, 7},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {uv_msgs__msg__Detection__FIELD_NAME__pixel_y, 7, 7},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {uv_msgs__msg__Detection__FIELD_NAME__bbox_x1, 7, 7},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {uv_msgs__msg__Detection__FIELD_NAME__bbox_y1, 7, 7},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {uv_msgs__msg__Detection__FIELD_NAME__bbox_x2, 7, 7},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {uv_msgs__msg__Detection__FIELD_NAME__bbox_y2, 7, 7},
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
uv_msgs__msg__Detection__get_type_description(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {uv_msgs__msg__Detection__TYPE_NAME, 21, 21},
      {uv_msgs__msg__Detection__FIELDS, 8, 8},
    },
    {NULL, 0, 0},
  };
  if (!constructed) {
    constructed = true;
  }
  return &description;
}

static char toplevel_type_raw_source[] =
  "# Single object detection result\n"
  "\n"
  "uint16 class_id\n"
  "float32 confidence\n"
  "float32 pixel_x       # Center pixel X\n"
  "float32 pixel_y       # Center pixel Y\n"
  "float32 bbox_x1       # Bounding box top-left X\n"
  "float32 bbox_y1       # Bounding box top-left Y\n"
  "float32 bbox_x2       # Bounding box bottom-right X\n"
  "float32 bbox_y2       # Bounding box bottom-right Y";

static char msg_encoding[] = "msg";

// Define all individual source functions

const rosidl_runtime_c__type_description__TypeSource *
uv_msgs__msg__Detection__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {uv_msgs__msg__Detection__TYPE_NAME, 21, 21},
    {msg_encoding, 3, 3},
    {toplevel_type_raw_source, 347, 347},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
uv_msgs__msg__Detection__get_type_description_sources(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[1];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 1, 1};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *uv_msgs__msg__Detection__get_individual_type_description_source(NULL),
    constructed = true;
  }
  return &source_sequence;
}
