// generated from rosidl_generator_c/resource/idl__description.c.em
// with input from zit6_interfaces:msg/ZitStatus.idl
// generated code does not contain a copyright notice

#include "zit6_interfaces/msg/detail/zit_status__functions.h"

ROSIDL_GENERATOR_C_PUBLIC_zit6_interfaces
const rosidl_type_hash_t *
zit6_interfaces__msg__ZitStatus__get_type_hash(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0x6e, 0x51, 0x2a, 0xac, 0xf7, 0xb8, 0x01, 0xe3,
      0x61, 0x33, 0x68, 0xbb, 0x7c, 0xe6, 0xf4, 0xd3,
      0xcd, 0x6d, 0x05, 0x7d, 0x12, 0x8f, 0x42, 0xc5,
      0x40, 0x20, 0xd1, 0xb2, 0x81, 0x7e, 0x68, 0x16,
    }};
  return &hash;
}

#include <assert.h>
#include <string.h>

// Include directives for referenced types

// Hashes for external referenced types
#ifndef NDEBUG
#endif

static char zit6_interfaces__msg__ZitStatus__TYPE_NAME[] = "zit6_interfaces/msg/ZitStatus";

// Define type names, field names, and default values
static char zit6_interfaces__msg__ZitStatus__FIELD_NAME__is_armed[] = "is_armed";
static char zit6_interfaces__msg__ZitStatus__FIELD_NAME__arm_mode[] = "arm_mode";
static char zit6_interfaces__msg__ZitStatus__FIELD_NAME__control_level[] = "control_level";
static char zit6_interfaces__msg__ZitStatus__FIELD_NAME__ins_state[] = "ins_state";
static char zit6_interfaces__msg__ZitStatus__FIELD_NAME__navigation_ready[] = "navigation_ready";
static char zit6_interfaces__msg__ZitStatus__FIELD_NAME__forces[] = "forces";
static char zit6_interfaces__msg__ZitStatus__FIELD_NAME__cycle_time_ms[] = "cycle_time_ms";
static char zit6_interfaces__msg__ZitStatus__FIELD_NAME__battery_voltage[] = "battery_voltage";
static char zit6_interfaces__msg__ZitStatus__FIELD_NAME__error_flags[] = "error_flags";

static rosidl_runtime_c__type_description__Field zit6_interfaces__msg__ZitStatus__FIELDS[] = {
  {
    {zit6_interfaces__msg__ZitStatus__FIELD_NAME__is_armed, 8, 8},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_BOOLEAN,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {zit6_interfaces__msg__ZitStatus__FIELD_NAME__arm_mode, 8, 8},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT8,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {zit6_interfaces__msg__ZitStatus__FIELD_NAME__control_level, 13, 13},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT8,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {zit6_interfaces__msg__ZitStatus__FIELD_NAME__ins_state, 9, 9},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT8,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {zit6_interfaces__msg__ZitStatus__FIELD_NAME__navigation_ready, 16, 16},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_BOOLEAN,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {zit6_interfaces__msg__ZitStatus__FIELD_NAME__forces, 6, 6},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT_ARRAY,
      4,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {zit6_interfaces__msg__ZitStatus__FIELD_NAME__cycle_time_ms, 13, 13},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {zit6_interfaces__msg__ZitStatus__FIELD_NAME__battery_voltage, 15, 15},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {zit6_interfaces__msg__ZitStatus__FIELD_NAME__error_flags, 11, 11},
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
zit6_interfaces__msg__ZitStatus__get_type_description(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {zit6_interfaces__msg__ZitStatus__TYPE_NAME, 29, 29},
      {zit6_interfaces__msg__ZitStatus__FIELDS, 9, 9},
    },
    {NULL, 0, 0},
  };
  if (!constructed) {
    constructed = true;
  }
  return &description;
}

static char toplevel_type_raw_source[] =
  "# ZIT6 AUV \\xe6\\xa0\\xb8\\xe5\\xbf\\x83\\xe7\\x8a\\xb6\\xe6\\x80\\x81\\xe6\\xb1\\x87\\xe6\\x80\\xbb (v5.0)\n"
  "# \\xe5\\xaf\\xb9\\xe5\\xba\\x94\\xe8\\xaf\\x9d\\xe9\\xa2\\x98: /zit6/state/status\n"
  "\n"
  "# \\xe6\\x8e\\xa7\\xe5\\x88\\xb6\\xe5\\xb1\\x82\\xe7\\xba\\xa7\\xe5\\xb8\\xb8\\xe9\\x87\\x8f (control_level)\n"
  "uint8 LEVEL_NONE = 0      # \\xe6\\x97\\xa0\\xe4\\xba\\xba\\xe6\\x8e\\xa7/\\xe5\\xae\\x89\\xe5\\x85\\xa8\\xe5\\x81\\x9c\n"
  "uint8 LEVEL_POS = 1       # \\xe4\\xbd\\x8d\\xe7\\xbd\\xae\\xe7\\x8e\\xaf\n"
  "uint8 LEVEL_VEL = 2       # \\xe9\\x80\\x9f\\xe5\\xba\\xa6\\xe7\\x8e\\xaf\n"
  "uint8 LEVEL_FORCE = 3     # \\xe6\\x8e\\xa8\\xe5\\x8a\\x9b\\xe7\\x8e\\xaf (\\xe7\\x9b\\xb4\\xe6\\x8e\\xa5\\xe6\\x8e\\xa7\\xe5\\x88\\xb6)\n"
  "\n"
  "# \\xe9\\x94\\x99\\xe8\\xaf\\xaf\\xe6\\xa0\\x87\\xe5\\xbf\\x97\\xe4\\xbd\\x8d\\xe5\\xb8\\xb8\\xe9\\x87\\x8f (error_flags)\n"
  "uint32 ERROR_FORCE_STOP = 1   # Bit0: \\xe5\\xbc\\xba\\xe5\\x88\\xb6\\xe5\\x81\\x9c\\xe6\\x9c\\xba\\xef\\xbc\\x88\\xe8\\x87\\xb4\\xe5\\x91\\xbd\\xef\\xbc\\x89\n"
  "uint32 ERROR_SENSOR_FAIL = 2  # Bit1: \\xe4\\xbc\\xa0\\xe6\\x84\\x9f\\xe5\\x99\\xa8\\xe6\\x95\\x85\\xe9\\x9a\\x9c\\xef\\xbc\\x88IMU/DVL\\xef\\xbc\\x89\n"
  "uint32 ERROR_VOLTAGE_LOW = 4  # Bit2: \\xe7\\x94\\xb5\\xe5\\x8e\\x8b\\xe5\\xbc\\x82\\xe5\\xb8\\xb8\n"
  "uint32 ERROR_COMM_TIMEOUT = 8 # Bit3: \\xe9\\x80\\x9a\\xe8\\xae\\xaf\\xe8\\xb6\\x85\\xe6\\x97\\xb6\n"
  "\n"
  "bool is_armed              # \\xe6\\x98\\xaf\\xe5\\x90\\xa6\\xe5\\xb7\\xb2\\xe8\\xa7\\xa3\\xe9\\x94\\x81\\xef\\xbc\\x88\\xe5\\x85\\x81\\xe8\\xae\\xb8\\xe6\\x89\\xa7\\xe8\\xa1\\x8c\\xe6\\x8e\\xa8\\xe5\\x8a\\x9b\\xe8\\xbe\\x93\\xe5\\x87\\xba\\xef\\xbc\\x89\n"
  "uint8 arm_mode             # \\xe8\\xa7\\xa3\\xe9\\x94\\x81\\xe6\\xa8\\xa1\\xe5\\xbc\\x8f (0:\\xe9\\xbb\\x98\\xe8\\xae\\xa4, 3:\\xe9\\x81\\xa5\\xe6\\x8e\\xa7\\xe6\\xa8\\xa1\\xe5\\xbc\\x8f/\\xe7\\xbb\\x95\\xe8\\xbf\\x87\\xe5\\xaf\\xbc\\xe8\\x88\\xaa\\xe6\\xa3\\x80\\xe6\\x9f\\xa5)\n"
  "uint8 control_level        # \\xe5\\xbd\\x93\\xe5\\x89\\x8d\\xe6\\x8e\\xa7\\xe5\\x88\\xb6\\xe5\\xb1\\x82\\xe7\\xba\\xa7 (\\xe8\\xa7\\x81 LEVEL_* \\xe5\\xb8\\xb8\\xe9\\x87\\x8f)\n"
  "uint8 ins_state            # \\xe6\\x83\\xaf\\xe5\\xaf\\xbc\\xe5\\x86\\x85\\xe9\\x83\\xa8\\xe7\\x8a\\xb6\\xe6\\x80\\x81 (0:\\xe5\\xbe\\x85\\xe6\\x9c\\xba, 1:\\xe7\\xb2\\x97\\xe5\\xaf\\xb9\\xe5\\x87\\x86, 2:\\xe7\\xb2\\xbe\\xe5\\xaf\\xb9\\xe5\\x87\\x86, 3:SINS/GPS/DVL, 4:SINS/DVL, 5:MRU)\n"
  "bool navigation_ready      # \\xe5\\xaf\\xbc\\xe8\\x88\\xaa\\xe5\\x87\\x86\\xe5\\xa4\\x87\\xe5\\xb0\\xb1\\xe7\\xbb\\xaa (\\xe6\\x83\\xaf\\xe5\\xaf\\xbc\\xe3\\x80\\x81DVL\\xe7\\xad\\x89\\xe4\\xbc\\xa0\\xe6\\x84\\x9f\\xe5\\x99\\xa8\\xe5\\x81\\xa5\\xe5\\xba\\xb7\\xe5\\xb9\\xb6\\xe5\\xaf\\xb9\\xe5\\x87\\x86)\n"
  "float32[4] forces          # 4-DOF \\xe7\\x9b\\xae\\xe6\\xa0\\x87\\xe6\\x8e\\xa8\\xe5\\x8a\\x9b/\\xe5\\x8a\\x9b\\xe7\\x9f\\xa9 [Fx, Fy, Fz, Mz] (\\xe5\\xbd\\x92\\xe4\\xb8\\x80\\xe5\\x8c\\x96\\xe6\\x88\\x96\\xe5\\x8d\\x95\\xe4\\xbd\\x8dN/Nm)\n"
  "float32 cycle_time_ms      # \\xe6\\x8e\\xa7\\xe5\\x88\\xb6\\xe4\\xb8\\xbb\\xe5\\xbe\\xaa\\xe7\\x8e\\xaf\\xe8\\x80\\x97\\xe6\\x97\\xb6 (\\xe5\\x8d\\x95\\xe4\\xbd\\x8d: ms)\n"
  "float32 battery_voltage    # \\xe7\\x94\\xb5\\xe6\\xb1\\xa0\\xe7\\x94\\xb5\\xe5\\x8e\\x8b (\\xe5\\x8d\\x95\\xe4\\xbd\\x8d: V)\n"
  "uint32 error_flags         # \\xe4\\xbd\\x8d\\xe5\\x9f\\x9f\\xe9\\x94\\x99\\xe8\\xaf\\xaf/\\xe8\\xad\\xa6\\xe5\\x91\\x8a\\xe6\\xa0\\x87\\xe8\\xaf\\x86\\xe4\\xbd\\x8d (\\xe8\\xa7\\x81 ERROR_* \\xe5\\xb8\\xb8\\xe9\\x87\\x8f)";

static char msg_encoding[] = "msg";

// Define all individual source functions

const rosidl_runtime_c__type_description__TypeSource *
zit6_interfaces__msg__ZitStatus__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {zit6_interfaces__msg__ZitStatus__TYPE_NAME, 29, 29},
    {msg_encoding, 3, 3},
    {toplevel_type_raw_source, 943, 943},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
zit6_interfaces__msg__ZitStatus__get_type_description_sources(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[1];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 1, 1};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *zit6_interfaces__msg__ZitStatus__get_individual_type_description_source(NULL),
    constructed = true;
  }
  return &source_sequence;
}
