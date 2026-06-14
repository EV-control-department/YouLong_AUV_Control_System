// generated from rosidl_generator_c/resource/idl__description.c.em
// with input from zit6_interfaces:srv/UpdateParams.idl
// generated code does not contain a copyright notice

#include "zit6_interfaces/srv/detail/update_params__functions.h"

ROSIDL_GENERATOR_C_PUBLIC_zit6_interfaces
const rosidl_type_hash_t *
zit6_interfaces__srv__UpdateParams__get_type_hash(
  const rosidl_service_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0x4c, 0xea, 0x69, 0x88, 0xd0, 0xbf, 0x3a, 0x6f,
      0x6f, 0x6f, 0x50, 0x40, 0x32, 0xb2, 0x99, 0x66,
      0xab, 0x25, 0xbf, 0x1f, 0x1f, 0x9f, 0x7c, 0xc5,
      0xbe, 0xc5, 0xe8, 0x8d, 0x7c, 0x19, 0x72, 0x52,
    }};
  return &hash;
}

ROSIDL_GENERATOR_C_PUBLIC_zit6_interfaces
const rosidl_type_hash_t *
zit6_interfaces__srv__UpdateParams_Request__get_type_hash(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0xd7, 0x52, 0xbd, 0x40, 0x58, 0x51, 0x82, 0x50,
      0x1b, 0x5d, 0x6d, 0x1a, 0x1d, 0x5b, 0x6b, 0xdb,
      0x8c, 0x57, 0x9f, 0x1b, 0xe7, 0xf5, 0xbe, 0x50,
      0xb6, 0x72, 0xe2, 0x87, 0xbb, 0xf5, 0x1c, 0x29,
    }};
  return &hash;
}

ROSIDL_GENERATOR_C_PUBLIC_zit6_interfaces
const rosidl_type_hash_t *
zit6_interfaces__srv__UpdateParams_Response__get_type_hash(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0x35, 0x19, 0x68, 0xd7, 0x3e, 0x56, 0x44, 0x8e,
      0xd4, 0xd3, 0xd5, 0x92, 0xce, 0x35, 0x21, 0x01,
      0xb8, 0x2f, 0x20, 0xf3, 0xcf, 0xfd, 0x33, 0xbb,
      0x67, 0x7b, 0x2f, 0x2c, 0x48, 0x66, 0x8f, 0x36,
    }};
  return &hash;
}

ROSIDL_GENERATOR_C_PUBLIC_zit6_interfaces
const rosidl_type_hash_t *
zit6_interfaces__srv__UpdateParams_Event__get_type_hash(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0x59, 0x3b, 0x43, 0xe2, 0xbe, 0xea, 0xf0, 0xc6,
      0x13, 0xc0, 0xd1, 0x36, 0xba, 0x6a, 0x58, 0xa6,
      0xd6, 0x43, 0x01, 0xea, 0x3e, 0x7e, 0xa1, 0x84,
      0x0d, 0xa7, 0x0d, 0x65, 0xf5, 0xfd, 0xee, 0x9a,
    }};
  return &hash;
}

#include <assert.h>
#include <string.h>

// Include directives for referenced types
#include "service_msgs/msg/detail/service_event_info__functions.h"
#include "builtin_interfaces/msg/detail/time__functions.h"

// Hashes for external referenced types
#ifndef NDEBUG
static const rosidl_type_hash_t builtin_interfaces__msg__Time__EXPECTED_HASH = {1, {
    0xb1, 0x06, 0x23, 0x5e, 0x25, 0xa4, 0xc5, 0xed,
    0x35, 0x09, 0x8a, 0xa0, 0xa6, 0x1a, 0x3e, 0xe9,
    0xc9, 0xb1, 0x8d, 0x19, 0x7f, 0x39, 0x8b, 0x0e,
    0x42, 0x06, 0xce, 0xa9, 0xac, 0xf9, 0xc1, 0x97,
  }};
static const rosidl_type_hash_t service_msgs__msg__ServiceEventInfo__EXPECTED_HASH = {1, {
    0x41, 0xbc, 0xbb, 0xe0, 0x7a, 0x75, 0xc9, 0xb5,
    0x2b, 0xc9, 0x6b, 0xfd, 0x5c, 0x24, 0xd7, 0xf0,
    0xfc, 0x0a, 0x08, 0xc0, 0xcb, 0x79, 0x21, 0xb3,
    0x37, 0x3c, 0x57, 0x32, 0x34, 0x5a, 0x6f, 0x45,
  }};
#endif

static char zit6_interfaces__srv__UpdateParams__TYPE_NAME[] = "zit6_interfaces/srv/UpdateParams";
static char builtin_interfaces__msg__Time__TYPE_NAME[] = "builtin_interfaces/msg/Time";
static char service_msgs__msg__ServiceEventInfo__TYPE_NAME[] = "service_msgs/msg/ServiceEventInfo";
static char zit6_interfaces__srv__UpdateParams_Event__TYPE_NAME[] = "zit6_interfaces/srv/UpdateParams_Event";
static char zit6_interfaces__srv__UpdateParams_Request__TYPE_NAME[] = "zit6_interfaces/srv/UpdateParams_Request";
static char zit6_interfaces__srv__UpdateParams_Response__TYPE_NAME[] = "zit6_interfaces/srv/UpdateParams_Response";

// Define type names, field names, and default values
static char zit6_interfaces__srv__UpdateParams__FIELD_NAME__request_message[] = "request_message";
static char zit6_interfaces__srv__UpdateParams__FIELD_NAME__response_message[] = "response_message";
static char zit6_interfaces__srv__UpdateParams__FIELD_NAME__event_message[] = "event_message";

static rosidl_runtime_c__type_description__Field zit6_interfaces__srv__UpdateParams__FIELDS[] = {
  {
    {zit6_interfaces__srv__UpdateParams__FIELD_NAME__request_message, 15, 15},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE,
      0,
      0,
      {zit6_interfaces__srv__UpdateParams_Request__TYPE_NAME, 40, 40},
    },
    {NULL, 0, 0},
  },
  {
    {zit6_interfaces__srv__UpdateParams__FIELD_NAME__response_message, 16, 16},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE,
      0,
      0,
      {zit6_interfaces__srv__UpdateParams_Response__TYPE_NAME, 41, 41},
    },
    {NULL, 0, 0},
  },
  {
    {zit6_interfaces__srv__UpdateParams__FIELD_NAME__event_message, 13, 13},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE,
      0,
      0,
      {zit6_interfaces__srv__UpdateParams_Event__TYPE_NAME, 38, 38},
    },
    {NULL, 0, 0},
  },
};

static rosidl_runtime_c__type_description__IndividualTypeDescription zit6_interfaces__srv__UpdateParams__REFERENCED_TYPE_DESCRIPTIONS[] = {
  {
    {builtin_interfaces__msg__Time__TYPE_NAME, 27, 27},
    {NULL, 0, 0},
  },
  {
    {service_msgs__msg__ServiceEventInfo__TYPE_NAME, 33, 33},
    {NULL, 0, 0},
  },
  {
    {zit6_interfaces__srv__UpdateParams_Event__TYPE_NAME, 38, 38},
    {NULL, 0, 0},
  },
  {
    {zit6_interfaces__srv__UpdateParams_Request__TYPE_NAME, 40, 40},
    {NULL, 0, 0},
  },
  {
    {zit6_interfaces__srv__UpdateParams_Response__TYPE_NAME, 41, 41},
    {NULL, 0, 0},
  },
};

const rosidl_runtime_c__type_description__TypeDescription *
zit6_interfaces__srv__UpdateParams__get_type_description(
  const rosidl_service_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {zit6_interfaces__srv__UpdateParams__TYPE_NAME, 32, 32},
      {zit6_interfaces__srv__UpdateParams__FIELDS, 3, 3},
    },
    {zit6_interfaces__srv__UpdateParams__REFERENCED_TYPE_DESCRIPTIONS, 5, 5},
  };
  if (!constructed) {
    assert(0 == memcmp(&builtin_interfaces__msg__Time__EXPECTED_HASH, builtin_interfaces__msg__Time__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[0].fields = builtin_interfaces__msg__Time__get_type_description(NULL)->type_description.fields;
    assert(0 == memcmp(&service_msgs__msg__ServiceEventInfo__EXPECTED_HASH, service_msgs__msg__ServiceEventInfo__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[1].fields = service_msgs__msg__ServiceEventInfo__get_type_description(NULL)->type_description.fields;
    description.referenced_type_descriptions.data[2].fields = zit6_interfaces__srv__UpdateParams_Event__get_type_description(NULL)->type_description.fields;
    description.referenced_type_descriptions.data[3].fields = zit6_interfaces__srv__UpdateParams_Request__get_type_description(NULL)->type_description.fields;
    description.referenced_type_descriptions.data[4].fields = zit6_interfaces__srv__UpdateParams_Response__get_type_description(NULL)->type_description.fields;
    constructed = true;
  }
  return &description;
}
// Define type names, field names, and default values
static char zit6_interfaces__srv__UpdateParams_Request__FIELD_NAME__json[] = "json";
static char zit6_interfaces__srv__UpdateParams_Request__FIELD_NAME__paths[] = "paths";
static char zit6_interfaces__srv__UpdateParams_Request__FIELD_NAME__values[] = "values";

static rosidl_runtime_c__type_description__Field zit6_interfaces__srv__UpdateParams_Request__FIELDS[] = {
  {
    {zit6_interfaces__srv__UpdateParams_Request__FIELD_NAME__json, 4, 4},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_STRING,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {zit6_interfaces__srv__UpdateParams_Request__FIELD_NAME__paths, 5, 5},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_STRING_UNBOUNDED_SEQUENCE,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {zit6_interfaces__srv__UpdateParams_Request__FIELD_NAME__values, 6, 6},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_STRING_UNBOUNDED_SEQUENCE,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
};

const rosidl_runtime_c__type_description__TypeDescription *
zit6_interfaces__srv__UpdateParams_Request__get_type_description(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {zit6_interfaces__srv__UpdateParams_Request__TYPE_NAME, 40, 40},
      {zit6_interfaces__srv__UpdateParams_Request__FIELDS, 3, 3},
    },
    {NULL, 0, 0},
  };
  if (!constructed) {
    constructed = true;
  }
  return &description;
}
// Define type names, field names, and default values
static char zit6_interfaces__srv__UpdateParams_Response__FIELD_NAME__success[] = "success";
static char zit6_interfaces__srv__UpdateParams_Response__FIELD_NAME__message[] = "message";

static rosidl_runtime_c__type_description__Field zit6_interfaces__srv__UpdateParams_Response__FIELDS[] = {
  {
    {zit6_interfaces__srv__UpdateParams_Response__FIELD_NAME__success, 7, 7},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_BOOLEAN,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {zit6_interfaces__srv__UpdateParams_Response__FIELD_NAME__message, 7, 7},
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
zit6_interfaces__srv__UpdateParams_Response__get_type_description(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {zit6_interfaces__srv__UpdateParams_Response__TYPE_NAME, 41, 41},
      {zit6_interfaces__srv__UpdateParams_Response__FIELDS, 2, 2},
    },
    {NULL, 0, 0},
  };
  if (!constructed) {
    constructed = true;
  }
  return &description;
}
// Define type names, field names, and default values
static char zit6_interfaces__srv__UpdateParams_Event__FIELD_NAME__info[] = "info";
static char zit6_interfaces__srv__UpdateParams_Event__FIELD_NAME__request[] = "request";
static char zit6_interfaces__srv__UpdateParams_Event__FIELD_NAME__response[] = "response";

static rosidl_runtime_c__type_description__Field zit6_interfaces__srv__UpdateParams_Event__FIELDS[] = {
  {
    {zit6_interfaces__srv__UpdateParams_Event__FIELD_NAME__info, 4, 4},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE,
      0,
      0,
      {service_msgs__msg__ServiceEventInfo__TYPE_NAME, 33, 33},
    },
    {NULL, 0, 0},
  },
  {
    {zit6_interfaces__srv__UpdateParams_Event__FIELD_NAME__request, 7, 7},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE_BOUNDED_SEQUENCE,
      1,
      0,
      {zit6_interfaces__srv__UpdateParams_Request__TYPE_NAME, 40, 40},
    },
    {NULL, 0, 0},
  },
  {
    {zit6_interfaces__srv__UpdateParams_Event__FIELD_NAME__response, 8, 8},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE_BOUNDED_SEQUENCE,
      1,
      0,
      {zit6_interfaces__srv__UpdateParams_Response__TYPE_NAME, 41, 41},
    },
    {NULL, 0, 0},
  },
};

static rosidl_runtime_c__type_description__IndividualTypeDescription zit6_interfaces__srv__UpdateParams_Event__REFERENCED_TYPE_DESCRIPTIONS[] = {
  {
    {builtin_interfaces__msg__Time__TYPE_NAME, 27, 27},
    {NULL, 0, 0},
  },
  {
    {service_msgs__msg__ServiceEventInfo__TYPE_NAME, 33, 33},
    {NULL, 0, 0},
  },
  {
    {zit6_interfaces__srv__UpdateParams_Request__TYPE_NAME, 40, 40},
    {NULL, 0, 0},
  },
  {
    {zit6_interfaces__srv__UpdateParams_Response__TYPE_NAME, 41, 41},
    {NULL, 0, 0},
  },
};

const rosidl_runtime_c__type_description__TypeDescription *
zit6_interfaces__srv__UpdateParams_Event__get_type_description(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {zit6_interfaces__srv__UpdateParams_Event__TYPE_NAME, 38, 38},
      {zit6_interfaces__srv__UpdateParams_Event__FIELDS, 3, 3},
    },
    {zit6_interfaces__srv__UpdateParams_Event__REFERENCED_TYPE_DESCRIPTIONS, 4, 4},
  };
  if (!constructed) {
    assert(0 == memcmp(&builtin_interfaces__msg__Time__EXPECTED_HASH, builtin_interfaces__msg__Time__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[0].fields = builtin_interfaces__msg__Time__get_type_description(NULL)->type_description.fields;
    assert(0 == memcmp(&service_msgs__msg__ServiceEventInfo__EXPECTED_HASH, service_msgs__msg__ServiceEventInfo__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[1].fields = service_msgs__msg__ServiceEventInfo__get_type_description(NULL)->type_description.fields;
    description.referenced_type_descriptions.data[2].fields = zit6_interfaces__srv__UpdateParams_Request__get_type_description(NULL)->type_description.fields;
    description.referenced_type_descriptions.data[3].fields = zit6_interfaces__srv__UpdateParams_Response__get_type_description(NULL)->type_description.fields;
    constructed = true;
  }
  return &description;
}

static char toplevel_type_raw_source[] =
  "# UpdateParams.srv\n"
  "# Request:\n"
  "# - `paths`: \\xe7\\x82\\xb9\\xe8\\xb7\\xaf\\xe5\\xbe\\x84\\xe6\\x95\\xb0\\xe7\\xbb\\x84\\xef\\xbc\\x88\\xe4\\xbe\\x8b\\xe5\\xa6\\x82 \"chassis.pid.pos.kp\"\\xef\\xbc\\x89\n"
  "# - `values`: \\xe4\\xb8\\x8e `paths` \\xe5\\xaf\\xb9\\xe5\\xba\\x94\\xe7\\x9a\\x84\\xe5\\xad\\x97\\xe7\\xac\\xa6\\xe4\\xb8\\xb2\\xe5\\x8c\\x96\\xe5\\x80\\xbc\\xef\\xbc\\x88\\xe6\\x9c\\x8d\\xe5\\x8a\\xa1\\xe7\\xab\\xaf\\xe8\\xb4\\x9f\\xe8\\xb4\\xa3\\xe6\\x8c\\x89\\xe7\\x8e\\xb0\\xe6\\x9c\\x89\\xe9\\x85\\x8d\\xe7\\xbd\\xae\\xe6\\x8e\\xa8\\xe6\\x96\\xad\\xe7\\xb1\\xbb\\xe5\\x9e\\x8b\\xef\\xbc\\x89\n"
  "# \\xe6\\xb3\\xa8\\xe6\\x84\\x8f\\xef\\xbc\\x9a`trigger_replan` / `persist` \\xe5\\xad\\x97\\xe6\\xae\\xb5\\xe5\\xb7\\xb2\\xe7\\xa7\\xbb\\xe9\\x99\\xa4\\xef\\xbc\\x8c\\xe7\\x94\\xb1 MCU \\xe6\\x9c\\x8d\\xe5\\x8a\\xa1\\xe7\\xab\\xaf\\xe5\\x86\\xb3\\xe5\\xae\\x9a\\xe5\\xa4\\x84\\xe7\\x90\\x86\\xe6\\x96\\xb9\\xe5\\xbc\\x8f\n"
  "# \\xe5\\x8f\\xaf\\xe9\\x80\\x89\\xef\\xbc\\x9a`json` \\xe5\\xad\\x97\\xe6\\xae\\xb5\\xe4\\xbc\\x98\\xe5\\x85\\x88\\xe8\\xa7\\xa3\\xe6\\x9e\\x90\\xef\\xbc\\x88\\xe6\\x9c\\x8d\\xe5\\x8a\\xa1\\xe7\\xab\\xaf\\xe4\\xbc\\x9a\\xe4\\xbc\\x98\\xe5\\x85\\x88\\xe5\\xa4\\x84\\xe7\\x90\\x86 `json`\\xef\\xbc\\x8c\\xe5\\x90\\xa6\\xe5\\x88\\x99\\xe6\\x8c\\x89 `paths`/`values` \\xe9\\x80\\x90\\xe9\\xa1\\xb9\\xe5\\xa4\\x84\\xe7\\x90\\x86\\xef\\xbc\\x89\n"
  "string json\n"
  "string[] paths\n"
  "string[] values\n"
  "---\n"
  "# Response:\n"
  "# - `success`: \\xe6\\x93\\x8d\\xe4\\xbd\\x9c\\xe6\\x98\\xaf\\xe5\\x90\\xa6\\xe6\\x88\\x90\\xe5\\x8a\\x9f\n"
  "# - `message`: \\xe5\\x8f\\xaf\\xe8\\xaf\\xbb\\xe7\\x9a\\x84\\xe9\\x94\\x99\\xe8\\xaf\\xaf\\xe6\\x88\\x96\\xe4\\xbf\\xa1\\xe6\\x81\\xaf\\xe8\\xaf\\xb4\\xe6\\x98\\x8e\n"
  "bool success\n"
  "string message";

static char srv_encoding[] = "srv";
static char implicit_encoding[] = "implicit";

// Define all individual source functions

const rosidl_runtime_c__type_description__TypeSource *
zit6_interfaces__srv__UpdateParams__get_individual_type_description_source(
  const rosidl_service_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {zit6_interfaces__srv__UpdateParams__TYPE_NAME, 32, 32},
    {srv_encoding, 3, 3},
    {toplevel_type_raw_source, 376, 376},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource *
zit6_interfaces__srv__UpdateParams_Request__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {zit6_interfaces__srv__UpdateParams_Request__TYPE_NAME, 40, 40},
    {implicit_encoding, 8, 8},
    {NULL, 0, 0},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource *
zit6_interfaces__srv__UpdateParams_Response__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {zit6_interfaces__srv__UpdateParams_Response__TYPE_NAME, 41, 41},
    {implicit_encoding, 8, 8},
    {NULL, 0, 0},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource *
zit6_interfaces__srv__UpdateParams_Event__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {zit6_interfaces__srv__UpdateParams_Event__TYPE_NAME, 38, 38},
    {implicit_encoding, 8, 8},
    {NULL, 0, 0},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
zit6_interfaces__srv__UpdateParams__get_type_description_sources(
  const rosidl_service_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[6];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 6, 6};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *zit6_interfaces__srv__UpdateParams__get_individual_type_description_source(NULL),
    sources[1] = *builtin_interfaces__msg__Time__get_individual_type_description_source(NULL);
    sources[2] = *service_msgs__msg__ServiceEventInfo__get_individual_type_description_source(NULL);
    sources[3] = *zit6_interfaces__srv__UpdateParams_Event__get_individual_type_description_source(NULL);
    sources[4] = *zit6_interfaces__srv__UpdateParams_Request__get_individual_type_description_source(NULL);
    sources[5] = *zit6_interfaces__srv__UpdateParams_Response__get_individual_type_description_source(NULL);
    constructed = true;
  }
  return &source_sequence;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
zit6_interfaces__srv__UpdateParams_Request__get_type_description_sources(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[1];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 1, 1};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *zit6_interfaces__srv__UpdateParams_Request__get_individual_type_description_source(NULL),
    constructed = true;
  }
  return &source_sequence;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
zit6_interfaces__srv__UpdateParams_Response__get_type_description_sources(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[1];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 1, 1};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *zit6_interfaces__srv__UpdateParams_Response__get_individual_type_description_source(NULL),
    constructed = true;
  }
  return &source_sequence;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
zit6_interfaces__srv__UpdateParams_Event__get_type_description_sources(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[5];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 5, 5};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *zit6_interfaces__srv__UpdateParams_Event__get_individual_type_description_source(NULL),
    sources[1] = *builtin_interfaces__msg__Time__get_individual_type_description_source(NULL);
    sources[2] = *service_msgs__msg__ServiceEventInfo__get_individual_type_description_source(NULL);
    sources[3] = *zit6_interfaces__srv__UpdateParams_Request__get_individual_type_description_source(NULL);
    sources[4] = *zit6_interfaces__srv__UpdateParams_Response__get_individual_type_description_source(NULL);
    constructed = true;
  }
  return &source_sequence;
}
