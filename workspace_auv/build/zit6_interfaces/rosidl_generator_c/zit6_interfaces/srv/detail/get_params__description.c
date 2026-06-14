// generated from rosidl_generator_c/resource/idl__description.c.em
// with input from zit6_interfaces:srv/GetParams.idl
// generated code does not contain a copyright notice

#include "zit6_interfaces/srv/detail/get_params__functions.h"

ROSIDL_GENERATOR_C_PUBLIC_zit6_interfaces
const rosidl_type_hash_t *
zit6_interfaces__srv__GetParams__get_type_hash(
  const rosidl_service_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0x5b, 0x9c, 0x9b, 0x21, 0x53, 0xba, 0x1f, 0xbc,
      0xd2, 0x72, 0x7a, 0x2e, 0x97, 0x15, 0xf2, 0x13,
      0x08, 0x33, 0xb6, 0xb5, 0xa7, 0xfa, 0x15, 0x09,
      0x8f, 0x70, 0x4e, 0xd5, 0x4d, 0x05, 0x98, 0x7f,
    }};
  return &hash;
}

ROSIDL_GENERATOR_C_PUBLIC_zit6_interfaces
const rosidl_type_hash_t *
zit6_interfaces__srv__GetParams_Request__get_type_hash(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0xa0, 0x97, 0xa0, 0x29, 0x2f, 0x09, 0xae, 0x2d,
      0xe9, 0x36, 0x62, 0xac, 0x69, 0x41, 0x15, 0x19,
      0x2e, 0x70, 0xa0, 0x74, 0x75, 0x15, 0x6e, 0x8c,
      0xac, 0xfc, 0xd4, 0x32, 0xa6, 0x0f, 0x98, 0x71,
    }};
  return &hash;
}

ROSIDL_GENERATOR_C_PUBLIC_zit6_interfaces
const rosidl_type_hash_t *
zit6_interfaces__srv__GetParams_Response__get_type_hash(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0xde, 0x6e, 0xae, 0x45, 0x49, 0x3b, 0x1a, 0x16,
      0xab, 0x25, 0x57, 0xec, 0xbe, 0xa5, 0xfc, 0xa8,
      0xf9, 0x75, 0xd1, 0x43, 0x3a, 0x0c, 0xbd, 0xd8,
      0x89, 0x11, 0x35, 0x9e, 0xc9, 0x36, 0x12, 0x92,
    }};
  return &hash;
}

ROSIDL_GENERATOR_C_PUBLIC_zit6_interfaces
const rosidl_type_hash_t *
zit6_interfaces__srv__GetParams_Event__get_type_hash(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0x56, 0x0b, 0x55, 0xf0, 0x12, 0x01, 0x96, 0x4f,
      0x8b, 0x09, 0xce, 0x5d, 0xc1, 0xbd, 0xe6, 0x31,
      0x9d, 0xe1, 0xf5, 0xe5, 0x50, 0x64, 0x58, 0x27,
      0x79, 0x62, 0x8e, 0xd8, 0x90, 0x67, 0x62, 0xc0,
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

static char zit6_interfaces__srv__GetParams__TYPE_NAME[] = "zit6_interfaces/srv/GetParams";
static char builtin_interfaces__msg__Time__TYPE_NAME[] = "builtin_interfaces/msg/Time";
static char service_msgs__msg__ServiceEventInfo__TYPE_NAME[] = "service_msgs/msg/ServiceEventInfo";
static char zit6_interfaces__srv__GetParams_Event__TYPE_NAME[] = "zit6_interfaces/srv/GetParams_Event";
static char zit6_interfaces__srv__GetParams_Request__TYPE_NAME[] = "zit6_interfaces/srv/GetParams_Request";
static char zit6_interfaces__srv__GetParams_Response__TYPE_NAME[] = "zit6_interfaces/srv/GetParams_Response";

// Define type names, field names, and default values
static char zit6_interfaces__srv__GetParams__FIELD_NAME__request_message[] = "request_message";
static char zit6_interfaces__srv__GetParams__FIELD_NAME__response_message[] = "response_message";
static char zit6_interfaces__srv__GetParams__FIELD_NAME__event_message[] = "event_message";

static rosidl_runtime_c__type_description__Field zit6_interfaces__srv__GetParams__FIELDS[] = {
  {
    {zit6_interfaces__srv__GetParams__FIELD_NAME__request_message, 15, 15},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE,
      0,
      0,
      {zit6_interfaces__srv__GetParams_Request__TYPE_NAME, 37, 37},
    },
    {NULL, 0, 0},
  },
  {
    {zit6_interfaces__srv__GetParams__FIELD_NAME__response_message, 16, 16},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE,
      0,
      0,
      {zit6_interfaces__srv__GetParams_Response__TYPE_NAME, 38, 38},
    },
    {NULL, 0, 0},
  },
  {
    {zit6_interfaces__srv__GetParams__FIELD_NAME__event_message, 13, 13},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE,
      0,
      0,
      {zit6_interfaces__srv__GetParams_Event__TYPE_NAME, 35, 35},
    },
    {NULL, 0, 0},
  },
};

static rosidl_runtime_c__type_description__IndividualTypeDescription zit6_interfaces__srv__GetParams__REFERENCED_TYPE_DESCRIPTIONS[] = {
  {
    {builtin_interfaces__msg__Time__TYPE_NAME, 27, 27},
    {NULL, 0, 0},
  },
  {
    {service_msgs__msg__ServiceEventInfo__TYPE_NAME, 33, 33},
    {NULL, 0, 0},
  },
  {
    {zit6_interfaces__srv__GetParams_Event__TYPE_NAME, 35, 35},
    {NULL, 0, 0},
  },
  {
    {zit6_interfaces__srv__GetParams_Request__TYPE_NAME, 37, 37},
    {NULL, 0, 0},
  },
  {
    {zit6_interfaces__srv__GetParams_Response__TYPE_NAME, 38, 38},
    {NULL, 0, 0},
  },
};

const rosidl_runtime_c__type_description__TypeDescription *
zit6_interfaces__srv__GetParams__get_type_description(
  const rosidl_service_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {zit6_interfaces__srv__GetParams__TYPE_NAME, 29, 29},
      {zit6_interfaces__srv__GetParams__FIELDS, 3, 3},
    },
    {zit6_interfaces__srv__GetParams__REFERENCED_TYPE_DESCRIPTIONS, 5, 5},
  };
  if (!constructed) {
    assert(0 == memcmp(&builtin_interfaces__msg__Time__EXPECTED_HASH, builtin_interfaces__msg__Time__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[0].fields = builtin_interfaces__msg__Time__get_type_description(NULL)->type_description.fields;
    assert(0 == memcmp(&service_msgs__msg__ServiceEventInfo__EXPECTED_HASH, service_msgs__msg__ServiceEventInfo__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[1].fields = service_msgs__msg__ServiceEventInfo__get_type_description(NULL)->type_description.fields;
    description.referenced_type_descriptions.data[2].fields = zit6_interfaces__srv__GetParams_Event__get_type_description(NULL)->type_description.fields;
    description.referenced_type_descriptions.data[3].fields = zit6_interfaces__srv__GetParams_Request__get_type_description(NULL)->type_description.fields;
    description.referenced_type_descriptions.data[4].fields = zit6_interfaces__srv__GetParams_Response__get_type_description(NULL)->type_description.fields;
    constructed = true;
  }
  return &description;
}
// Define type names, field names, and default values
static char zit6_interfaces__srv__GetParams_Request__FIELD_NAME__paths[] = "paths";

static rosidl_runtime_c__type_description__Field zit6_interfaces__srv__GetParams_Request__FIELDS[] = {
  {
    {zit6_interfaces__srv__GetParams_Request__FIELD_NAME__paths, 5, 5},
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
zit6_interfaces__srv__GetParams_Request__get_type_description(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {zit6_interfaces__srv__GetParams_Request__TYPE_NAME, 37, 37},
      {zit6_interfaces__srv__GetParams_Request__FIELDS, 1, 1},
    },
    {NULL, 0, 0},
  };
  if (!constructed) {
    constructed = true;
  }
  return &description;
}
// Define type names, field names, and default values
static char zit6_interfaces__srv__GetParams_Response__FIELD_NAME__success[] = "success";
static char zit6_interfaces__srv__GetParams_Response__FIELD_NAME__message[] = "message";
static char zit6_interfaces__srv__GetParams_Response__FIELD_NAME__config_json[] = "config_json";

static rosidl_runtime_c__type_description__Field zit6_interfaces__srv__GetParams_Response__FIELDS[] = {
  {
    {zit6_interfaces__srv__GetParams_Response__FIELD_NAME__success, 7, 7},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_BOOLEAN,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {zit6_interfaces__srv__GetParams_Response__FIELD_NAME__message, 7, 7},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_STRING,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {zit6_interfaces__srv__GetParams_Response__FIELD_NAME__config_json, 11, 11},
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
zit6_interfaces__srv__GetParams_Response__get_type_description(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {zit6_interfaces__srv__GetParams_Response__TYPE_NAME, 38, 38},
      {zit6_interfaces__srv__GetParams_Response__FIELDS, 3, 3},
    },
    {NULL, 0, 0},
  };
  if (!constructed) {
    constructed = true;
  }
  return &description;
}
// Define type names, field names, and default values
static char zit6_interfaces__srv__GetParams_Event__FIELD_NAME__info[] = "info";
static char zit6_interfaces__srv__GetParams_Event__FIELD_NAME__request[] = "request";
static char zit6_interfaces__srv__GetParams_Event__FIELD_NAME__response[] = "response";

static rosidl_runtime_c__type_description__Field zit6_interfaces__srv__GetParams_Event__FIELDS[] = {
  {
    {zit6_interfaces__srv__GetParams_Event__FIELD_NAME__info, 4, 4},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE,
      0,
      0,
      {service_msgs__msg__ServiceEventInfo__TYPE_NAME, 33, 33},
    },
    {NULL, 0, 0},
  },
  {
    {zit6_interfaces__srv__GetParams_Event__FIELD_NAME__request, 7, 7},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE_BOUNDED_SEQUENCE,
      1,
      0,
      {zit6_interfaces__srv__GetParams_Request__TYPE_NAME, 37, 37},
    },
    {NULL, 0, 0},
  },
  {
    {zit6_interfaces__srv__GetParams_Event__FIELD_NAME__response, 8, 8},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE_BOUNDED_SEQUENCE,
      1,
      0,
      {zit6_interfaces__srv__GetParams_Response__TYPE_NAME, 38, 38},
    },
    {NULL, 0, 0},
  },
};

static rosidl_runtime_c__type_description__IndividualTypeDescription zit6_interfaces__srv__GetParams_Event__REFERENCED_TYPE_DESCRIPTIONS[] = {
  {
    {builtin_interfaces__msg__Time__TYPE_NAME, 27, 27},
    {NULL, 0, 0},
  },
  {
    {service_msgs__msg__ServiceEventInfo__TYPE_NAME, 33, 33},
    {NULL, 0, 0},
  },
  {
    {zit6_interfaces__srv__GetParams_Request__TYPE_NAME, 37, 37},
    {NULL, 0, 0},
  },
  {
    {zit6_interfaces__srv__GetParams_Response__TYPE_NAME, 38, 38},
    {NULL, 0, 0},
  },
};

const rosidl_runtime_c__type_description__TypeDescription *
zit6_interfaces__srv__GetParams_Event__get_type_description(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {zit6_interfaces__srv__GetParams_Event__TYPE_NAME, 35, 35},
      {zit6_interfaces__srv__GetParams_Event__FIELDS, 3, 3},
    },
    {zit6_interfaces__srv__GetParams_Event__REFERENCED_TYPE_DESCRIPTIONS, 4, 4},
  };
  if (!constructed) {
    assert(0 == memcmp(&builtin_interfaces__msg__Time__EXPECTED_HASH, builtin_interfaces__msg__Time__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[0].fields = builtin_interfaces__msg__Time__get_type_description(NULL)->type_description.fields;
    assert(0 == memcmp(&service_msgs__msg__ServiceEventInfo__EXPECTED_HASH, service_msgs__msg__ServiceEventInfo__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[1].fields = service_msgs__msg__ServiceEventInfo__get_type_description(NULL)->type_description.fields;
    description.referenced_type_descriptions.data[2].fields = zit6_interfaces__srv__GetParams_Request__get_type_description(NULL)->type_description.fields;
    description.referenced_type_descriptions.data[3].fields = zit6_interfaces__srv__GetParams_Response__get_type_description(NULL)->type_description.fields;
    constructed = true;
  }
  return &description;
}

static char toplevel_type_raw_source[] =
  "# GetParams.srv\n"
  "# Request:\n"
  "# - `paths`: \\xe5\\x8f\\xaf\\xe9\\x80\\x89\\xe7\\x9a\\x84\\xe7\\x82\\xb9\\xe8\\xb7\\xaf\\xe5\\xbe\\x84\\xe6\\x95\\xb0\\xe7\\xbb\\x84\\xef\\xbc\\x88\\xe4\\xbe\\x8b\\xe5\\xa6\\x82 \"chassis.pid.pos.kp\"\\xef\\xbc\\x89\\xef\\xbc\\x9b\\xe4\\xb8\\xba\\xe7\\xa9\\xba\\xe8\\xa1\\xa8\\xe7\\xa4\\xba\\xe8\\xbf\\x94\\xe5\\x9b\\x9e\\xe5\\x85\\xa8\\xe9\\x83\\xa8\\xe9\\x85\\x8d\\xe7\\xbd\\xae\n"
  "string[] paths\n"
  "---\n"
  "# Response:\n"
  "# - `success`: \\xe6\\x98\\xaf\\xe5\\x90\\xa6\\xe6\\x88\\x90\\xe5\\x8a\\x9f\n"
  "# - `message`: \\xe5\\xa4\\xb1\\xe8\\xb4\\xa5\\xe6\\x97\\xb6\\xe7\\x9a\\x84\\xe8\\xaf\\xb4\\xe6\\x98\\x8e\n"
  "# - `config_json`: JSON \\xe5\\xad\\x97\\xe7\\xac\\xa6\\xe4\\xb8\\xb2\\xef\\xbc\\x8c\\xe5\\x8c\\x85\\xe5\\x90\\xab\\xe6\\x89\\x80\\xe8\\xaf\\xb7\\xe6\\xb1\\x82\\xe7\\x9a\\x84\\xe5\\x8f\\x82\\xe6\\x95\\xb0\\xef\\xbc\\x88\\xe7\\xa9\\xba paths \\xe8\\xbf\\x94\\xe5\\x9b\\x9e\\xe5\\xae\\x8c\\xe6\\x95\\xb4\\xe9\\x85\\x8d\\xe7\\xbd\\xae\\xef\\xbc\\x89\n"
  "bool success\n"
  "string message\n"
  "string config_json";

static char srv_encoding[] = "srv";
static char implicit_encoding[] = "implicit";

// Define all individual source functions

const rosidl_runtime_c__type_description__TypeSource *
zit6_interfaces__srv__GetParams__get_individual_type_description_source(
  const rosidl_service_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {zit6_interfaces__srv__GetParams__TYPE_NAME, 29, 29},
    {srv_encoding, 3, 3},
    {toplevel_type_raw_source, 258, 258},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource *
zit6_interfaces__srv__GetParams_Request__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {zit6_interfaces__srv__GetParams_Request__TYPE_NAME, 37, 37},
    {implicit_encoding, 8, 8},
    {NULL, 0, 0},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource *
zit6_interfaces__srv__GetParams_Response__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {zit6_interfaces__srv__GetParams_Response__TYPE_NAME, 38, 38},
    {implicit_encoding, 8, 8},
    {NULL, 0, 0},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource *
zit6_interfaces__srv__GetParams_Event__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {zit6_interfaces__srv__GetParams_Event__TYPE_NAME, 35, 35},
    {implicit_encoding, 8, 8},
    {NULL, 0, 0},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
zit6_interfaces__srv__GetParams__get_type_description_sources(
  const rosidl_service_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[6];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 6, 6};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *zit6_interfaces__srv__GetParams__get_individual_type_description_source(NULL),
    sources[1] = *builtin_interfaces__msg__Time__get_individual_type_description_source(NULL);
    sources[2] = *service_msgs__msg__ServiceEventInfo__get_individual_type_description_source(NULL);
    sources[3] = *zit6_interfaces__srv__GetParams_Event__get_individual_type_description_source(NULL);
    sources[4] = *zit6_interfaces__srv__GetParams_Request__get_individual_type_description_source(NULL);
    sources[5] = *zit6_interfaces__srv__GetParams_Response__get_individual_type_description_source(NULL);
    constructed = true;
  }
  return &source_sequence;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
zit6_interfaces__srv__GetParams_Request__get_type_description_sources(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[1];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 1, 1};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *zit6_interfaces__srv__GetParams_Request__get_individual_type_description_source(NULL),
    constructed = true;
  }
  return &source_sequence;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
zit6_interfaces__srv__GetParams_Response__get_type_description_sources(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[1];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 1, 1};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *zit6_interfaces__srv__GetParams_Response__get_individual_type_description_source(NULL),
    constructed = true;
  }
  return &source_sequence;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
zit6_interfaces__srv__GetParams_Event__get_type_description_sources(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[5];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 5, 5};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *zit6_interfaces__srv__GetParams_Event__get_individual_type_description_source(NULL),
    sources[1] = *builtin_interfaces__msg__Time__get_individual_type_description_source(NULL);
    sources[2] = *service_msgs__msg__ServiceEventInfo__get_individual_type_description_source(NULL);
    sources[3] = *zit6_interfaces__srv__GetParams_Request__get_individual_type_description_source(NULL);
    sources[4] = *zit6_interfaces__srv__GetParams_Response__get_individual_type_description_source(NULL);
    constructed = true;
  }
  return &source_sequence;
}
