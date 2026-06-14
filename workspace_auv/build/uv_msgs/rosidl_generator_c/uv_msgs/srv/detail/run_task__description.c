// generated from rosidl_generator_c/resource/idl__description.c.em
// with input from uv_msgs:srv/RunTask.idl
// generated code does not contain a copyright notice

#include "uv_msgs/srv/detail/run_task__functions.h"

ROSIDL_GENERATOR_C_PUBLIC_uv_msgs
const rosidl_type_hash_t *
uv_msgs__srv__RunTask__get_type_hash(
  const rosidl_service_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0x70, 0x7a, 0x4b, 0x84, 0xff, 0xa0, 0x28, 0x97,
      0x2d, 0xef, 0xdf, 0xdc, 0xa6, 0x6a, 0x8a, 0x8b,
      0x35, 0x29, 0xa4, 0x0b, 0xab, 0xb5, 0x3a, 0xe4,
      0xf5, 0x0e, 0x6b, 0xd7, 0x81, 0x6a, 0x92, 0xf5,
    }};
  return &hash;
}

ROSIDL_GENERATOR_C_PUBLIC_uv_msgs
const rosidl_type_hash_t *
uv_msgs__srv__RunTask_Request__get_type_hash(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0xab, 0xa0, 0x1f, 0x53, 0xff, 0x02, 0x93, 0x7a,
      0xd7, 0xde, 0x50, 0x81, 0xe2, 0x3c, 0xbb, 0x6b,
      0xc1, 0x1a, 0xc0, 0xb2, 0xf0, 0x38, 0x0d, 0x8c,
      0xb2, 0x82, 0x1f, 0x01, 0xa5, 0xda, 0x3d, 0xef,
    }};
  return &hash;
}

ROSIDL_GENERATOR_C_PUBLIC_uv_msgs
const rosidl_type_hash_t *
uv_msgs__srv__RunTask_Response__get_type_hash(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0xba, 0x91, 0xfb, 0x1d, 0xcc, 0x09, 0x6e, 0x87,
      0xf9, 0xc2, 0xfc, 0x7f, 0x83, 0x67, 0xcb, 0xa2,
      0x02, 0x85, 0xb3, 0x11, 0x56, 0x22, 0xd3, 0x13,
      0x03, 0x8a, 0xfc, 0x8f, 0x97, 0x91, 0x24, 0x92,
    }};
  return &hash;
}

ROSIDL_GENERATOR_C_PUBLIC_uv_msgs
const rosidl_type_hash_t *
uv_msgs__srv__RunTask_Event__get_type_hash(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0x19, 0xb1, 0xed, 0xdb, 0x8d, 0x06, 0x8a, 0x7f,
      0xa3, 0x00, 0x95, 0xbc, 0x1a, 0x70, 0xa4, 0x8a,
      0xfc, 0x21, 0xf8, 0xc1, 0x5a, 0x1d, 0x55, 0xcf,
      0x3e, 0x5f, 0x1f, 0xcb, 0x1a, 0xad, 0x9b, 0xe8,
    }};
  return &hash;
}

#include <assert.h>
#include <string.h>

// Include directives for referenced types
#include "builtin_interfaces/msg/detail/time__functions.h"
#include "service_msgs/msg/detail/service_event_info__functions.h"

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

static char uv_msgs__srv__RunTask__TYPE_NAME[] = "uv_msgs/srv/RunTask";
static char builtin_interfaces__msg__Time__TYPE_NAME[] = "builtin_interfaces/msg/Time";
static char service_msgs__msg__ServiceEventInfo__TYPE_NAME[] = "service_msgs/msg/ServiceEventInfo";
static char uv_msgs__srv__RunTask_Event__TYPE_NAME[] = "uv_msgs/srv/RunTask_Event";
static char uv_msgs__srv__RunTask_Request__TYPE_NAME[] = "uv_msgs/srv/RunTask_Request";
static char uv_msgs__srv__RunTask_Response__TYPE_NAME[] = "uv_msgs/srv/RunTask_Response";

// Define type names, field names, and default values
static char uv_msgs__srv__RunTask__FIELD_NAME__request_message[] = "request_message";
static char uv_msgs__srv__RunTask__FIELD_NAME__response_message[] = "response_message";
static char uv_msgs__srv__RunTask__FIELD_NAME__event_message[] = "event_message";

static rosidl_runtime_c__type_description__Field uv_msgs__srv__RunTask__FIELDS[] = {
  {
    {uv_msgs__srv__RunTask__FIELD_NAME__request_message, 15, 15},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE,
      0,
      0,
      {uv_msgs__srv__RunTask_Request__TYPE_NAME, 27, 27},
    },
    {NULL, 0, 0},
  },
  {
    {uv_msgs__srv__RunTask__FIELD_NAME__response_message, 16, 16},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE,
      0,
      0,
      {uv_msgs__srv__RunTask_Response__TYPE_NAME, 28, 28},
    },
    {NULL, 0, 0},
  },
  {
    {uv_msgs__srv__RunTask__FIELD_NAME__event_message, 13, 13},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE,
      0,
      0,
      {uv_msgs__srv__RunTask_Event__TYPE_NAME, 25, 25},
    },
    {NULL, 0, 0},
  },
};

static rosidl_runtime_c__type_description__IndividualTypeDescription uv_msgs__srv__RunTask__REFERENCED_TYPE_DESCRIPTIONS[] = {
  {
    {builtin_interfaces__msg__Time__TYPE_NAME, 27, 27},
    {NULL, 0, 0},
  },
  {
    {service_msgs__msg__ServiceEventInfo__TYPE_NAME, 33, 33},
    {NULL, 0, 0},
  },
  {
    {uv_msgs__srv__RunTask_Event__TYPE_NAME, 25, 25},
    {NULL, 0, 0},
  },
  {
    {uv_msgs__srv__RunTask_Request__TYPE_NAME, 27, 27},
    {NULL, 0, 0},
  },
  {
    {uv_msgs__srv__RunTask_Response__TYPE_NAME, 28, 28},
    {NULL, 0, 0},
  },
};

const rosidl_runtime_c__type_description__TypeDescription *
uv_msgs__srv__RunTask__get_type_description(
  const rosidl_service_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {uv_msgs__srv__RunTask__TYPE_NAME, 19, 19},
      {uv_msgs__srv__RunTask__FIELDS, 3, 3},
    },
    {uv_msgs__srv__RunTask__REFERENCED_TYPE_DESCRIPTIONS, 5, 5},
  };
  if (!constructed) {
    assert(0 == memcmp(&builtin_interfaces__msg__Time__EXPECTED_HASH, builtin_interfaces__msg__Time__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[0].fields = builtin_interfaces__msg__Time__get_type_description(NULL)->type_description.fields;
    assert(0 == memcmp(&service_msgs__msg__ServiceEventInfo__EXPECTED_HASH, service_msgs__msg__ServiceEventInfo__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[1].fields = service_msgs__msg__ServiceEventInfo__get_type_description(NULL)->type_description.fields;
    description.referenced_type_descriptions.data[2].fields = uv_msgs__srv__RunTask_Event__get_type_description(NULL)->type_description.fields;
    description.referenced_type_descriptions.data[3].fields = uv_msgs__srv__RunTask_Request__get_type_description(NULL)->type_description.fields;
    description.referenced_type_descriptions.data[4].fields = uv_msgs__srv__RunTask_Response__get_type_description(NULL)->type_description.fields;
    constructed = true;
  }
  return &description;
}
// Define type names, field names, and default values
static char uv_msgs__srv__RunTask_Request__FIELD_NAME__task_name[] = "task_name";
static char uv_msgs__srv__RunTask_Request__FIELD_NAME__start[] = "start";

static rosidl_runtime_c__type_description__Field uv_msgs__srv__RunTask_Request__FIELDS[] = {
  {
    {uv_msgs__srv__RunTask_Request__FIELD_NAME__task_name, 9, 9},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_STRING,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {uv_msgs__srv__RunTask_Request__FIELD_NAME__start, 5, 5},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_BOOLEAN,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
};

const rosidl_runtime_c__type_description__TypeDescription *
uv_msgs__srv__RunTask_Request__get_type_description(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {uv_msgs__srv__RunTask_Request__TYPE_NAME, 27, 27},
      {uv_msgs__srv__RunTask_Request__FIELDS, 2, 2},
    },
    {NULL, 0, 0},
  };
  if (!constructed) {
    constructed = true;
  }
  return &description;
}
// Define type names, field names, and default values
static char uv_msgs__srv__RunTask_Response__FIELD_NAME__success[] = "success";
static char uv_msgs__srv__RunTask_Response__FIELD_NAME__message[] = "message";

static rosidl_runtime_c__type_description__Field uv_msgs__srv__RunTask_Response__FIELDS[] = {
  {
    {uv_msgs__srv__RunTask_Response__FIELD_NAME__success, 7, 7},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_BOOLEAN,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {uv_msgs__srv__RunTask_Response__FIELD_NAME__message, 7, 7},
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
uv_msgs__srv__RunTask_Response__get_type_description(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {uv_msgs__srv__RunTask_Response__TYPE_NAME, 28, 28},
      {uv_msgs__srv__RunTask_Response__FIELDS, 2, 2},
    },
    {NULL, 0, 0},
  };
  if (!constructed) {
    constructed = true;
  }
  return &description;
}
// Define type names, field names, and default values
static char uv_msgs__srv__RunTask_Event__FIELD_NAME__info[] = "info";
static char uv_msgs__srv__RunTask_Event__FIELD_NAME__request[] = "request";
static char uv_msgs__srv__RunTask_Event__FIELD_NAME__response[] = "response";

static rosidl_runtime_c__type_description__Field uv_msgs__srv__RunTask_Event__FIELDS[] = {
  {
    {uv_msgs__srv__RunTask_Event__FIELD_NAME__info, 4, 4},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE,
      0,
      0,
      {service_msgs__msg__ServiceEventInfo__TYPE_NAME, 33, 33},
    },
    {NULL, 0, 0},
  },
  {
    {uv_msgs__srv__RunTask_Event__FIELD_NAME__request, 7, 7},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE_BOUNDED_SEQUENCE,
      1,
      0,
      {uv_msgs__srv__RunTask_Request__TYPE_NAME, 27, 27},
    },
    {NULL, 0, 0},
  },
  {
    {uv_msgs__srv__RunTask_Event__FIELD_NAME__response, 8, 8},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE_BOUNDED_SEQUENCE,
      1,
      0,
      {uv_msgs__srv__RunTask_Response__TYPE_NAME, 28, 28},
    },
    {NULL, 0, 0},
  },
};

static rosidl_runtime_c__type_description__IndividualTypeDescription uv_msgs__srv__RunTask_Event__REFERENCED_TYPE_DESCRIPTIONS[] = {
  {
    {builtin_interfaces__msg__Time__TYPE_NAME, 27, 27},
    {NULL, 0, 0},
  },
  {
    {service_msgs__msg__ServiceEventInfo__TYPE_NAME, 33, 33},
    {NULL, 0, 0},
  },
  {
    {uv_msgs__srv__RunTask_Request__TYPE_NAME, 27, 27},
    {NULL, 0, 0},
  },
  {
    {uv_msgs__srv__RunTask_Response__TYPE_NAME, 28, 28},
    {NULL, 0, 0},
  },
};

const rosidl_runtime_c__type_description__TypeDescription *
uv_msgs__srv__RunTask_Event__get_type_description(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {uv_msgs__srv__RunTask_Event__TYPE_NAME, 25, 25},
      {uv_msgs__srv__RunTask_Event__FIELDS, 3, 3},
    },
    {uv_msgs__srv__RunTask_Event__REFERENCED_TYPE_DESCRIPTIONS, 4, 4},
  };
  if (!constructed) {
    assert(0 == memcmp(&builtin_interfaces__msg__Time__EXPECTED_HASH, builtin_interfaces__msg__Time__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[0].fields = builtin_interfaces__msg__Time__get_type_description(NULL)->type_description.fields;
    assert(0 == memcmp(&service_msgs__msg__ServiceEventInfo__EXPECTED_HASH, service_msgs__msg__ServiceEventInfo__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[1].fields = service_msgs__msg__ServiceEventInfo__get_type_description(NULL)->type_description.fields;
    description.referenced_type_descriptions.data[2].fields = uv_msgs__srv__RunTask_Request__get_type_description(NULL)->type_description.fields;
    description.referenced_type_descriptions.data[3].fields = uv_msgs__srv__RunTask_Response__get_type_description(NULL)->type_description.fields;
    constructed = true;
  }
  return &description;
}

static char toplevel_type_raw_source[] =
  "# Request to run a named task or task list\n"
  "\n"
  "string task_name      # Name of task or path to JSON file\n"
  "bool start            # true=start, false=stop\n"
  "---\n"
  "bool success\n"
  "string message";

static char srv_encoding[] = "srv";
static char implicit_encoding[] = "implicit";

// Define all individual source functions

const rosidl_runtime_c__type_description__TypeSource *
uv_msgs__srv__RunTask__get_individual_type_description_source(
  const rosidl_service_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {uv_msgs__srv__RunTask__TYPE_NAME, 19, 19},
    {srv_encoding, 3, 3},
    {toplevel_type_raw_source, 181, 181},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource *
uv_msgs__srv__RunTask_Request__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {uv_msgs__srv__RunTask_Request__TYPE_NAME, 27, 27},
    {implicit_encoding, 8, 8},
    {NULL, 0, 0},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource *
uv_msgs__srv__RunTask_Response__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {uv_msgs__srv__RunTask_Response__TYPE_NAME, 28, 28},
    {implicit_encoding, 8, 8},
    {NULL, 0, 0},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource *
uv_msgs__srv__RunTask_Event__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {uv_msgs__srv__RunTask_Event__TYPE_NAME, 25, 25},
    {implicit_encoding, 8, 8},
    {NULL, 0, 0},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
uv_msgs__srv__RunTask__get_type_description_sources(
  const rosidl_service_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[6];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 6, 6};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *uv_msgs__srv__RunTask__get_individual_type_description_source(NULL),
    sources[1] = *builtin_interfaces__msg__Time__get_individual_type_description_source(NULL);
    sources[2] = *service_msgs__msg__ServiceEventInfo__get_individual_type_description_source(NULL);
    sources[3] = *uv_msgs__srv__RunTask_Event__get_individual_type_description_source(NULL);
    sources[4] = *uv_msgs__srv__RunTask_Request__get_individual_type_description_source(NULL);
    sources[5] = *uv_msgs__srv__RunTask_Response__get_individual_type_description_source(NULL);
    constructed = true;
  }
  return &source_sequence;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
uv_msgs__srv__RunTask_Request__get_type_description_sources(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[1];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 1, 1};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *uv_msgs__srv__RunTask_Request__get_individual_type_description_source(NULL),
    constructed = true;
  }
  return &source_sequence;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
uv_msgs__srv__RunTask_Response__get_type_description_sources(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[1];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 1, 1};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *uv_msgs__srv__RunTask_Response__get_individual_type_description_source(NULL),
    constructed = true;
  }
  return &source_sequence;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
uv_msgs__srv__RunTask_Event__get_type_description_sources(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[5];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 5, 5};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *uv_msgs__srv__RunTask_Event__get_individual_type_description_source(NULL),
    sources[1] = *builtin_interfaces__msg__Time__get_individual_type_description_source(NULL);
    sources[2] = *service_msgs__msg__ServiceEventInfo__get_individual_type_description_source(NULL);
    sources[3] = *uv_msgs__srv__RunTask_Request__get_individual_type_description_source(NULL);
    sources[4] = *uv_msgs__srv__RunTask_Response__get_individual_type_description_source(NULL);
    constructed = true;
  }
  return &source_sequence;
}
