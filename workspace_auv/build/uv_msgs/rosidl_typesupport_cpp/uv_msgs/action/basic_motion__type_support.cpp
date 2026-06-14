// generated from rosidl_typesupport_cpp/resource/idl__type_support.cpp.em
// with input from uv_msgs:action/BasicMotion.idl
// generated code does not contain a copyright notice

#include "cstddef"
#include "rosidl_runtime_c/message_type_support_struct.h"
#include "uv_msgs/action/detail/basic_motion__functions.h"
#include "uv_msgs/action/detail/basic_motion__struct.hpp"
#include "rosidl_typesupport_cpp/identifier.hpp"
#include "rosidl_typesupport_cpp/message_type_support.hpp"
#include "rosidl_typesupport_c/type_support_map.h"
#include "rosidl_typesupport_cpp/message_type_support_dispatch.hpp"
#include "rosidl_typesupport_cpp/visibility_control.h"
#include "rosidl_typesupport_interface/macros.h"

namespace uv_msgs
{

namespace action
{

namespace rosidl_typesupport_cpp
{

typedef struct _BasicMotion_Goal_type_support_ids_t
{
  const char * typesupport_identifier[2];
} _BasicMotion_Goal_type_support_ids_t;

static const _BasicMotion_Goal_type_support_ids_t _BasicMotion_Goal_message_typesupport_ids = {
  {
    "rosidl_typesupport_fastrtps_cpp",  // ::rosidl_typesupport_fastrtps_cpp::typesupport_identifier,
    "rosidl_typesupport_introspection_cpp",  // ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  }
};

typedef struct _BasicMotion_Goal_type_support_symbol_names_t
{
  const char * symbol_name[2];
} _BasicMotion_Goal_type_support_symbol_names_t;

#define STRINGIFY_(s) #s
#define STRINGIFY(s) STRINGIFY_(s)

static const _BasicMotion_Goal_type_support_symbol_names_t _BasicMotion_Goal_message_typesupport_symbol_names = {
  {
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_cpp, uv_msgs, action, BasicMotion_Goal)),
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, uv_msgs, action, BasicMotion_Goal)),
  }
};

typedef struct _BasicMotion_Goal_type_support_data_t
{
  void * data[2];
} _BasicMotion_Goal_type_support_data_t;

static _BasicMotion_Goal_type_support_data_t _BasicMotion_Goal_message_typesupport_data = {
  {
    0,  // will store the shared library later
    0,  // will store the shared library later
  }
};

static const type_support_map_t _BasicMotion_Goal_message_typesupport_map = {
  2,
  "uv_msgs",
  &_BasicMotion_Goal_message_typesupport_ids.typesupport_identifier[0],
  &_BasicMotion_Goal_message_typesupport_symbol_names.symbol_name[0],
  &_BasicMotion_Goal_message_typesupport_data.data[0],
};

static const rosidl_message_type_support_t BasicMotion_Goal_message_type_support_handle = {
  ::rosidl_typesupport_cpp::typesupport_identifier,
  reinterpret_cast<const type_support_map_t *>(&_BasicMotion_Goal_message_typesupport_map),
  ::rosidl_typesupport_cpp::get_message_typesupport_handle_function,
  &uv_msgs__action__BasicMotion_Goal__get_type_hash,
  &uv_msgs__action__BasicMotion_Goal__get_type_description,
  &uv_msgs__action__BasicMotion_Goal__get_type_description_sources,
};

}  // namespace rosidl_typesupport_cpp

}  // namespace action

}  // namespace uv_msgs

namespace rosidl_typesupport_cpp
{

template<>
ROSIDL_TYPESUPPORT_CPP_PUBLIC
const rosidl_message_type_support_t *
get_message_type_support_handle<uv_msgs::action::BasicMotion_Goal>()
{
  return &::uv_msgs::action::rosidl_typesupport_cpp::BasicMotion_Goal_message_type_support_handle;
}

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_CPP_PUBLIC
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_cpp, uv_msgs, action, BasicMotion_Goal)() {
  return get_message_type_support_handle<uv_msgs::action::BasicMotion_Goal>();
}

#ifdef __cplusplus
}
#endif
}  // namespace rosidl_typesupport_cpp

// already included above
// #include "cstddef"
// already included above
// #include "rosidl_runtime_c/message_type_support_struct.h"
// already included above
// #include "uv_msgs/action/detail/basic_motion__functions.h"
// already included above
// #include "uv_msgs/action/detail/basic_motion__struct.hpp"
// already included above
// #include "rosidl_typesupport_cpp/identifier.hpp"
// already included above
// #include "rosidl_typesupport_cpp/message_type_support.hpp"
// already included above
// #include "rosidl_typesupport_c/type_support_map.h"
// already included above
// #include "rosidl_typesupport_cpp/message_type_support_dispatch.hpp"
// already included above
// #include "rosidl_typesupport_cpp/visibility_control.h"
// already included above
// #include "rosidl_typesupport_interface/macros.h"

namespace uv_msgs
{

namespace action
{

namespace rosidl_typesupport_cpp
{

typedef struct _BasicMotion_Result_type_support_ids_t
{
  const char * typesupport_identifier[2];
} _BasicMotion_Result_type_support_ids_t;

static const _BasicMotion_Result_type_support_ids_t _BasicMotion_Result_message_typesupport_ids = {
  {
    "rosidl_typesupport_fastrtps_cpp",  // ::rosidl_typesupport_fastrtps_cpp::typesupport_identifier,
    "rosidl_typesupport_introspection_cpp",  // ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  }
};

typedef struct _BasicMotion_Result_type_support_symbol_names_t
{
  const char * symbol_name[2];
} _BasicMotion_Result_type_support_symbol_names_t;

#define STRINGIFY_(s) #s
#define STRINGIFY(s) STRINGIFY_(s)

static const _BasicMotion_Result_type_support_symbol_names_t _BasicMotion_Result_message_typesupport_symbol_names = {
  {
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_cpp, uv_msgs, action, BasicMotion_Result)),
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, uv_msgs, action, BasicMotion_Result)),
  }
};

typedef struct _BasicMotion_Result_type_support_data_t
{
  void * data[2];
} _BasicMotion_Result_type_support_data_t;

static _BasicMotion_Result_type_support_data_t _BasicMotion_Result_message_typesupport_data = {
  {
    0,  // will store the shared library later
    0,  // will store the shared library later
  }
};

static const type_support_map_t _BasicMotion_Result_message_typesupport_map = {
  2,
  "uv_msgs",
  &_BasicMotion_Result_message_typesupport_ids.typesupport_identifier[0],
  &_BasicMotion_Result_message_typesupport_symbol_names.symbol_name[0],
  &_BasicMotion_Result_message_typesupport_data.data[0],
};

static const rosidl_message_type_support_t BasicMotion_Result_message_type_support_handle = {
  ::rosidl_typesupport_cpp::typesupport_identifier,
  reinterpret_cast<const type_support_map_t *>(&_BasicMotion_Result_message_typesupport_map),
  ::rosidl_typesupport_cpp::get_message_typesupport_handle_function,
  &uv_msgs__action__BasicMotion_Result__get_type_hash,
  &uv_msgs__action__BasicMotion_Result__get_type_description,
  &uv_msgs__action__BasicMotion_Result__get_type_description_sources,
};

}  // namespace rosidl_typesupport_cpp

}  // namespace action

}  // namespace uv_msgs

namespace rosidl_typesupport_cpp
{

template<>
ROSIDL_TYPESUPPORT_CPP_PUBLIC
const rosidl_message_type_support_t *
get_message_type_support_handle<uv_msgs::action::BasicMotion_Result>()
{
  return &::uv_msgs::action::rosidl_typesupport_cpp::BasicMotion_Result_message_type_support_handle;
}

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_CPP_PUBLIC
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_cpp, uv_msgs, action, BasicMotion_Result)() {
  return get_message_type_support_handle<uv_msgs::action::BasicMotion_Result>();
}

#ifdef __cplusplus
}
#endif
}  // namespace rosidl_typesupport_cpp

// already included above
// #include "cstddef"
// already included above
// #include "rosidl_runtime_c/message_type_support_struct.h"
// already included above
// #include "uv_msgs/action/detail/basic_motion__functions.h"
// already included above
// #include "uv_msgs/action/detail/basic_motion__struct.hpp"
// already included above
// #include "rosidl_typesupport_cpp/identifier.hpp"
// already included above
// #include "rosidl_typesupport_cpp/message_type_support.hpp"
// already included above
// #include "rosidl_typesupport_c/type_support_map.h"
// already included above
// #include "rosidl_typesupport_cpp/message_type_support_dispatch.hpp"
// already included above
// #include "rosidl_typesupport_cpp/visibility_control.h"
// already included above
// #include "rosidl_typesupport_interface/macros.h"

namespace uv_msgs
{

namespace action
{

namespace rosidl_typesupport_cpp
{

typedef struct _BasicMotion_Feedback_type_support_ids_t
{
  const char * typesupport_identifier[2];
} _BasicMotion_Feedback_type_support_ids_t;

static const _BasicMotion_Feedback_type_support_ids_t _BasicMotion_Feedback_message_typesupport_ids = {
  {
    "rosidl_typesupport_fastrtps_cpp",  // ::rosidl_typesupport_fastrtps_cpp::typesupport_identifier,
    "rosidl_typesupport_introspection_cpp",  // ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  }
};

typedef struct _BasicMotion_Feedback_type_support_symbol_names_t
{
  const char * symbol_name[2];
} _BasicMotion_Feedback_type_support_symbol_names_t;

#define STRINGIFY_(s) #s
#define STRINGIFY(s) STRINGIFY_(s)

static const _BasicMotion_Feedback_type_support_symbol_names_t _BasicMotion_Feedback_message_typesupport_symbol_names = {
  {
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_cpp, uv_msgs, action, BasicMotion_Feedback)),
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, uv_msgs, action, BasicMotion_Feedback)),
  }
};

typedef struct _BasicMotion_Feedback_type_support_data_t
{
  void * data[2];
} _BasicMotion_Feedback_type_support_data_t;

static _BasicMotion_Feedback_type_support_data_t _BasicMotion_Feedback_message_typesupport_data = {
  {
    0,  // will store the shared library later
    0,  // will store the shared library later
  }
};

static const type_support_map_t _BasicMotion_Feedback_message_typesupport_map = {
  2,
  "uv_msgs",
  &_BasicMotion_Feedback_message_typesupport_ids.typesupport_identifier[0],
  &_BasicMotion_Feedback_message_typesupport_symbol_names.symbol_name[0],
  &_BasicMotion_Feedback_message_typesupport_data.data[0],
};

static const rosidl_message_type_support_t BasicMotion_Feedback_message_type_support_handle = {
  ::rosidl_typesupport_cpp::typesupport_identifier,
  reinterpret_cast<const type_support_map_t *>(&_BasicMotion_Feedback_message_typesupport_map),
  ::rosidl_typesupport_cpp::get_message_typesupport_handle_function,
  &uv_msgs__action__BasicMotion_Feedback__get_type_hash,
  &uv_msgs__action__BasicMotion_Feedback__get_type_description,
  &uv_msgs__action__BasicMotion_Feedback__get_type_description_sources,
};

}  // namespace rosidl_typesupport_cpp

}  // namespace action

}  // namespace uv_msgs

namespace rosidl_typesupport_cpp
{

template<>
ROSIDL_TYPESUPPORT_CPP_PUBLIC
const rosidl_message_type_support_t *
get_message_type_support_handle<uv_msgs::action::BasicMotion_Feedback>()
{
  return &::uv_msgs::action::rosidl_typesupport_cpp::BasicMotion_Feedback_message_type_support_handle;
}

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_CPP_PUBLIC
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_cpp, uv_msgs, action, BasicMotion_Feedback)() {
  return get_message_type_support_handle<uv_msgs::action::BasicMotion_Feedback>();
}

#ifdef __cplusplus
}
#endif
}  // namespace rosidl_typesupport_cpp

// already included above
// #include "cstddef"
// already included above
// #include "rosidl_runtime_c/message_type_support_struct.h"
// already included above
// #include "uv_msgs/action/detail/basic_motion__functions.h"
// already included above
// #include "uv_msgs/action/detail/basic_motion__struct.hpp"
// already included above
// #include "rosidl_typesupport_cpp/identifier.hpp"
// already included above
// #include "rosidl_typesupport_cpp/message_type_support.hpp"
// already included above
// #include "rosidl_typesupport_c/type_support_map.h"
// already included above
// #include "rosidl_typesupport_cpp/message_type_support_dispatch.hpp"
// already included above
// #include "rosidl_typesupport_cpp/visibility_control.h"
// already included above
// #include "rosidl_typesupport_interface/macros.h"

namespace uv_msgs
{

namespace action
{

namespace rosidl_typesupport_cpp
{

typedef struct _BasicMotion_SendGoal_Request_type_support_ids_t
{
  const char * typesupport_identifier[2];
} _BasicMotion_SendGoal_Request_type_support_ids_t;

static const _BasicMotion_SendGoal_Request_type_support_ids_t _BasicMotion_SendGoal_Request_message_typesupport_ids = {
  {
    "rosidl_typesupport_fastrtps_cpp",  // ::rosidl_typesupport_fastrtps_cpp::typesupport_identifier,
    "rosidl_typesupport_introspection_cpp",  // ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  }
};

typedef struct _BasicMotion_SendGoal_Request_type_support_symbol_names_t
{
  const char * symbol_name[2];
} _BasicMotion_SendGoal_Request_type_support_symbol_names_t;

#define STRINGIFY_(s) #s
#define STRINGIFY(s) STRINGIFY_(s)

static const _BasicMotion_SendGoal_Request_type_support_symbol_names_t _BasicMotion_SendGoal_Request_message_typesupport_symbol_names = {
  {
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_cpp, uv_msgs, action, BasicMotion_SendGoal_Request)),
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, uv_msgs, action, BasicMotion_SendGoal_Request)),
  }
};

typedef struct _BasicMotion_SendGoal_Request_type_support_data_t
{
  void * data[2];
} _BasicMotion_SendGoal_Request_type_support_data_t;

static _BasicMotion_SendGoal_Request_type_support_data_t _BasicMotion_SendGoal_Request_message_typesupport_data = {
  {
    0,  // will store the shared library later
    0,  // will store the shared library later
  }
};

static const type_support_map_t _BasicMotion_SendGoal_Request_message_typesupport_map = {
  2,
  "uv_msgs",
  &_BasicMotion_SendGoal_Request_message_typesupport_ids.typesupport_identifier[0],
  &_BasicMotion_SendGoal_Request_message_typesupport_symbol_names.symbol_name[0],
  &_BasicMotion_SendGoal_Request_message_typesupport_data.data[0],
};

static const rosidl_message_type_support_t BasicMotion_SendGoal_Request_message_type_support_handle = {
  ::rosidl_typesupport_cpp::typesupport_identifier,
  reinterpret_cast<const type_support_map_t *>(&_BasicMotion_SendGoal_Request_message_typesupport_map),
  ::rosidl_typesupport_cpp::get_message_typesupport_handle_function,
  &uv_msgs__action__BasicMotion_SendGoal_Request__get_type_hash,
  &uv_msgs__action__BasicMotion_SendGoal_Request__get_type_description,
  &uv_msgs__action__BasicMotion_SendGoal_Request__get_type_description_sources,
};

}  // namespace rosidl_typesupport_cpp

}  // namespace action

}  // namespace uv_msgs

namespace rosidl_typesupport_cpp
{

template<>
ROSIDL_TYPESUPPORT_CPP_PUBLIC
const rosidl_message_type_support_t *
get_message_type_support_handle<uv_msgs::action::BasicMotion_SendGoal_Request>()
{
  return &::uv_msgs::action::rosidl_typesupport_cpp::BasicMotion_SendGoal_Request_message_type_support_handle;
}

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_CPP_PUBLIC
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_cpp, uv_msgs, action, BasicMotion_SendGoal_Request)() {
  return get_message_type_support_handle<uv_msgs::action::BasicMotion_SendGoal_Request>();
}

#ifdef __cplusplus
}
#endif
}  // namespace rosidl_typesupport_cpp

// already included above
// #include "cstddef"
// already included above
// #include "rosidl_runtime_c/message_type_support_struct.h"
// already included above
// #include "uv_msgs/action/detail/basic_motion__functions.h"
// already included above
// #include "uv_msgs/action/detail/basic_motion__struct.hpp"
// already included above
// #include "rosidl_typesupport_cpp/identifier.hpp"
// already included above
// #include "rosidl_typesupport_cpp/message_type_support.hpp"
// already included above
// #include "rosidl_typesupport_c/type_support_map.h"
// already included above
// #include "rosidl_typesupport_cpp/message_type_support_dispatch.hpp"
// already included above
// #include "rosidl_typesupport_cpp/visibility_control.h"
// already included above
// #include "rosidl_typesupport_interface/macros.h"

namespace uv_msgs
{

namespace action
{

namespace rosidl_typesupport_cpp
{

typedef struct _BasicMotion_SendGoal_Response_type_support_ids_t
{
  const char * typesupport_identifier[2];
} _BasicMotion_SendGoal_Response_type_support_ids_t;

static const _BasicMotion_SendGoal_Response_type_support_ids_t _BasicMotion_SendGoal_Response_message_typesupport_ids = {
  {
    "rosidl_typesupport_fastrtps_cpp",  // ::rosidl_typesupport_fastrtps_cpp::typesupport_identifier,
    "rosidl_typesupport_introspection_cpp",  // ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  }
};

typedef struct _BasicMotion_SendGoal_Response_type_support_symbol_names_t
{
  const char * symbol_name[2];
} _BasicMotion_SendGoal_Response_type_support_symbol_names_t;

#define STRINGIFY_(s) #s
#define STRINGIFY(s) STRINGIFY_(s)

static const _BasicMotion_SendGoal_Response_type_support_symbol_names_t _BasicMotion_SendGoal_Response_message_typesupport_symbol_names = {
  {
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_cpp, uv_msgs, action, BasicMotion_SendGoal_Response)),
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, uv_msgs, action, BasicMotion_SendGoal_Response)),
  }
};

typedef struct _BasicMotion_SendGoal_Response_type_support_data_t
{
  void * data[2];
} _BasicMotion_SendGoal_Response_type_support_data_t;

static _BasicMotion_SendGoal_Response_type_support_data_t _BasicMotion_SendGoal_Response_message_typesupport_data = {
  {
    0,  // will store the shared library later
    0,  // will store the shared library later
  }
};

static const type_support_map_t _BasicMotion_SendGoal_Response_message_typesupport_map = {
  2,
  "uv_msgs",
  &_BasicMotion_SendGoal_Response_message_typesupport_ids.typesupport_identifier[0],
  &_BasicMotion_SendGoal_Response_message_typesupport_symbol_names.symbol_name[0],
  &_BasicMotion_SendGoal_Response_message_typesupport_data.data[0],
};

static const rosidl_message_type_support_t BasicMotion_SendGoal_Response_message_type_support_handle = {
  ::rosidl_typesupport_cpp::typesupport_identifier,
  reinterpret_cast<const type_support_map_t *>(&_BasicMotion_SendGoal_Response_message_typesupport_map),
  ::rosidl_typesupport_cpp::get_message_typesupport_handle_function,
  &uv_msgs__action__BasicMotion_SendGoal_Response__get_type_hash,
  &uv_msgs__action__BasicMotion_SendGoal_Response__get_type_description,
  &uv_msgs__action__BasicMotion_SendGoal_Response__get_type_description_sources,
};

}  // namespace rosidl_typesupport_cpp

}  // namespace action

}  // namespace uv_msgs

namespace rosidl_typesupport_cpp
{

template<>
ROSIDL_TYPESUPPORT_CPP_PUBLIC
const rosidl_message_type_support_t *
get_message_type_support_handle<uv_msgs::action::BasicMotion_SendGoal_Response>()
{
  return &::uv_msgs::action::rosidl_typesupport_cpp::BasicMotion_SendGoal_Response_message_type_support_handle;
}

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_CPP_PUBLIC
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_cpp, uv_msgs, action, BasicMotion_SendGoal_Response)() {
  return get_message_type_support_handle<uv_msgs::action::BasicMotion_SendGoal_Response>();
}

#ifdef __cplusplus
}
#endif
}  // namespace rosidl_typesupport_cpp

// already included above
// #include "cstddef"
// already included above
// #include "rosidl_runtime_c/message_type_support_struct.h"
// already included above
// #include "uv_msgs/action/detail/basic_motion__functions.h"
// already included above
// #include "uv_msgs/action/detail/basic_motion__struct.hpp"
// already included above
// #include "rosidl_typesupport_cpp/identifier.hpp"
// already included above
// #include "rosidl_typesupport_cpp/message_type_support.hpp"
// already included above
// #include "rosidl_typesupport_c/type_support_map.h"
// already included above
// #include "rosidl_typesupport_cpp/message_type_support_dispatch.hpp"
// already included above
// #include "rosidl_typesupport_cpp/visibility_control.h"
// already included above
// #include "rosidl_typesupport_interface/macros.h"

namespace uv_msgs
{

namespace action
{

namespace rosidl_typesupport_cpp
{

typedef struct _BasicMotion_SendGoal_Event_type_support_ids_t
{
  const char * typesupport_identifier[2];
} _BasicMotion_SendGoal_Event_type_support_ids_t;

static const _BasicMotion_SendGoal_Event_type_support_ids_t _BasicMotion_SendGoal_Event_message_typesupport_ids = {
  {
    "rosidl_typesupport_fastrtps_cpp",  // ::rosidl_typesupport_fastrtps_cpp::typesupport_identifier,
    "rosidl_typesupport_introspection_cpp",  // ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  }
};

typedef struct _BasicMotion_SendGoal_Event_type_support_symbol_names_t
{
  const char * symbol_name[2];
} _BasicMotion_SendGoal_Event_type_support_symbol_names_t;

#define STRINGIFY_(s) #s
#define STRINGIFY(s) STRINGIFY_(s)

static const _BasicMotion_SendGoal_Event_type_support_symbol_names_t _BasicMotion_SendGoal_Event_message_typesupport_symbol_names = {
  {
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_cpp, uv_msgs, action, BasicMotion_SendGoal_Event)),
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, uv_msgs, action, BasicMotion_SendGoal_Event)),
  }
};

typedef struct _BasicMotion_SendGoal_Event_type_support_data_t
{
  void * data[2];
} _BasicMotion_SendGoal_Event_type_support_data_t;

static _BasicMotion_SendGoal_Event_type_support_data_t _BasicMotion_SendGoal_Event_message_typesupport_data = {
  {
    0,  // will store the shared library later
    0,  // will store the shared library later
  }
};

static const type_support_map_t _BasicMotion_SendGoal_Event_message_typesupport_map = {
  2,
  "uv_msgs",
  &_BasicMotion_SendGoal_Event_message_typesupport_ids.typesupport_identifier[0],
  &_BasicMotion_SendGoal_Event_message_typesupport_symbol_names.symbol_name[0],
  &_BasicMotion_SendGoal_Event_message_typesupport_data.data[0],
};

static const rosidl_message_type_support_t BasicMotion_SendGoal_Event_message_type_support_handle = {
  ::rosidl_typesupport_cpp::typesupport_identifier,
  reinterpret_cast<const type_support_map_t *>(&_BasicMotion_SendGoal_Event_message_typesupport_map),
  ::rosidl_typesupport_cpp::get_message_typesupport_handle_function,
  &uv_msgs__action__BasicMotion_SendGoal_Event__get_type_hash,
  &uv_msgs__action__BasicMotion_SendGoal_Event__get_type_description,
  &uv_msgs__action__BasicMotion_SendGoal_Event__get_type_description_sources,
};

}  // namespace rosidl_typesupport_cpp

}  // namespace action

}  // namespace uv_msgs

namespace rosidl_typesupport_cpp
{

template<>
ROSIDL_TYPESUPPORT_CPP_PUBLIC
const rosidl_message_type_support_t *
get_message_type_support_handle<uv_msgs::action::BasicMotion_SendGoal_Event>()
{
  return &::uv_msgs::action::rosidl_typesupport_cpp::BasicMotion_SendGoal_Event_message_type_support_handle;
}

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_CPP_PUBLIC
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_cpp, uv_msgs, action, BasicMotion_SendGoal_Event)() {
  return get_message_type_support_handle<uv_msgs::action::BasicMotion_SendGoal_Event>();
}

#ifdef __cplusplus
}
#endif
}  // namespace rosidl_typesupport_cpp

// already included above
// #include "cstddef"
#include "rosidl_runtime_c/service_type_support_struct.h"
#include "rosidl_typesupport_cpp/service_type_support.hpp"
// already included above
// #include "uv_msgs/action/detail/basic_motion__struct.hpp"
// already included above
// #include "rosidl_typesupport_cpp/identifier.hpp"
// already included above
// #include "rosidl_typesupport_c/type_support_map.h"
#include "rosidl_typesupport_cpp/service_type_support_dispatch.hpp"
// already included above
// #include "rosidl_typesupport_cpp/visibility_control.h"
// already included above
// #include "rosidl_typesupport_interface/macros.h"

namespace uv_msgs
{

namespace action
{

namespace rosidl_typesupport_cpp
{

typedef struct _BasicMotion_SendGoal_type_support_ids_t
{
  const char * typesupport_identifier[2];
} _BasicMotion_SendGoal_type_support_ids_t;

static const _BasicMotion_SendGoal_type_support_ids_t _BasicMotion_SendGoal_service_typesupport_ids = {
  {
    "rosidl_typesupport_fastrtps_cpp",  // ::rosidl_typesupport_fastrtps_cpp::typesupport_identifier,
    "rosidl_typesupport_introspection_cpp",  // ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  }
};

typedef struct _BasicMotion_SendGoal_type_support_symbol_names_t
{
  const char * symbol_name[2];
} _BasicMotion_SendGoal_type_support_symbol_names_t;
#define STRINGIFY_(s) #s
#define STRINGIFY(s) STRINGIFY_(s)

static const _BasicMotion_SendGoal_type_support_symbol_names_t _BasicMotion_SendGoal_service_typesupport_symbol_names = {
  {
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_SYMBOL_NAME(rosidl_typesupport_fastrtps_cpp, uv_msgs, action, BasicMotion_SendGoal)),
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, uv_msgs, action, BasicMotion_SendGoal)),
  }
};

typedef struct _BasicMotion_SendGoal_type_support_data_t
{
  void * data[2];
} _BasicMotion_SendGoal_type_support_data_t;

static _BasicMotion_SendGoal_type_support_data_t _BasicMotion_SendGoal_service_typesupport_data = {
  {
    0,  // will store the shared library later
    0,  // will store the shared library later
  }
};

static const type_support_map_t _BasicMotion_SendGoal_service_typesupport_map = {
  2,
  "uv_msgs",
  &_BasicMotion_SendGoal_service_typesupport_ids.typesupport_identifier[0],
  &_BasicMotion_SendGoal_service_typesupport_symbol_names.symbol_name[0],
  &_BasicMotion_SendGoal_service_typesupport_data.data[0],
};

static const rosidl_service_type_support_t BasicMotion_SendGoal_service_type_support_handle = {
  ::rosidl_typesupport_cpp::typesupport_identifier,
  reinterpret_cast<const type_support_map_t *>(&_BasicMotion_SendGoal_service_typesupport_map),
  ::rosidl_typesupport_cpp::get_service_typesupport_handle_function,
  ::rosidl_typesupport_cpp::get_message_type_support_handle<uv_msgs::action::BasicMotion_SendGoal_Request>(),
  ::rosidl_typesupport_cpp::get_message_type_support_handle<uv_msgs::action::BasicMotion_SendGoal_Response>(),
  ::rosidl_typesupport_cpp::get_message_type_support_handle<uv_msgs::action::BasicMotion_SendGoal_Event>(),
  &::rosidl_typesupport_cpp::service_create_event_message<uv_msgs::action::BasicMotion_SendGoal>,
  &::rosidl_typesupport_cpp::service_destroy_event_message<uv_msgs::action::BasicMotion_SendGoal>,
  &uv_msgs__action__BasicMotion_SendGoal__get_type_hash,
  &uv_msgs__action__BasicMotion_SendGoal__get_type_description,
  &uv_msgs__action__BasicMotion_SendGoal__get_type_description_sources,
};

}  // namespace rosidl_typesupport_cpp

}  // namespace action

}  // namespace uv_msgs

namespace rosidl_typesupport_cpp
{

template<>
ROSIDL_TYPESUPPORT_CPP_PUBLIC
const rosidl_service_type_support_t *
get_service_type_support_handle<uv_msgs::action::BasicMotion_SendGoal>()
{
  return &::uv_msgs::action::rosidl_typesupport_cpp::BasicMotion_SendGoal_service_type_support_handle;
}

}  // namespace rosidl_typesupport_cpp

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_CPP_PUBLIC
const rosidl_service_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_SYMBOL_NAME(rosidl_typesupport_cpp, uv_msgs, action, BasicMotion_SendGoal)() {
  return ::rosidl_typesupport_cpp::get_service_type_support_handle<uv_msgs::action::BasicMotion_SendGoal>();
}

#ifdef __cplusplus
}
#endif

// already included above
// #include "cstddef"
// already included above
// #include "rosidl_runtime_c/message_type_support_struct.h"
// already included above
// #include "uv_msgs/action/detail/basic_motion__functions.h"
// already included above
// #include "uv_msgs/action/detail/basic_motion__struct.hpp"
// already included above
// #include "rosidl_typesupport_cpp/identifier.hpp"
// already included above
// #include "rosidl_typesupport_cpp/message_type_support.hpp"
// already included above
// #include "rosidl_typesupport_c/type_support_map.h"
// already included above
// #include "rosidl_typesupport_cpp/message_type_support_dispatch.hpp"
// already included above
// #include "rosidl_typesupport_cpp/visibility_control.h"
// already included above
// #include "rosidl_typesupport_interface/macros.h"

namespace uv_msgs
{

namespace action
{

namespace rosidl_typesupport_cpp
{

typedef struct _BasicMotion_GetResult_Request_type_support_ids_t
{
  const char * typesupport_identifier[2];
} _BasicMotion_GetResult_Request_type_support_ids_t;

static const _BasicMotion_GetResult_Request_type_support_ids_t _BasicMotion_GetResult_Request_message_typesupport_ids = {
  {
    "rosidl_typesupport_fastrtps_cpp",  // ::rosidl_typesupport_fastrtps_cpp::typesupport_identifier,
    "rosidl_typesupport_introspection_cpp",  // ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  }
};

typedef struct _BasicMotion_GetResult_Request_type_support_symbol_names_t
{
  const char * symbol_name[2];
} _BasicMotion_GetResult_Request_type_support_symbol_names_t;

#define STRINGIFY_(s) #s
#define STRINGIFY(s) STRINGIFY_(s)

static const _BasicMotion_GetResult_Request_type_support_symbol_names_t _BasicMotion_GetResult_Request_message_typesupport_symbol_names = {
  {
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_cpp, uv_msgs, action, BasicMotion_GetResult_Request)),
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, uv_msgs, action, BasicMotion_GetResult_Request)),
  }
};

typedef struct _BasicMotion_GetResult_Request_type_support_data_t
{
  void * data[2];
} _BasicMotion_GetResult_Request_type_support_data_t;

static _BasicMotion_GetResult_Request_type_support_data_t _BasicMotion_GetResult_Request_message_typesupport_data = {
  {
    0,  // will store the shared library later
    0,  // will store the shared library later
  }
};

static const type_support_map_t _BasicMotion_GetResult_Request_message_typesupport_map = {
  2,
  "uv_msgs",
  &_BasicMotion_GetResult_Request_message_typesupport_ids.typesupport_identifier[0],
  &_BasicMotion_GetResult_Request_message_typesupport_symbol_names.symbol_name[0],
  &_BasicMotion_GetResult_Request_message_typesupport_data.data[0],
};

static const rosidl_message_type_support_t BasicMotion_GetResult_Request_message_type_support_handle = {
  ::rosidl_typesupport_cpp::typesupport_identifier,
  reinterpret_cast<const type_support_map_t *>(&_BasicMotion_GetResult_Request_message_typesupport_map),
  ::rosidl_typesupport_cpp::get_message_typesupport_handle_function,
  &uv_msgs__action__BasicMotion_GetResult_Request__get_type_hash,
  &uv_msgs__action__BasicMotion_GetResult_Request__get_type_description,
  &uv_msgs__action__BasicMotion_GetResult_Request__get_type_description_sources,
};

}  // namespace rosidl_typesupport_cpp

}  // namespace action

}  // namespace uv_msgs

namespace rosidl_typesupport_cpp
{

template<>
ROSIDL_TYPESUPPORT_CPP_PUBLIC
const rosidl_message_type_support_t *
get_message_type_support_handle<uv_msgs::action::BasicMotion_GetResult_Request>()
{
  return &::uv_msgs::action::rosidl_typesupport_cpp::BasicMotion_GetResult_Request_message_type_support_handle;
}

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_CPP_PUBLIC
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_cpp, uv_msgs, action, BasicMotion_GetResult_Request)() {
  return get_message_type_support_handle<uv_msgs::action::BasicMotion_GetResult_Request>();
}

#ifdef __cplusplus
}
#endif
}  // namespace rosidl_typesupport_cpp

// already included above
// #include "cstddef"
// already included above
// #include "rosidl_runtime_c/message_type_support_struct.h"
// already included above
// #include "uv_msgs/action/detail/basic_motion__functions.h"
// already included above
// #include "uv_msgs/action/detail/basic_motion__struct.hpp"
// already included above
// #include "rosidl_typesupport_cpp/identifier.hpp"
// already included above
// #include "rosidl_typesupport_cpp/message_type_support.hpp"
// already included above
// #include "rosidl_typesupport_c/type_support_map.h"
// already included above
// #include "rosidl_typesupport_cpp/message_type_support_dispatch.hpp"
// already included above
// #include "rosidl_typesupport_cpp/visibility_control.h"
// already included above
// #include "rosidl_typesupport_interface/macros.h"

namespace uv_msgs
{

namespace action
{

namespace rosidl_typesupport_cpp
{

typedef struct _BasicMotion_GetResult_Response_type_support_ids_t
{
  const char * typesupport_identifier[2];
} _BasicMotion_GetResult_Response_type_support_ids_t;

static const _BasicMotion_GetResult_Response_type_support_ids_t _BasicMotion_GetResult_Response_message_typesupport_ids = {
  {
    "rosidl_typesupport_fastrtps_cpp",  // ::rosidl_typesupport_fastrtps_cpp::typesupport_identifier,
    "rosidl_typesupport_introspection_cpp",  // ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  }
};

typedef struct _BasicMotion_GetResult_Response_type_support_symbol_names_t
{
  const char * symbol_name[2];
} _BasicMotion_GetResult_Response_type_support_symbol_names_t;

#define STRINGIFY_(s) #s
#define STRINGIFY(s) STRINGIFY_(s)

static const _BasicMotion_GetResult_Response_type_support_symbol_names_t _BasicMotion_GetResult_Response_message_typesupport_symbol_names = {
  {
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_cpp, uv_msgs, action, BasicMotion_GetResult_Response)),
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, uv_msgs, action, BasicMotion_GetResult_Response)),
  }
};

typedef struct _BasicMotion_GetResult_Response_type_support_data_t
{
  void * data[2];
} _BasicMotion_GetResult_Response_type_support_data_t;

static _BasicMotion_GetResult_Response_type_support_data_t _BasicMotion_GetResult_Response_message_typesupport_data = {
  {
    0,  // will store the shared library later
    0,  // will store the shared library later
  }
};

static const type_support_map_t _BasicMotion_GetResult_Response_message_typesupport_map = {
  2,
  "uv_msgs",
  &_BasicMotion_GetResult_Response_message_typesupport_ids.typesupport_identifier[0],
  &_BasicMotion_GetResult_Response_message_typesupport_symbol_names.symbol_name[0],
  &_BasicMotion_GetResult_Response_message_typesupport_data.data[0],
};

static const rosidl_message_type_support_t BasicMotion_GetResult_Response_message_type_support_handle = {
  ::rosidl_typesupport_cpp::typesupport_identifier,
  reinterpret_cast<const type_support_map_t *>(&_BasicMotion_GetResult_Response_message_typesupport_map),
  ::rosidl_typesupport_cpp::get_message_typesupport_handle_function,
  &uv_msgs__action__BasicMotion_GetResult_Response__get_type_hash,
  &uv_msgs__action__BasicMotion_GetResult_Response__get_type_description,
  &uv_msgs__action__BasicMotion_GetResult_Response__get_type_description_sources,
};

}  // namespace rosidl_typesupport_cpp

}  // namespace action

}  // namespace uv_msgs

namespace rosidl_typesupport_cpp
{

template<>
ROSIDL_TYPESUPPORT_CPP_PUBLIC
const rosidl_message_type_support_t *
get_message_type_support_handle<uv_msgs::action::BasicMotion_GetResult_Response>()
{
  return &::uv_msgs::action::rosidl_typesupport_cpp::BasicMotion_GetResult_Response_message_type_support_handle;
}

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_CPP_PUBLIC
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_cpp, uv_msgs, action, BasicMotion_GetResult_Response)() {
  return get_message_type_support_handle<uv_msgs::action::BasicMotion_GetResult_Response>();
}

#ifdef __cplusplus
}
#endif
}  // namespace rosidl_typesupport_cpp

// already included above
// #include "cstddef"
// already included above
// #include "rosidl_runtime_c/message_type_support_struct.h"
// already included above
// #include "uv_msgs/action/detail/basic_motion__functions.h"
// already included above
// #include "uv_msgs/action/detail/basic_motion__struct.hpp"
// already included above
// #include "rosidl_typesupport_cpp/identifier.hpp"
// already included above
// #include "rosidl_typesupport_cpp/message_type_support.hpp"
// already included above
// #include "rosidl_typesupport_c/type_support_map.h"
// already included above
// #include "rosidl_typesupport_cpp/message_type_support_dispatch.hpp"
// already included above
// #include "rosidl_typesupport_cpp/visibility_control.h"
// already included above
// #include "rosidl_typesupport_interface/macros.h"

namespace uv_msgs
{

namespace action
{

namespace rosidl_typesupport_cpp
{

typedef struct _BasicMotion_GetResult_Event_type_support_ids_t
{
  const char * typesupport_identifier[2];
} _BasicMotion_GetResult_Event_type_support_ids_t;

static const _BasicMotion_GetResult_Event_type_support_ids_t _BasicMotion_GetResult_Event_message_typesupport_ids = {
  {
    "rosidl_typesupport_fastrtps_cpp",  // ::rosidl_typesupport_fastrtps_cpp::typesupport_identifier,
    "rosidl_typesupport_introspection_cpp",  // ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  }
};

typedef struct _BasicMotion_GetResult_Event_type_support_symbol_names_t
{
  const char * symbol_name[2];
} _BasicMotion_GetResult_Event_type_support_symbol_names_t;

#define STRINGIFY_(s) #s
#define STRINGIFY(s) STRINGIFY_(s)

static const _BasicMotion_GetResult_Event_type_support_symbol_names_t _BasicMotion_GetResult_Event_message_typesupport_symbol_names = {
  {
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_cpp, uv_msgs, action, BasicMotion_GetResult_Event)),
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, uv_msgs, action, BasicMotion_GetResult_Event)),
  }
};

typedef struct _BasicMotion_GetResult_Event_type_support_data_t
{
  void * data[2];
} _BasicMotion_GetResult_Event_type_support_data_t;

static _BasicMotion_GetResult_Event_type_support_data_t _BasicMotion_GetResult_Event_message_typesupport_data = {
  {
    0,  // will store the shared library later
    0,  // will store the shared library later
  }
};

static const type_support_map_t _BasicMotion_GetResult_Event_message_typesupport_map = {
  2,
  "uv_msgs",
  &_BasicMotion_GetResult_Event_message_typesupport_ids.typesupport_identifier[0],
  &_BasicMotion_GetResult_Event_message_typesupport_symbol_names.symbol_name[0],
  &_BasicMotion_GetResult_Event_message_typesupport_data.data[0],
};

static const rosidl_message_type_support_t BasicMotion_GetResult_Event_message_type_support_handle = {
  ::rosidl_typesupport_cpp::typesupport_identifier,
  reinterpret_cast<const type_support_map_t *>(&_BasicMotion_GetResult_Event_message_typesupport_map),
  ::rosidl_typesupport_cpp::get_message_typesupport_handle_function,
  &uv_msgs__action__BasicMotion_GetResult_Event__get_type_hash,
  &uv_msgs__action__BasicMotion_GetResult_Event__get_type_description,
  &uv_msgs__action__BasicMotion_GetResult_Event__get_type_description_sources,
};

}  // namespace rosidl_typesupport_cpp

}  // namespace action

}  // namespace uv_msgs

namespace rosidl_typesupport_cpp
{

template<>
ROSIDL_TYPESUPPORT_CPP_PUBLIC
const rosidl_message_type_support_t *
get_message_type_support_handle<uv_msgs::action::BasicMotion_GetResult_Event>()
{
  return &::uv_msgs::action::rosidl_typesupport_cpp::BasicMotion_GetResult_Event_message_type_support_handle;
}

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_CPP_PUBLIC
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_cpp, uv_msgs, action, BasicMotion_GetResult_Event)() {
  return get_message_type_support_handle<uv_msgs::action::BasicMotion_GetResult_Event>();
}

#ifdef __cplusplus
}
#endif
}  // namespace rosidl_typesupport_cpp

// already included above
// #include "cstddef"
// already included above
// #include "rosidl_runtime_c/service_type_support_struct.h"
// already included above
// #include "rosidl_typesupport_cpp/service_type_support.hpp"
// already included above
// #include "uv_msgs/action/detail/basic_motion__struct.hpp"
// already included above
// #include "rosidl_typesupport_cpp/identifier.hpp"
// already included above
// #include "rosidl_typesupport_c/type_support_map.h"
// already included above
// #include "rosidl_typesupport_cpp/service_type_support_dispatch.hpp"
// already included above
// #include "rosidl_typesupport_cpp/visibility_control.h"
// already included above
// #include "rosidl_typesupport_interface/macros.h"

namespace uv_msgs
{

namespace action
{

namespace rosidl_typesupport_cpp
{

typedef struct _BasicMotion_GetResult_type_support_ids_t
{
  const char * typesupport_identifier[2];
} _BasicMotion_GetResult_type_support_ids_t;

static const _BasicMotion_GetResult_type_support_ids_t _BasicMotion_GetResult_service_typesupport_ids = {
  {
    "rosidl_typesupport_fastrtps_cpp",  // ::rosidl_typesupport_fastrtps_cpp::typesupport_identifier,
    "rosidl_typesupport_introspection_cpp",  // ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  }
};

typedef struct _BasicMotion_GetResult_type_support_symbol_names_t
{
  const char * symbol_name[2];
} _BasicMotion_GetResult_type_support_symbol_names_t;
#define STRINGIFY_(s) #s
#define STRINGIFY(s) STRINGIFY_(s)

static const _BasicMotion_GetResult_type_support_symbol_names_t _BasicMotion_GetResult_service_typesupport_symbol_names = {
  {
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_SYMBOL_NAME(rosidl_typesupport_fastrtps_cpp, uv_msgs, action, BasicMotion_GetResult)),
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, uv_msgs, action, BasicMotion_GetResult)),
  }
};

typedef struct _BasicMotion_GetResult_type_support_data_t
{
  void * data[2];
} _BasicMotion_GetResult_type_support_data_t;

static _BasicMotion_GetResult_type_support_data_t _BasicMotion_GetResult_service_typesupport_data = {
  {
    0,  // will store the shared library later
    0,  // will store the shared library later
  }
};

static const type_support_map_t _BasicMotion_GetResult_service_typesupport_map = {
  2,
  "uv_msgs",
  &_BasicMotion_GetResult_service_typesupport_ids.typesupport_identifier[0],
  &_BasicMotion_GetResult_service_typesupport_symbol_names.symbol_name[0],
  &_BasicMotion_GetResult_service_typesupport_data.data[0],
};

static const rosidl_service_type_support_t BasicMotion_GetResult_service_type_support_handle = {
  ::rosidl_typesupport_cpp::typesupport_identifier,
  reinterpret_cast<const type_support_map_t *>(&_BasicMotion_GetResult_service_typesupport_map),
  ::rosidl_typesupport_cpp::get_service_typesupport_handle_function,
  ::rosidl_typesupport_cpp::get_message_type_support_handle<uv_msgs::action::BasicMotion_GetResult_Request>(),
  ::rosidl_typesupport_cpp::get_message_type_support_handle<uv_msgs::action::BasicMotion_GetResult_Response>(),
  ::rosidl_typesupport_cpp::get_message_type_support_handle<uv_msgs::action::BasicMotion_GetResult_Event>(),
  &::rosidl_typesupport_cpp::service_create_event_message<uv_msgs::action::BasicMotion_GetResult>,
  &::rosidl_typesupport_cpp::service_destroy_event_message<uv_msgs::action::BasicMotion_GetResult>,
  &uv_msgs__action__BasicMotion_GetResult__get_type_hash,
  &uv_msgs__action__BasicMotion_GetResult__get_type_description,
  &uv_msgs__action__BasicMotion_GetResult__get_type_description_sources,
};

}  // namespace rosidl_typesupport_cpp

}  // namespace action

}  // namespace uv_msgs

namespace rosidl_typesupport_cpp
{

template<>
ROSIDL_TYPESUPPORT_CPP_PUBLIC
const rosidl_service_type_support_t *
get_service_type_support_handle<uv_msgs::action::BasicMotion_GetResult>()
{
  return &::uv_msgs::action::rosidl_typesupport_cpp::BasicMotion_GetResult_service_type_support_handle;
}

}  // namespace rosidl_typesupport_cpp

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_CPP_PUBLIC
const rosidl_service_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_SYMBOL_NAME(rosidl_typesupport_cpp, uv_msgs, action, BasicMotion_GetResult)() {
  return ::rosidl_typesupport_cpp::get_service_type_support_handle<uv_msgs::action::BasicMotion_GetResult>();
}

#ifdef __cplusplus
}
#endif

// already included above
// #include "cstddef"
// already included above
// #include "rosidl_runtime_c/message_type_support_struct.h"
// already included above
// #include "uv_msgs/action/detail/basic_motion__functions.h"
// already included above
// #include "uv_msgs/action/detail/basic_motion__struct.hpp"
// already included above
// #include "rosidl_typesupport_cpp/identifier.hpp"
// already included above
// #include "rosidl_typesupport_cpp/message_type_support.hpp"
// already included above
// #include "rosidl_typesupport_c/type_support_map.h"
// already included above
// #include "rosidl_typesupport_cpp/message_type_support_dispatch.hpp"
// already included above
// #include "rosidl_typesupport_cpp/visibility_control.h"
// already included above
// #include "rosidl_typesupport_interface/macros.h"

namespace uv_msgs
{

namespace action
{

namespace rosidl_typesupport_cpp
{

typedef struct _BasicMotion_FeedbackMessage_type_support_ids_t
{
  const char * typesupport_identifier[2];
} _BasicMotion_FeedbackMessage_type_support_ids_t;

static const _BasicMotion_FeedbackMessage_type_support_ids_t _BasicMotion_FeedbackMessage_message_typesupport_ids = {
  {
    "rosidl_typesupport_fastrtps_cpp",  // ::rosidl_typesupport_fastrtps_cpp::typesupport_identifier,
    "rosidl_typesupport_introspection_cpp",  // ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  }
};

typedef struct _BasicMotion_FeedbackMessage_type_support_symbol_names_t
{
  const char * symbol_name[2];
} _BasicMotion_FeedbackMessage_type_support_symbol_names_t;

#define STRINGIFY_(s) #s
#define STRINGIFY(s) STRINGIFY_(s)

static const _BasicMotion_FeedbackMessage_type_support_symbol_names_t _BasicMotion_FeedbackMessage_message_typesupport_symbol_names = {
  {
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_cpp, uv_msgs, action, BasicMotion_FeedbackMessage)),
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, uv_msgs, action, BasicMotion_FeedbackMessage)),
  }
};

typedef struct _BasicMotion_FeedbackMessage_type_support_data_t
{
  void * data[2];
} _BasicMotion_FeedbackMessage_type_support_data_t;

static _BasicMotion_FeedbackMessage_type_support_data_t _BasicMotion_FeedbackMessage_message_typesupport_data = {
  {
    0,  // will store the shared library later
    0,  // will store the shared library later
  }
};

static const type_support_map_t _BasicMotion_FeedbackMessage_message_typesupport_map = {
  2,
  "uv_msgs",
  &_BasicMotion_FeedbackMessage_message_typesupport_ids.typesupport_identifier[0],
  &_BasicMotion_FeedbackMessage_message_typesupport_symbol_names.symbol_name[0],
  &_BasicMotion_FeedbackMessage_message_typesupport_data.data[0],
};

static const rosidl_message_type_support_t BasicMotion_FeedbackMessage_message_type_support_handle = {
  ::rosidl_typesupport_cpp::typesupport_identifier,
  reinterpret_cast<const type_support_map_t *>(&_BasicMotion_FeedbackMessage_message_typesupport_map),
  ::rosidl_typesupport_cpp::get_message_typesupport_handle_function,
  &uv_msgs__action__BasicMotion_FeedbackMessage__get_type_hash,
  &uv_msgs__action__BasicMotion_FeedbackMessage__get_type_description,
  &uv_msgs__action__BasicMotion_FeedbackMessage__get_type_description_sources,
};

}  // namespace rosidl_typesupport_cpp

}  // namespace action

}  // namespace uv_msgs

namespace rosidl_typesupport_cpp
{

template<>
ROSIDL_TYPESUPPORT_CPP_PUBLIC
const rosidl_message_type_support_t *
get_message_type_support_handle<uv_msgs::action::BasicMotion_FeedbackMessage>()
{
  return &::uv_msgs::action::rosidl_typesupport_cpp::BasicMotion_FeedbackMessage_message_type_support_handle;
}

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_CPP_PUBLIC
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_cpp, uv_msgs, action, BasicMotion_FeedbackMessage)() {
  return get_message_type_support_handle<uv_msgs::action::BasicMotion_FeedbackMessage>();
}

#ifdef __cplusplus
}
#endif
}  // namespace rosidl_typesupport_cpp

#include "action_msgs/msg/goal_status_array.hpp"
#include "action_msgs/srv/cancel_goal.hpp"
// already included above
// #include "uv_msgs/action/detail/basic_motion__struct.hpp"
// already included above
// #include "rosidl_typesupport_cpp/visibility_control.h"
#include "rosidl_runtime_c/action_type_support_struct.h"
#include "rosidl_typesupport_cpp/action_type_support.hpp"
// already included above
// #include "rosidl_typesupport_cpp/message_type_support.hpp"
// already included above
// #include "rosidl_typesupport_cpp/service_type_support.hpp"

namespace uv_msgs
{

namespace action
{

namespace rosidl_typesupport_cpp
{

static rosidl_action_type_support_t BasicMotion_action_type_support_handle = {
  NULL, NULL, NULL, NULL, NULL,
  &uv_msgs__action__BasicMotion__get_type_hash,
  &uv_msgs__action__BasicMotion__get_type_description,
  &uv_msgs__action__BasicMotion__get_type_description_sources,
};

}  // namespace rosidl_typesupport_cpp

}  // namespace action

}  // namespace uv_msgs

namespace rosidl_typesupport_cpp
{

template<>
ROSIDL_TYPESUPPORT_CPP_PUBLIC
const rosidl_action_type_support_t *
get_action_type_support_handle<uv_msgs::action::BasicMotion>()
{
  using ::uv_msgs::action::rosidl_typesupport_cpp::BasicMotion_action_type_support_handle;
  // Thread-safe by always writing the same values to the static struct
  BasicMotion_action_type_support_handle.goal_service_type_support = get_service_type_support_handle<::uv_msgs::action::BasicMotion::Impl::SendGoalService>();
  BasicMotion_action_type_support_handle.result_service_type_support = get_service_type_support_handle<::uv_msgs::action::BasicMotion::Impl::GetResultService>();
  BasicMotion_action_type_support_handle.cancel_service_type_support = get_service_type_support_handle<::uv_msgs::action::BasicMotion::Impl::CancelGoalService>();
  BasicMotion_action_type_support_handle.feedback_message_type_support = get_message_type_support_handle<::uv_msgs::action::BasicMotion::Impl::FeedbackMessage>();
  BasicMotion_action_type_support_handle.status_message_type_support = get_message_type_support_handle<::uv_msgs::action::BasicMotion::Impl::GoalStatusMessage>();
  return &BasicMotion_action_type_support_handle;
}

}  // namespace rosidl_typesupport_cpp

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_CPP_PUBLIC
const rosidl_action_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__ACTION_SYMBOL_NAME(rosidl_typesupport_cpp, uv_msgs, action, BasicMotion)() {
  return ::rosidl_typesupport_cpp::get_action_type_support_handle<uv_msgs::action::BasicMotion>();
}

#ifdef __cplusplus
}
#endif
