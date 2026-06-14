// generated from rosidl_typesupport_c/resource/idl__type_support.cpp.em
// with input from zit6_interfaces:srv/UpdateParams.idl
// generated code does not contain a copyright notice

#include "cstddef"
#include "rosidl_runtime_c/message_type_support_struct.h"
#include "zit6_interfaces/srv/detail/update_params__struct.h"
#include "zit6_interfaces/srv/detail/update_params__type_support.h"
#include "zit6_interfaces/srv/detail/update_params__functions.h"
#include "rosidl_typesupport_c/identifier.h"
#include "rosidl_typesupport_c/message_type_support_dispatch.h"
#include "rosidl_typesupport_c/type_support_map.h"
#include "rosidl_typesupport_c/visibility_control.h"
#include "rosidl_typesupport_interface/macros.h"

namespace zit6_interfaces
{

namespace srv
{

namespace rosidl_typesupport_c
{

typedef struct _UpdateParams_Request_type_support_ids_t
{
  const char * typesupport_identifier[2];
} _UpdateParams_Request_type_support_ids_t;

static const _UpdateParams_Request_type_support_ids_t _UpdateParams_Request_message_typesupport_ids = {
  {
    "rosidl_typesupport_fastrtps_c",  // ::rosidl_typesupport_fastrtps_c::typesupport_identifier,
    "rosidl_typesupport_introspection_c",  // ::rosidl_typesupport_introspection_c::typesupport_identifier,
  }
};

typedef struct _UpdateParams_Request_type_support_symbol_names_t
{
  const char * symbol_name[2];
} _UpdateParams_Request_type_support_symbol_names_t;

#define STRINGIFY_(s) #s
#define STRINGIFY(s) STRINGIFY_(s)

static const _UpdateParams_Request_type_support_symbol_names_t _UpdateParams_Request_message_typesupport_symbol_names = {
  {
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, zit6_interfaces, srv, UpdateParams_Request)),
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, zit6_interfaces, srv, UpdateParams_Request)),
  }
};

typedef struct _UpdateParams_Request_type_support_data_t
{
  void * data[2];
} _UpdateParams_Request_type_support_data_t;

static _UpdateParams_Request_type_support_data_t _UpdateParams_Request_message_typesupport_data = {
  {
    0,  // will store the shared library later
    0,  // will store the shared library later
  }
};

static const type_support_map_t _UpdateParams_Request_message_typesupport_map = {
  2,
  "zit6_interfaces",
  &_UpdateParams_Request_message_typesupport_ids.typesupport_identifier[0],
  &_UpdateParams_Request_message_typesupport_symbol_names.symbol_name[0],
  &_UpdateParams_Request_message_typesupport_data.data[0],
};

static const rosidl_message_type_support_t UpdateParams_Request_message_type_support_handle = {
  rosidl_typesupport_c__typesupport_identifier,
  reinterpret_cast<const type_support_map_t *>(&_UpdateParams_Request_message_typesupport_map),
  rosidl_typesupport_c__get_message_typesupport_handle_function,
  &zit6_interfaces__srv__UpdateParams_Request__get_type_hash,
  &zit6_interfaces__srv__UpdateParams_Request__get_type_description,
  &zit6_interfaces__srv__UpdateParams_Request__get_type_description_sources,
};

}  // namespace rosidl_typesupport_c

}  // namespace srv

}  // namespace zit6_interfaces

#ifdef __cplusplus
extern "C"
{
#endif

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_c, zit6_interfaces, srv, UpdateParams_Request)() {
  return &::zit6_interfaces::srv::rosidl_typesupport_c::UpdateParams_Request_message_type_support_handle;
}

#ifdef __cplusplus
}
#endif

// already included above
// #include "cstddef"
// already included above
// #include "rosidl_runtime_c/message_type_support_struct.h"
// already included above
// #include "zit6_interfaces/srv/detail/update_params__struct.h"
// already included above
// #include "zit6_interfaces/srv/detail/update_params__type_support.h"
// already included above
// #include "zit6_interfaces/srv/detail/update_params__functions.h"
// already included above
// #include "rosidl_typesupport_c/identifier.h"
// already included above
// #include "rosidl_typesupport_c/message_type_support_dispatch.h"
// already included above
// #include "rosidl_typesupport_c/type_support_map.h"
// already included above
// #include "rosidl_typesupport_c/visibility_control.h"
// already included above
// #include "rosidl_typesupport_interface/macros.h"

namespace zit6_interfaces
{

namespace srv
{

namespace rosidl_typesupport_c
{

typedef struct _UpdateParams_Response_type_support_ids_t
{
  const char * typesupport_identifier[2];
} _UpdateParams_Response_type_support_ids_t;

static const _UpdateParams_Response_type_support_ids_t _UpdateParams_Response_message_typesupport_ids = {
  {
    "rosidl_typesupport_fastrtps_c",  // ::rosidl_typesupport_fastrtps_c::typesupport_identifier,
    "rosidl_typesupport_introspection_c",  // ::rosidl_typesupport_introspection_c::typesupport_identifier,
  }
};

typedef struct _UpdateParams_Response_type_support_symbol_names_t
{
  const char * symbol_name[2];
} _UpdateParams_Response_type_support_symbol_names_t;

#define STRINGIFY_(s) #s
#define STRINGIFY(s) STRINGIFY_(s)

static const _UpdateParams_Response_type_support_symbol_names_t _UpdateParams_Response_message_typesupport_symbol_names = {
  {
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, zit6_interfaces, srv, UpdateParams_Response)),
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, zit6_interfaces, srv, UpdateParams_Response)),
  }
};

typedef struct _UpdateParams_Response_type_support_data_t
{
  void * data[2];
} _UpdateParams_Response_type_support_data_t;

static _UpdateParams_Response_type_support_data_t _UpdateParams_Response_message_typesupport_data = {
  {
    0,  // will store the shared library later
    0,  // will store the shared library later
  }
};

static const type_support_map_t _UpdateParams_Response_message_typesupport_map = {
  2,
  "zit6_interfaces",
  &_UpdateParams_Response_message_typesupport_ids.typesupport_identifier[0],
  &_UpdateParams_Response_message_typesupport_symbol_names.symbol_name[0],
  &_UpdateParams_Response_message_typesupport_data.data[0],
};

static const rosidl_message_type_support_t UpdateParams_Response_message_type_support_handle = {
  rosidl_typesupport_c__typesupport_identifier,
  reinterpret_cast<const type_support_map_t *>(&_UpdateParams_Response_message_typesupport_map),
  rosidl_typesupport_c__get_message_typesupport_handle_function,
  &zit6_interfaces__srv__UpdateParams_Response__get_type_hash,
  &zit6_interfaces__srv__UpdateParams_Response__get_type_description,
  &zit6_interfaces__srv__UpdateParams_Response__get_type_description_sources,
};

}  // namespace rosidl_typesupport_c

}  // namespace srv

}  // namespace zit6_interfaces

#ifdef __cplusplus
extern "C"
{
#endif

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_c, zit6_interfaces, srv, UpdateParams_Response)() {
  return &::zit6_interfaces::srv::rosidl_typesupport_c::UpdateParams_Response_message_type_support_handle;
}

#ifdef __cplusplus
}
#endif

// already included above
// #include "cstddef"
// already included above
// #include "rosidl_runtime_c/message_type_support_struct.h"
// already included above
// #include "zit6_interfaces/srv/detail/update_params__struct.h"
// already included above
// #include "zit6_interfaces/srv/detail/update_params__type_support.h"
// already included above
// #include "zit6_interfaces/srv/detail/update_params__functions.h"
// already included above
// #include "rosidl_typesupport_c/identifier.h"
// already included above
// #include "rosidl_typesupport_c/message_type_support_dispatch.h"
// already included above
// #include "rosidl_typesupport_c/type_support_map.h"
// already included above
// #include "rosidl_typesupport_c/visibility_control.h"
// already included above
// #include "rosidl_typesupport_interface/macros.h"

namespace zit6_interfaces
{

namespace srv
{

namespace rosidl_typesupport_c
{

typedef struct _UpdateParams_Event_type_support_ids_t
{
  const char * typesupport_identifier[2];
} _UpdateParams_Event_type_support_ids_t;

static const _UpdateParams_Event_type_support_ids_t _UpdateParams_Event_message_typesupport_ids = {
  {
    "rosidl_typesupport_fastrtps_c",  // ::rosidl_typesupport_fastrtps_c::typesupport_identifier,
    "rosidl_typesupport_introspection_c",  // ::rosidl_typesupport_introspection_c::typesupport_identifier,
  }
};

typedef struct _UpdateParams_Event_type_support_symbol_names_t
{
  const char * symbol_name[2];
} _UpdateParams_Event_type_support_symbol_names_t;

#define STRINGIFY_(s) #s
#define STRINGIFY(s) STRINGIFY_(s)

static const _UpdateParams_Event_type_support_symbol_names_t _UpdateParams_Event_message_typesupport_symbol_names = {
  {
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, zit6_interfaces, srv, UpdateParams_Event)),
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, zit6_interfaces, srv, UpdateParams_Event)),
  }
};

typedef struct _UpdateParams_Event_type_support_data_t
{
  void * data[2];
} _UpdateParams_Event_type_support_data_t;

static _UpdateParams_Event_type_support_data_t _UpdateParams_Event_message_typesupport_data = {
  {
    0,  // will store the shared library later
    0,  // will store the shared library later
  }
};

static const type_support_map_t _UpdateParams_Event_message_typesupport_map = {
  2,
  "zit6_interfaces",
  &_UpdateParams_Event_message_typesupport_ids.typesupport_identifier[0],
  &_UpdateParams_Event_message_typesupport_symbol_names.symbol_name[0],
  &_UpdateParams_Event_message_typesupport_data.data[0],
};

static const rosidl_message_type_support_t UpdateParams_Event_message_type_support_handle = {
  rosidl_typesupport_c__typesupport_identifier,
  reinterpret_cast<const type_support_map_t *>(&_UpdateParams_Event_message_typesupport_map),
  rosidl_typesupport_c__get_message_typesupport_handle_function,
  &zit6_interfaces__srv__UpdateParams_Event__get_type_hash,
  &zit6_interfaces__srv__UpdateParams_Event__get_type_description,
  &zit6_interfaces__srv__UpdateParams_Event__get_type_description_sources,
};

}  // namespace rosidl_typesupport_c

}  // namespace srv

}  // namespace zit6_interfaces

#ifdef __cplusplus
extern "C"
{
#endif

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_c, zit6_interfaces, srv, UpdateParams_Event)() {
  return &::zit6_interfaces::srv::rosidl_typesupport_c::UpdateParams_Event_message_type_support_handle;
}

#ifdef __cplusplus
}
#endif

// already included above
// #include "cstddef"
#include "rosidl_runtime_c/service_type_support_struct.h"
// already included above
// #include "zit6_interfaces/srv/detail/update_params__type_support.h"
// already included above
// #include "rosidl_typesupport_c/identifier.h"
#include "rosidl_typesupport_c/service_type_support_dispatch.h"
// already included above
// #include "rosidl_typesupport_c/type_support_map.h"
// already included above
// #include "rosidl_typesupport_interface/macros.h"
#include "service_msgs/msg/service_event_info.h"
#include "builtin_interfaces/msg/time.h"

namespace zit6_interfaces
{

namespace srv
{

namespace rosidl_typesupport_c
{
typedef struct _UpdateParams_type_support_ids_t
{
  const char * typesupport_identifier[2];
} _UpdateParams_type_support_ids_t;

static const _UpdateParams_type_support_ids_t _UpdateParams_service_typesupport_ids = {
  {
    "rosidl_typesupport_fastrtps_c",  // ::rosidl_typesupport_fastrtps_c::typesupport_identifier,
    "rosidl_typesupport_introspection_c",  // ::rosidl_typesupport_introspection_c::typesupport_identifier,
  }
};

typedef struct _UpdateParams_type_support_symbol_names_t
{
  const char * symbol_name[2];
} _UpdateParams_type_support_symbol_names_t;

#define STRINGIFY_(s) #s
#define STRINGIFY(s) STRINGIFY_(s)

static const _UpdateParams_type_support_symbol_names_t _UpdateParams_service_typesupport_symbol_names = {
  {
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, zit6_interfaces, srv, UpdateParams)),
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_SYMBOL_NAME(rosidl_typesupport_introspection_c, zit6_interfaces, srv, UpdateParams)),
  }
};

typedef struct _UpdateParams_type_support_data_t
{
  void * data[2];
} _UpdateParams_type_support_data_t;

static _UpdateParams_type_support_data_t _UpdateParams_service_typesupport_data = {
  {
    0,  // will store the shared library later
    0,  // will store the shared library later
  }
};

static const type_support_map_t _UpdateParams_service_typesupport_map = {
  2,
  "zit6_interfaces",
  &_UpdateParams_service_typesupport_ids.typesupport_identifier[0],
  &_UpdateParams_service_typesupport_symbol_names.symbol_name[0],
  &_UpdateParams_service_typesupport_data.data[0],
};

static const rosidl_service_type_support_t UpdateParams_service_type_support_handle = {
  rosidl_typesupport_c__typesupport_identifier,
  reinterpret_cast<const type_support_map_t *>(&_UpdateParams_service_typesupport_map),
  rosidl_typesupport_c__get_service_typesupport_handle_function,
  &UpdateParams_Request_message_type_support_handle,
  &UpdateParams_Response_message_type_support_handle,
  &UpdateParams_Event_message_type_support_handle,
  ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_CREATE_EVENT_MESSAGE_SYMBOL_NAME(
    rosidl_typesupport_c,
    zit6_interfaces,
    srv,
    UpdateParams
  ),
  ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_DESTROY_EVENT_MESSAGE_SYMBOL_NAME(
    rosidl_typesupport_c,
    zit6_interfaces,
    srv,
    UpdateParams
  ),
  &zit6_interfaces__srv__UpdateParams__get_type_hash,
  &zit6_interfaces__srv__UpdateParams__get_type_description,
  &zit6_interfaces__srv__UpdateParams__get_type_description_sources,
};

}  // namespace rosidl_typesupport_c

}  // namespace srv

}  // namespace zit6_interfaces

#ifdef __cplusplus
extern "C"
{
#endif

const rosidl_service_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_SYMBOL_NAME(rosidl_typesupport_c, zit6_interfaces, srv, UpdateParams)() {
  return &::zit6_interfaces::srv::rosidl_typesupport_c::UpdateParams_service_type_support_handle;
}

#ifdef __cplusplus
}
#endif
