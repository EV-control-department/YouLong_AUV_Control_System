// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from uv_msgs:srv/RunTask.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "uv_msgs/srv/detail/run_task__rosidl_typesupport_introspection_c.h"
#include "uv_msgs/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "uv_msgs/srv/detail/run_task__functions.h"
#include "uv_msgs/srv/detail/run_task__struct.h"


// Include directives for member types
// Member `task_name`
#include "rosidl_runtime_c/string_functions.h"

#ifdef __cplusplus
extern "C"
{
#endif

void uv_msgs__srv__RunTask_Request__rosidl_typesupport_introspection_c__RunTask_Request_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  uv_msgs__srv__RunTask_Request__init(message_memory);
}

void uv_msgs__srv__RunTask_Request__rosidl_typesupport_introspection_c__RunTask_Request_fini_function(void * message_memory)
{
  uv_msgs__srv__RunTask_Request__fini(message_memory);
}

static rosidl_typesupport_introspection_c__MessageMember uv_msgs__srv__RunTask_Request__rosidl_typesupport_introspection_c__RunTask_Request_message_member_array[2] = {
  {
    "task_name",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_STRING,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(uv_msgs__srv__RunTask_Request, task_name),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "start",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_BOOLEAN,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(uv_msgs__srv__RunTask_Request, start),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers uv_msgs__srv__RunTask_Request__rosidl_typesupport_introspection_c__RunTask_Request_message_members = {
  "uv_msgs__srv",  // message namespace
  "RunTask_Request",  // message name
  2,  // number of fields
  sizeof(uv_msgs__srv__RunTask_Request),
  false,  // has_any_key_member_
  uv_msgs__srv__RunTask_Request__rosidl_typesupport_introspection_c__RunTask_Request_message_member_array,  // message members
  uv_msgs__srv__RunTask_Request__rosidl_typesupport_introspection_c__RunTask_Request_init_function,  // function to initialize message memory (memory has to be allocated)
  uv_msgs__srv__RunTask_Request__rosidl_typesupport_introspection_c__RunTask_Request_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t uv_msgs__srv__RunTask_Request__rosidl_typesupport_introspection_c__RunTask_Request_message_type_support_handle = {
  0,
  &uv_msgs__srv__RunTask_Request__rosidl_typesupport_introspection_c__RunTask_Request_message_members,
  get_message_typesupport_handle_function,
  &uv_msgs__srv__RunTask_Request__get_type_hash,
  &uv_msgs__srv__RunTask_Request__get_type_description,
  &uv_msgs__srv__RunTask_Request__get_type_description_sources,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_uv_msgs
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, uv_msgs, srv, RunTask_Request)() {
  if (!uv_msgs__srv__RunTask_Request__rosidl_typesupport_introspection_c__RunTask_Request_message_type_support_handle.typesupport_identifier) {
    uv_msgs__srv__RunTask_Request__rosidl_typesupport_introspection_c__RunTask_Request_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &uv_msgs__srv__RunTask_Request__rosidl_typesupport_introspection_c__RunTask_Request_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif

// already included above
// #include <stddef.h>
// already included above
// #include "uv_msgs/srv/detail/run_task__rosidl_typesupport_introspection_c.h"
// already included above
// #include "uv_msgs/msg/rosidl_typesupport_introspection_c__visibility_control.h"
// already included above
// #include "rosidl_typesupport_introspection_c/field_types.h"
// already included above
// #include "rosidl_typesupport_introspection_c/identifier.h"
// already included above
// #include "rosidl_typesupport_introspection_c/message_introspection.h"
// already included above
// #include "uv_msgs/srv/detail/run_task__functions.h"
// already included above
// #include "uv_msgs/srv/detail/run_task__struct.h"


// Include directives for member types
// Member `message`
// already included above
// #include "rosidl_runtime_c/string_functions.h"

#ifdef __cplusplus
extern "C"
{
#endif

void uv_msgs__srv__RunTask_Response__rosidl_typesupport_introspection_c__RunTask_Response_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  uv_msgs__srv__RunTask_Response__init(message_memory);
}

void uv_msgs__srv__RunTask_Response__rosidl_typesupport_introspection_c__RunTask_Response_fini_function(void * message_memory)
{
  uv_msgs__srv__RunTask_Response__fini(message_memory);
}

static rosidl_typesupport_introspection_c__MessageMember uv_msgs__srv__RunTask_Response__rosidl_typesupport_introspection_c__RunTask_Response_message_member_array[2] = {
  {
    "success",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_BOOLEAN,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(uv_msgs__srv__RunTask_Response, success),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "message",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_STRING,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(uv_msgs__srv__RunTask_Response, message),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers uv_msgs__srv__RunTask_Response__rosidl_typesupport_introspection_c__RunTask_Response_message_members = {
  "uv_msgs__srv",  // message namespace
  "RunTask_Response",  // message name
  2,  // number of fields
  sizeof(uv_msgs__srv__RunTask_Response),
  false,  // has_any_key_member_
  uv_msgs__srv__RunTask_Response__rosidl_typesupport_introspection_c__RunTask_Response_message_member_array,  // message members
  uv_msgs__srv__RunTask_Response__rosidl_typesupport_introspection_c__RunTask_Response_init_function,  // function to initialize message memory (memory has to be allocated)
  uv_msgs__srv__RunTask_Response__rosidl_typesupport_introspection_c__RunTask_Response_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t uv_msgs__srv__RunTask_Response__rosidl_typesupport_introspection_c__RunTask_Response_message_type_support_handle = {
  0,
  &uv_msgs__srv__RunTask_Response__rosidl_typesupport_introspection_c__RunTask_Response_message_members,
  get_message_typesupport_handle_function,
  &uv_msgs__srv__RunTask_Response__get_type_hash,
  &uv_msgs__srv__RunTask_Response__get_type_description,
  &uv_msgs__srv__RunTask_Response__get_type_description_sources,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_uv_msgs
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, uv_msgs, srv, RunTask_Response)() {
  if (!uv_msgs__srv__RunTask_Response__rosidl_typesupport_introspection_c__RunTask_Response_message_type_support_handle.typesupport_identifier) {
    uv_msgs__srv__RunTask_Response__rosidl_typesupport_introspection_c__RunTask_Response_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &uv_msgs__srv__RunTask_Response__rosidl_typesupport_introspection_c__RunTask_Response_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif

// already included above
// #include <stddef.h>
// already included above
// #include "uv_msgs/srv/detail/run_task__rosidl_typesupport_introspection_c.h"
// already included above
// #include "uv_msgs/msg/rosidl_typesupport_introspection_c__visibility_control.h"
// already included above
// #include "rosidl_typesupport_introspection_c/field_types.h"
// already included above
// #include "rosidl_typesupport_introspection_c/identifier.h"
// already included above
// #include "rosidl_typesupport_introspection_c/message_introspection.h"
// already included above
// #include "uv_msgs/srv/detail/run_task__functions.h"
// already included above
// #include "uv_msgs/srv/detail/run_task__struct.h"


// Include directives for member types
// Member `info`
#include "service_msgs/msg/service_event_info.h"
// Member `info`
#include "service_msgs/msg/detail/service_event_info__rosidl_typesupport_introspection_c.h"
// Member `request`
// Member `response`
#include "uv_msgs/srv/run_task.h"
// Member `request`
// Member `response`
// already included above
// #include "uv_msgs/srv/detail/run_task__rosidl_typesupport_introspection_c.h"

#ifdef __cplusplus
extern "C"
{
#endif

void uv_msgs__srv__RunTask_Event__rosidl_typesupport_introspection_c__RunTask_Event_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  uv_msgs__srv__RunTask_Event__init(message_memory);
}

void uv_msgs__srv__RunTask_Event__rosidl_typesupport_introspection_c__RunTask_Event_fini_function(void * message_memory)
{
  uv_msgs__srv__RunTask_Event__fini(message_memory);
}

size_t uv_msgs__srv__RunTask_Event__rosidl_typesupport_introspection_c__size_function__RunTask_Event__request(
  const void * untyped_member)
{
  const uv_msgs__srv__RunTask_Request__Sequence * member =
    (const uv_msgs__srv__RunTask_Request__Sequence *)(untyped_member);
  return member->size;
}

const void * uv_msgs__srv__RunTask_Event__rosidl_typesupport_introspection_c__get_const_function__RunTask_Event__request(
  const void * untyped_member, size_t index)
{
  const uv_msgs__srv__RunTask_Request__Sequence * member =
    (const uv_msgs__srv__RunTask_Request__Sequence *)(untyped_member);
  return &member->data[index];
}

void * uv_msgs__srv__RunTask_Event__rosidl_typesupport_introspection_c__get_function__RunTask_Event__request(
  void * untyped_member, size_t index)
{
  uv_msgs__srv__RunTask_Request__Sequence * member =
    (uv_msgs__srv__RunTask_Request__Sequence *)(untyped_member);
  return &member->data[index];
}

void uv_msgs__srv__RunTask_Event__rosidl_typesupport_introspection_c__fetch_function__RunTask_Event__request(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const uv_msgs__srv__RunTask_Request * item =
    ((const uv_msgs__srv__RunTask_Request *)
    uv_msgs__srv__RunTask_Event__rosidl_typesupport_introspection_c__get_const_function__RunTask_Event__request(untyped_member, index));
  uv_msgs__srv__RunTask_Request * value =
    (uv_msgs__srv__RunTask_Request *)(untyped_value);
  *value = *item;
}

void uv_msgs__srv__RunTask_Event__rosidl_typesupport_introspection_c__assign_function__RunTask_Event__request(
  void * untyped_member, size_t index, const void * untyped_value)
{
  uv_msgs__srv__RunTask_Request * item =
    ((uv_msgs__srv__RunTask_Request *)
    uv_msgs__srv__RunTask_Event__rosidl_typesupport_introspection_c__get_function__RunTask_Event__request(untyped_member, index));
  const uv_msgs__srv__RunTask_Request * value =
    (const uv_msgs__srv__RunTask_Request *)(untyped_value);
  *item = *value;
}

bool uv_msgs__srv__RunTask_Event__rosidl_typesupport_introspection_c__resize_function__RunTask_Event__request(
  void * untyped_member, size_t size)
{
  uv_msgs__srv__RunTask_Request__Sequence * member =
    (uv_msgs__srv__RunTask_Request__Sequence *)(untyped_member);
  uv_msgs__srv__RunTask_Request__Sequence__fini(member);
  return uv_msgs__srv__RunTask_Request__Sequence__init(member, size);
}

size_t uv_msgs__srv__RunTask_Event__rosidl_typesupport_introspection_c__size_function__RunTask_Event__response(
  const void * untyped_member)
{
  const uv_msgs__srv__RunTask_Response__Sequence * member =
    (const uv_msgs__srv__RunTask_Response__Sequence *)(untyped_member);
  return member->size;
}

const void * uv_msgs__srv__RunTask_Event__rosidl_typesupport_introspection_c__get_const_function__RunTask_Event__response(
  const void * untyped_member, size_t index)
{
  const uv_msgs__srv__RunTask_Response__Sequence * member =
    (const uv_msgs__srv__RunTask_Response__Sequence *)(untyped_member);
  return &member->data[index];
}

void * uv_msgs__srv__RunTask_Event__rosidl_typesupport_introspection_c__get_function__RunTask_Event__response(
  void * untyped_member, size_t index)
{
  uv_msgs__srv__RunTask_Response__Sequence * member =
    (uv_msgs__srv__RunTask_Response__Sequence *)(untyped_member);
  return &member->data[index];
}

void uv_msgs__srv__RunTask_Event__rosidl_typesupport_introspection_c__fetch_function__RunTask_Event__response(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const uv_msgs__srv__RunTask_Response * item =
    ((const uv_msgs__srv__RunTask_Response *)
    uv_msgs__srv__RunTask_Event__rosidl_typesupport_introspection_c__get_const_function__RunTask_Event__response(untyped_member, index));
  uv_msgs__srv__RunTask_Response * value =
    (uv_msgs__srv__RunTask_Response *)(untyped_value);
  *value = *item;
}

void uv_msgs__srv__RunTask_Event__rosidl_typesupport_introspection_c__assign_function__RunTask_Event__response(
  void * untyped_member, size_t index, const void * untyped_value)
{
  uv_msgs__srv__RunTask_Response * item =
    ((uv_msgs__srv__RunTask_Response *)
    uv_msgs__srv__RunTask_Event__rosidl_typesupport_introspection_c__get_function__RunTask_Event__response(untyped_member, index));
  const uv_msgs__srv__RunTask_Response * value =
    (const uv_msgs__srv__RunTask_Response *)(untyped_value);
  *item = *value;
}

bool uv_msgs__srv__RunTask_Event__rosidl_typesupport_introspection_c__resize_function__RunTask_Event__response(
  void * untyped_member, size_t size)
{
  uv_msgs__srv__RunTask_Response__Sequence * member =
    (uv_msgs__srv__RunTask_Response__Sequence *)(untyped_member);
  uv_msgs__srv__RunTask_Response__Sequence__fini(member);
  return uv_msgs__srv__RunTask_Response__Sequence__init(member, size);
}

static rosidl_typesupport_introspection_c__MessageMember uv_msgs__srv__RunTask_Event__rosidl_typesupport_introspection_c__RunTask_Event_message_member_array[3] = {
  {
    "info",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(uv_msgs__srv__RunTask_Event, info),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "request",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is key
    true,  // is array
    1,  // array size
    true,  // is upper bound
    offsetof(uv_msgs__srv__RunTask_Event, request),  // bytes offset in struct
    NULL,  // default value
    uv_msgs__srv__RunTask_Event__rosidl_typesupport_introspection_c__size_function__RunTask_Event__request,  // size() function pointer
    uv_msgs__srv__RunTask_Event__rosidl_typesupport_introspection_c__get_const_function__RunTask_Event__request,  // get_const(index) function pointer
    uv_msgs__srv__RunTask_Event__rosidl_typesupport_introspection_c__get_function__RunTask_Event__request,  // get(index) function pointer
    uv_msgs__srv__RunTask_Event__rosidl_typesupport_introspection_c__fetch_function__RunTask_Event__request,  // fetch(index, &value) function pointer
    uv_msgs__srv__RunTask_Event__rosidl_typesupport_introspection_c__assign_function__RunTask_Event__request,  // assign(index, value) function pointer
    uv_msgs__srv__RunTask_Event__rosidl_typesupport_introspection_c__resize_function__RunTask_Event__request  // resize(index) function pointer
  },
  {
    "response",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is key
    true,  // is array
    1,  // array size
    true,  // is upper bound
    offsetof(uv_msgs__srv__RunTask_Event, response),  // bytes offset in struct
    NULL,  // default value
    uv_msgs__srv__RunTask_Event__rosidl_typesupport_introspection_c__size_function__RunTask_Event__response,  // size() function pointer
    uv_msgs__srv__RunTask_Event__rosidl_typesupport_introspection_c__get_const_function__RunTask_Event__response,  // get_const(index) function pointer
    uv_msgs__srv__RunTask_Event__rosidl_typesupport_introspection_c__get_function__RunTask_Event__response,  // get(index) function pointer
    uv_msgs__srv__RunTask_Event__rosidl_typesupport_introspection_c__fetch_function__RunTask_Event__response,  // fetch(index, &value) function pointer
    uv_msgs__srv__RunTask_Event__rosidl_typesupport_introspection_c__assign_function__RunTask_Event__response,  // assign(index, value) function pointer
    uv_msgs__srv__RunTask_Event__rosidl_typesupport_introspection_c__resize_function__RunTask_Event__response  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers uv_msgs__srv__RunTask_Event__rosidl_typesupport_introspection_c__RunTask_Event_message_members = {
  "uv_msgs__srv",  // message namespace
  "RunTask_Event",  // message name
  3,  // number of fields
  sizeof(uv_msgs__srv__RunTask_Event),
  false,  // has_any_key_member_
  uv_msgs__srv__RunTask_Event__rosidl_typesupport_introspection_c__RunTask_Event_message_member_array,  // message members
  uv_msgs__srv__RunTask_Event__rosidl_typesupport_introspection_c__RunTask_Event_init_function,  // function to initialize message memory (memory has to be allocated)
  uv_msgs__srv__RunTask_Event__rosidl_typesupport_introspection_c__RunTask_Event_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t uv_msgs__srv__RunTask_Event__rosidl_typesupport_introspection_c__RunTask_Event_message_type_support_handle = {
  0,
  &uv_msgs__srv__RunTask_Event__rosidl_typesupport_introspection_c__RunTask_Event_message_members,
  get_message_typesupport_handle_function,
  &uv_msgs__srv__RunTask_Event__get_type_hash,
  &uv_msgs__srv__RunTask_Event__get_type_description,
  &uv_msgs__srv__RunTask_Event__get_type_description_sources,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_uv_msgs
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, uv_msgs, srv, RunTask_Event)() {
  uv_msgs__srv__RunTask_Event__rosidl_typesupport_introspection_c__RunTask_Event_message_member_array[0].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, service_msgs, msg, ServiceEventInfo)();
  uv_msgs__srv__RunTask_Event__rosidl_typesupport_introspection_c__RunTask_Event_message_member_array[1].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, uv_msgs, srv, RunTask_Request)();
  uv_msgs__srv__RunTask_Event__rosidl_typesupport_introspection_c__RunTask_Event_message_member_array[2].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, uv_msgs, srv, RunTask_Response)();
  if (!uv_msgs__srv__RunTask_Event__rosidl_typesupport_introspection_c__RunTask_Event_message_type_support_handle.typesupport_identifier) {
    uv_msgs__srv__RunTask_Event__rosidl_typesupport_introspection_c__RunTask_Event_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &uv_msgs__srv__RunTask_Event__rosidl_typesupport_introspection_c__RunTask_Event_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif

#include "rosidl_runtime_c/service_type_support_struct.h"
// already included above
// #include "uv_msgs/msg/rosidl_typesupport_introspection_c__visibility_control.h"
// already included above
// #include "uv_msgs/srv/detail/run_task__rosidl_typesupport_introspection_c.h"
// already included above
// #include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/service_introspection.h"

// this is intentionally not const to allow initialization later to prevent an initialization race
static rosidl_typesupport_introspection_c__ServiceMembers uv_msgs__srv__detail__run_task__rosidl_typesupport_introspection_c__RunTask_service_members = {
  "uv_msgs__srv",  // service namespace
  "RunTask",  // service name
  // the following fields are initialized below on first access
  NULL,  // request message
  // uv_msgs__srv__detail__run_task__rosidl_typesupport_introspection_c__RunTask_Request_message_type_support_handle,
  NULL,  // response message
  // uv_msgs__srv__detail__run_task__rosidl_typesupport_introspection_c__RunTask_Response_message_type_support_handle
  NULL  // event_message
  // uv_msgs__srv__detail__run_task__rosidl_typesupport_introspection_c__RunTask_Response_message_type_support_handle
};


static rosidl_service_type_support_t uv_msgs__srv__detail__run_task__rosidl_typesupport_introspection_c__RunTask_service_type_support_handle = {
  0,
  &uv_msgs__srv__detail__run_task__rosidl_typesupport_introspection_c__RunTask_service_members,
  get_service_typesupport_handle_function,
  &uv_msgs__srv__RunTask_Request__rosidl_typesupport_introspection_c__RunTask_Request_message_type_support_handle,
  &uv_msgs__srv__RunTask_Response__rosidl_typesupport_introspection_c__RunTask_Response_message_type_support_handle,
  &uv_msgs__srv__RunTask_Event__rosidl_typesupport_introspection_c__RunTask_Event_message_type_support_handle,
  ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_CREATE_EVENT_MESSAGE_SYMBOL_NAME(
    rosidl_typesupport_c,
    uv_msgs,
    srv,
    RunTask
  ),
  ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_DESTROY_EVENT_MESSAGE_SYMBOL_NAME(
    rosidl_typesupport_c,
    uv_msgs,
    srv,
    RunTask
  ),
  &uv_msgs__srv__RunTask__get_type_hash,
  &uv_msgs__srv__RunTask__get_type_description,
  &uv_msgs__srv__RunTask__get_type_description_sources,
};

// Forward declaration of message type support functions for service members
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, uv_msgs, srv, RunTask_Request)(void);

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, uv_msgs, srv, RunTask_Response)(void);

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, uv_msgs, srv, RunTask_Event)(void);

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_uv_msgs
const rosidl_service_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_SYMBOL_NAME(rosidl_typesupport_introspection_c, uv_msgs, srv, RunTask)(void) {
  if (!uv_msgs__srv__detail__run_task__rosidl_typesupport_introspection_c__RunTask_service_type_support_handle.typesupport_identifier) {
    uv_msgs__srv__detail__run_task__rosidl_typesupport_introspection_c__RunTask_service_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  rosidl_typesupport_introspection_c__ServiceMembers * service_members =
    (rosidl_typesupport_introspection_c__ServiceMembers *)uv_msgs__srv__detail__run_task__rosidl_typesupport_introspection_c__RunTask_service_type_support_handle.data;

  if (!service_members->request_members_) {
    service_members->request_members_ =
      (const rosidl_typesupport_introspection_c__MessageMembers *)
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, uv_msgs, srv, RunTask_Request)()->data;
  }
  if (!service_members->response_members_) {
    service_members->response_members_ =
      (const rosidl_typesupport_introspection_c__MessageMembers *)
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, uv_msgs, srv, RunTask_Response)()->data;
  }
  if (!service_members->event_members_) {
    service_members->event_members_ =
      (const rosidl_typesupport_introspection_c__MessageMembers *)
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, uv_msgs, srv, RunTask_Event)()->data;
  }

  return &uv_msgs__srv__detail__run_task__rosidl_typesupport_introspection_c__RunTask_service_type_support_handle;
}
