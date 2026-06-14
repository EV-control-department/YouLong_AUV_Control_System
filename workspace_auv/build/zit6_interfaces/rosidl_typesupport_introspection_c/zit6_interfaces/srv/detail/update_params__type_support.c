// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from zit6_interfaces:srv/UpdateParams.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "zit6_interfaces/srv/detail/update_params__rosidl_typesupport_introspection_c.h"
#include "zit6_interfaces/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "zit6_interfaces/srv/detail/update_params__functions.h"
#include "zit6_interfaces/srv/detail/update_params__struct.h"


// Include directives for member types
// Member `json`
// Member `paths`
// Member `values`
#include "rosidl_runtime_c/string_functions.h"

#ifdef __cplusplus
extern "C"
{
#endif

void zit6_interfaces__srv__UpdateParams_Request__rosidl_typesupport_introspection_c__UpdateParams_Request_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  zit6_interfaces__srv__UpdateParams_Request__init(message_memory);
}

void zit6_interfaces__srv__UpdateParams_Request__rosidl_typesupport_introspection_c__UpdateParams_Request_fini_function(void * message_memory)
{
  zit6_interfaces__srv__UpdateParams_Request__fini(message_memory);
}

size_t zit6_interfaces__srv__UpdateParams_Request__rosidl_typesupport_introspection_c__size_function__UpdateParams_Request__paths(
  const void * untyped_member)
{
  const rosidl_runtime_c__String__Sequence * member =
    (const rosidl_runtime_c__String__Sequence *)(untyped_member);
  return member->size;
}

const void * zit6_interfaces__srv__UpdateParams_Request__rosidl_typesupport_introspection_c__get_const_function__UpdateParams_Request__paths(
  const void * untyped_member, size_t index)
{
  const rosidl_runtime_c__String__Sequence * member =
    (const rosidl_runtime_c__String__Sequence *)(untyped_member);
  return &member->data[index];
}

void * zit6_interfaces__srv__UpdateParams_Request__rosidl_typesupport_introspection_c__get_function__UpdateParams_Request__paths(
  void * untyped_member, size_t index)
{
  rosidl_runtime_c__String__Sequence * member =
    (rosidl_runtime_c__String__Sequence *)(untyped_member);
  return &member->data[index];
}

void zit6_interfaces__srv__UpdateParams_Request__rosidl_typesupport_introspection_c__fetch_function__UpdateParams_Request__paths(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const rosidl_runtime_c__String * item =
    ((const rosidl_runtime_c__String *)
    zit6_interfaces__srv__UpdateParams_Request__rosidl_typesupport_introspection_c__get_const_function__UpdateParams_Request__paths(untyped_member, index));
  rosidl_runtime_c__String * value =
    (rosidl_runtime_c__String *)(untyped_value);
  *value = *item;
}

void zit6_interfaces__srv__UpdateParams_Request__rosidl_typesupport_introspection_c__assign_function__UpdateParams_Request__paths(
  void * untyped_member, size_t index, const void * untyped_value)
{
  rosidl_runtime_c__String * item =
    ((rosidl_runtime_c__String *)
    zit6_interfaces__srv__UpdateParams_Request__rosidl_typesupport_introspection_c__get_function__UpdateParams_Request__paths(untyped_member, index));
  const rosidl_runtime_c__String * value =
    (const rosidl_runtime_c__String *)(untyped_value);
  *item = *value;
}

bool zit6_interfaces__srv__UpdateParams_Request__rosidl_typesupport_introspection_c__resize_function__UpdateParams_Request__paths(
  void * untyped_member, size_t size)
{
  rosidl_runtime_c__String__Sequence * member =
    (rosidl_runtime_c__String__Sequence *)(untyped_member);
  rosidl_runtime_c__String__Sequence__fini(member);
  return rosidl_runtime_c__String__Sequence__init(member, size);
}

size_t zit6_interfaces__srv__UpdateParams_Request__rosidl_typesupport_introspection_c__size_function__UpdateParams_Request__values(
  const void * untyped_member)
{
  const rosidl_runtime_c__String__Sequence * member =
    (const rosidl_runtime_c__String__Sequence *)(untyped_member);
  return member->size;
}

const void * zit6_interfaces__srv__UpdateParams_Request__rosidl_typesupport_introspection_c__get_const_function__UpdateParams_Request__values(
  const void * untyped_member, size_t index)
{
  const rosidl_runtime_c__String__Sequence * member =
    (const rosidl_runtime_c__String__Sequence *)(untyped_member);
  return &member->data[index];
}

void * zit6_interfaces__srv__UpdateParams_Request__rosidl_typesupport_introspection_c__get_function__UpdateParams_Request__values(
  void * untyped_member, size_t index)
{
  rosidl_runtime_c__String__Sequence * member =
    (rosidl_runtime_c__String__Sequence *)(untyped_member);
  return &member->data[index];
}

void zit6_interfaces__srv__UpdateParams_Request__rosidl_typesupport_introspection_c__fetch_function__UpdateParams_Request__values(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const rosidl_runtime_c__String * item =
    ((const rosidl_runtime_c__String *)
    zit6_interfaces__srv__UpdateParams_Request__rosidl_typesupport_introspection_c__get_const_function__UpdateParams_Request__values(untyped_member, index));
  rosidl_runtime_c__String * value =
    (rosidl_runtime_c__String *)(untyped_value);
  *value = *item;
}

void zit6_interfaces__srv__UpdateParams_Request__rosidl_typesupport_introspection_c__assign_function__UpdateParams_Request__values(
  void * untyped_member, size_t index, const void * untyped_value)
{
  rosidl_runtime_c__String * item =
    ((rosidl_runtime_c__String *)
    zit6_interfaces__srv__UpdateParams_Request__rosidl_typesupport_introspection_c__get_function__UpdateParams_Request__values(untyped_member, index));
  const rosidl_runtime_c__String * value =
    (const rosidl_runtime_c__String *)(untyped_value);
  *item = *value;
}

bool zit6_interfaces__srv__UpdateParams_Request__rosidl_typesupport_introspection_c__resize_function__UpdateParams_Request__values(
  void * untyped_member, size_t size)
{
  rosidl_runtime_c__String__Sequence * member =
    (rosidl_runtime_c__String__Sequence *)(untyped_member);
  rosidl_runtime_c__String__Sequence__fini(member);
  return rosidl_runtime_c__String__Sequence__init(member, size);
}

static rosidl_typesupport_introspection_c__MessageMember zit6_interfaces__srv__UpdateParams_Request__rosidl_typesupport_introspection_c__UpdateParams_Request_message_member_array[3] = {
  {
    "json",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_STRING,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(zit6_interfaces__srv__UpdateParams_Request, json),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "paths",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_STRING,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is key
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(zit6_interfaces__srv__UpdateParams_Request, paths),  // bytes offset in struct
    NULL,  // default value
    zit6_interfaces__srv__UpdateParams_Request__rosidl_typesupport_introspection_c__size_function__UpdateParams_Request__paths,  // size() function pointer
    zit6_interfaces__srv__UpdateParams_Request__rosidl_typesupport_introspection_c__get_const_function__UpdateParams_Request__paths,  // get_const(index) function pointer
    zit6_interfaces__srv__UpdateParams_Request__rosidl_typesupport_introspection_c__get_function__UpdateParams_Request__paths,  // get(index) function pointer
    zit6_interfaces__srv__UpdateParams_Request__rosidl_typesupport_introspection_c__fetch_function__UpdateParams_Request__paths,  // fetch(index, &value) function pointer
    zit6_interfaces__srv__UpdateParams_Request__rosidl_typesupport_introspection_c__assign_function__UpdateParams_Request__paths,  // assign(index, value) function pointer
    zit6_interfaces__srv__UpdateParams_Request__rosidl_typesupport_introspection_c__resize_function__UpdateParams_Request__paths  // resize(index) function pointer
  },
  {
    "values",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_STRING,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is key
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(zit6_interfaces__srv__UpdateParams_Request, values),  // bytes offset in struct
    NULL,  // default value
    zit6_interfaces__srv__UpdateParams_Request__rosidl_typesupport_introspection_c__size_function__UpdateParams_Request__values,  // size() function pointer
    zit6_interfaces__srv__UpdateParams_Request__rosidl_typesupport_introspection_c__get_const_function__UpdateParams_Request__values,  // get_const(index) function pointer
    zit6_interfaces__srv__UpdateParams_Request__rosidl_typesupport_introspection_c__get_function__UpdateParams_Request__values,  // get(index) function pointer
    zit6_interfaces__srv__UpdateParams_Request__rosidl_typesupport_introspection_c__fetch_function__UpdateParams_Request__values,  // fetch(index, &value) function pointer
    zit6_interfaces__srv__UpdateParams_Request__rosidl_typesupport_introspection_c__assign_function__UpdateParams_Request__values,  // assign(index, value) function pointer
    zit6_interfaces__srv__UpdateParams_Request__rosidl_typesupport_introspection_c__resize_function__UpdateParams_Request__values  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers zit6_interfaces__srv__UpdateParams_Request__rosidl_typesupport_introspection_c__UpdateParams_Request_message_members = {
  "zit6_interfaces__srv",  // message namespace
  "UpdateParams_Request",  // message name
  3,  // number of fields
  sizeof(zit6_interfaces__srv__UpdateParams_Request),
  false,  // has_any_key_member_
  zit6_interfaces__srv__UpdateParams_Request__rosidl_typesupport_introspection_c__UpdateParams_Request_message_member_array,  // message members
  zit6_interfaces__srv__UpdateParams_Request__rosidl_typesupport_introspection_c__UpdateParams_Request_init_function,  // function to initialize message memory (memory has to be allocated)
  zit6_interfaces__srv__UpdateParams_Request__rosidl_typesupport_introspection_c__UpdateParams_Request_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t zit6_interfaces__srv__UpdateParams_Request__rosidl_typesupport_introspection_c__UpdateParams_Request_message_type_support_handle = {
  0,
  &zit6_interfaces__srv__UpdateParams_Request__rosidl_typesupport_introspection_c__UpdateParams_Request_message_members,
  get_message_typesupport_handle_function,
  &zit6_interfaces__srv__UpdateParams_Request__get_type_hash,
  &zit6_interfaces__srv__UpdateParams_Request__get_type_description,
  &zit6_interfaces__srv__UpdateParams_Request__get_type_description_sources,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_zit6_interfaces
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, zit6_interfaces, srv, UpdateParams_Request)() {
  if (!zit6_interfaces__srv__UpdateParams_Request__rosidl_typesupport_introspection_c__UpdateParams_Request_message_type_support_handle.typesupport_identifier) {
    zit6_interfaces__srv__UpdateParams_Request__rosidl_typesupport_introspection_c__UpdateParams_Request_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &zit6_interfaces__srv__UpdateParams_Request__rosidl_typesupport_introspection_c__UpdateParams_Request_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif

// already included above
// #include <stddef.h>
// already included above
// #include "zit6_interfaces/srv/detail/update_params__rosidl_typesupport_introspection_c.h"
// already included above
// #include "zit6_interfaces/msg/rosidl_typesupport_introspection_c__visibility_control.h"
// already included above
// #include "rosidl_typesupport_introspection_c/field_types.h"
// already included above
// #include "rosidl_typesupport_introspection_c/identifier.h"
// already included above
// #include "rosidl_typesupport_introspection_c/message_introspection.h"
// already included above
// #include "zit6_interfaces/srv/detail/update_params__functions.h"
// already included above
// #include "zit6_interfaces/srv/detail/update_params__struct.h"


// Include directives for member types
// Member `message`
// already included above
// #include "rosidl_runtime_c/string_functions.h"

#ifdef __cplusplus
extern "C"
{
#endif

void zit6_interfaces__srv__UpdateParams_Response__rosidl_typesupport_introspection_c__UpdateParams_Response_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  zit6_interfaces__srv__UpdateParams_Response__init(message_memory);
}

void zit6_interfaces__srv__UpdateParams_Response__rosidl_typesupport_introspection_c__UpdateParams_Response_fini_function(void * message_memory)
{
  zit6_interfaces__srv__UpdateParams_Response__fini(message_memory);
}

static rosidl_typesupport_introspection_c__MessageMember zit6_interfaces__srv__UpdateParams_Response__rosidl_typesupport_introspection_c__UpdateParams_Response_message_member_array[2] = {
  {
    "success",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_BOOLEAN,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(zit6_interfaces__srv__UpdateParams_Response, success),  // bytes offset in struct
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
    offsetof(zit6_interfaces__srv__UpdateParams_Response, message),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers zit6_interfaces__srv__UpdateParams_Response__rosidl_typesupport_introspection_c__UpdateParams_Response_message_members = {
  "zit6_interfaces__srv",  // message namespace
  "UpdateParams_Response",  // message name
  2,  // number of fields
  sizeof(zit6_interfaces__srv__UpdateParams_Response),
  false,  // has_any_key_member_
  zit6_interfaces__srv__UpdateParams_Response__rosidl_typesupport_introspection_c__UpdateParams_Response_message_member_array,  // message members
  zit6_interfaces__srv__UpdateParams_Response__rosidl_typesupport_introspection_c__UpdateParams_Response_init_function,  // function to initialize message memory (memory has to be allocated)
  zit6_interfaces__srv__UpdateParams_Response__rosidl_typesupport_introspection_c__UpdateParams_Response_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t zit6_interfaces__srv__UpdateParams_Response__rosidl_typesupport_introspection_c__UpdateParams_Response_message_type_support_handle = {
  0,
  &zit6_interfaces__srv__UpdateParams_Response__rosidl_typesupport_introspection_c__UpdateParams_Response_message_members,
  get_message_typesupport_handle_function,
  &zit6_interfaces__srv__UpdateParams_Response__get_type_hash,
  &zit6_interfaces__srv__UpdateParams_Response__get_type_description,
  &zit6_interfaces__srv__UpdateParams_Response__get_type_description_sources,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_zit6_interfaces
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, zit6_interfaces, srv, UpdateParams_Response)() {
  if (!zit6_interfaces__srv__UpdateParams_Response__rosidl_typesupport_introspection_c__UpdateParams_Response_message_type_support_handle.typesupport_identifier) {
    zit6_interfaces__srv__UpdateParams_Response__rosidl_typesupport_introspection_c__UpdateParams_Response_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &zit6_interfaces__srv__UpdateParams_Response__rosidl_typesupport_introspection_c__UpdateParams_Response_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif

// already included above
// #include <stddef.h>
// already included above
// #include "zit6_interfaces/srv/detail/update_params__rosidl_typesupport_introspection_c.h"
// already included above
// #include "zit6_interfaces/msg/rosidl_typesupport_introspection_c__visibility_control.h"
// already included above
// #include "rosidl_typesupport_introspection_c/field_types.h"
// already included above
// #include "rosidl_typesupport_introspection_c/identifier.h"
// already included above
// #include "rosidl_typesupport_introspection_c/message_introspection.h"
// already included above
// #include "zit6_interfaces/srv/detail/update_params__functions.h"
// already included above
// #include "zit6_interfaces/srv/detail/update_params__struct.h"


// Include directives for member types
// Member `info`
#include "service_msgs/msg/service_event_info.h"
// Member `info`
#include "service_msgs/msg/detail/service_event_info__rosidl_typesupport_introspection_c.h"
// Member `request`
// Member `response`
#include "zit6_interfaces/srv/update_params.h"
// Member `request`
// Member `response`
// already included above
// #include "zit6_interfaces/srv/detail/update_params__rosidl_typesupport_introspection_c.h"

#ifdef __cplusplus
extern "C"
{
#endif

void zit6_interfaces__srv__UpdateParams_Event__rosidl_typesupport_introspection_c__UpdateParams_Event_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  zit6_interfaces__srv__UpdateParams_Event__init(message_memory);
}

void zit6_interfaces__srv__UpdateParams_Event__rosidl_typesupport_introspection_c__UpdateParams_Event_fini_function(void * message_memory)
{
  zit6_interfaces__srv__UpdateParams_Event__fini(message_memory);
}

size_t zit6_interfaces__srv__UpdateParams_Event__rosidl_typesupport_introspection_c__size_function__UpdateParams_Event__request(
  const void * untyped_member)
{
  const zit6_interfaces__srv__UpdateParams_Request__Sequence * member =
    (const zit6_interfaces__srv__UpdateParams_Request__Sequence *)(untyped_member);
  return member->size;
}

const void * zit6_interfaces__srv__UpdateParams_Event__rosidl_typesupport_introspection_c__get_const_function__UpdateParams_Event__request(
  const void * untyped_member, size_t index)
{
  const zit6_interfaces__srv__UpdateParams_Request__Sequence * member =
    (const zit6_interfaces__srv__UpdateParams_Request__Sequence *)(untyped_member);
  return &member->data[index];
}

void * zit6_interfaces__srv__UpdateParams_Event__rosidl_typesupport_introspection_c__get_function__UpdateParams_Event__request(
  void * untyped_member, size_t index)
{
  zit6_interfaces__srv__UpdateParams_Request__Sequence * member =
    (zit6_interfaces__srv__UpdateParams_Request__Sequence *)(untyped_member);
  return &member->data[index];
}

void zit6_interfaces__srv__UpdateParams_Event__rosidl_typesupport_introspection_c__fetch_function__UpdateParams_Event__request(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const zit6_interfaces__srv__UpdateParams_Request * item =
    ((const zit6_interfaces__srv__UpdateParams_Request *)
    zit6_interfaces__srv__UpdateParams_Event__rosidl_typesupport_introspection_c__get_const_function__UpdateParams_Event__request(untyped_member, index));
  zit6_interfaces__srv__UpdateParams_Request * value =
    (zit6_interfaces__srv__UpdateParams_Request *)(untyped_value);
  *value = *item;
}

void zit6_interfaces__srv__UpdateParams_Event__rosidl_typesupport_introspection_c__assign_function__UpdateParams_Event__request(
  void * untyped_member, size_t index, const void * untyped_value)
{
  zit6_interfaces__srv__UpdateParams_Request * item =
    ((zit6_interfaces__srv__UpdateParams_Request *)
    zit6_interfaces__srv__UpdateParams_Event__rosidl_typesupport_introspection_c__get_function__UpdateParams_Event__request(untyped_member, index));
  const zit6_interfaces__srv__UpdateParams_Request * value =
    (const zit6_interfaces__srv__UpdateParams_Request *)(untyped_value);
  *item = *value;
}

bool zit6_interfaces__srv__UpdateParams_Event__rosidl_typesupport_introspection_c__resize_function__UpdateParams_Event__request(
  void * untyped_member, size_t size)
{
  zit6_interfaces__srv__UpdateParams_Request__Sequence * member =
    (zit6_interfaces__srv__UpdateParams_Request__Sequence *)(untyped_member);
  zit6_interfaces__srv__UpdateParams_Request__Sequence__fini(member);
  return zit6_interfaces__srv__UpdateParams_Request__Sequence__init(member, size);
}

size_t zit6_interfaces__srv__UpdateParams_Event__rosidl_typesupport_introspection_c__size_function__UpdateParams_Event__response(
  const void * untyped_member)
{
  const zit6_interfaces__srv__UpdateParams_Response__Sequence * member =
    (const zit6_interfaces__srv__UpdateParams_Response__Sequence *)(untyped_member);
  return member->size;
}

const void * zit6_interfaces__srv__UpdateParams_Event__rosidl_typesupport_introspection_c__get_const_function__UpdateParams_Event__response(
  const void * untyped_member, size_t index)
{
  const zit6_interfaces__srv__UpdateParams_Response__Sequence * member =
    (const zit6_interfaces__srv__UpdateParams_Response__Sequence *)(untyped_member);
  return &member->data[index];
}

void * zit6_interfaces__srv__UpdateParams_Event__rosidl_typesupport_introspection_c__get_function__UpdateParams_Event__response(
  void * untyped_member, size_t index)
{
  zit6_interfaces__srv__UpdateParams_Response__Sequence * member =
    (zit6_interfaces__srv__UpdateParams_Response__Sequence *)(untyped_member);
  return &member->data[index];
}

void zit6_interfaces__srv__UpdateParams_Event__rosidl_typesupport_introspection_c__fetch_function__UpdateParams_Event__response(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const zit6_interfaces__srv__UpdateParams_Response * item =
    ((const zit6_interfaces__srv__UpdateParams_Response *)
    zit6_interfaces__srv__UpdateParams_Event__rosidl_typesupport_introspection_c__get_const_function__UpdateParams_Event__response(untyped_member, index));
  zit6_interfaces__srv__UpdateParams_Response * value =
    (zit6_interfaces__srv__UpdateParams_Response *)(untyped_value);
  *value = *item;
}

void zit6_interfaces__srv__UpdateParams_Event__rosidl_typesupport_introspection_c__assign_function__UpdateParams_Event__response(
  void * untyped_member, size_t index, const void * untyped_value)
{
  zit6_interfaces__srv__UpdateParams_Response * item =
    ((zit6_interfaces__srv__UpdateParams_Response *)
    zit6_interfaces__srv__UpdateParams_Event__rosidl_typesupport_introspection_c__get_function__UpdateParams_Event__response(untyped_member, index));
  const zit6_interfaces__srv__UpdateParams_Response * value =
    (const zit6_interfaces__srv__UpdateParams_Response *)(untyped_value);
  *item = *value;
}

bool zit6_interfaces__srv__UpdateParams_Event__rosidl_typesupport_introspection_c__resize_function__UpdateParams_Event__response(
  void * untyped_member, size_t size)
{
  zit6_interfaces__srv__UpdateParams_Response__Sequence * member =
    (zit6_interfaces__srv__UpdateParams_Response__Sequence *)(untyped_member);
  zit6_interfaces__srv__UpdateParams_Response__Sequence__fini(member);
  return zit6_interfaces__srv__UpdateParams_Response__Sequence__init(member, size);
}

static rosidl_typesupport_introspection_c__MessageMember zit6_interfaces__srv__UpdateParams_Event__rosidl_typesupport_introspection_c__UpdateParams_Event_message_member_array[3] = {
  {
    "info",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(zit6_interfaces__srv__UpdateParams_Event, info),  // bytes offset in struct
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
    offsetof(zit6_interfaces__srv__UpdateParams_Event, request),  // bytes offset in struct
    NULL,  // default value
    zit6_interfaces__srv__UpdateParams_Event__rosidl_typesupport_introspection_c__size_function__UpdateParams_Event__request,  // size() function pointer
    zit6_interfaces__srv__UpdateParams_Event__rosidl_typesupport_introspection_c__get_const_function__UpdateParams_Event__request,  // get_const(index) function pointer
    zit6_interfaces__srv__UpdateParams_Event__rosidl_typesupport_introspection_c__get_function__UpdateParams_Event__request,  // get(index) function pointer
    zit6_interfaces__srv__UpdateParams_Event__rosidl_typesupport_introspection_c__fetch_function__UpdateParams_Event__request,  // fetch(index, &value) function pointer
    zit6_interfaces__srv__UpdateParams_Event__rosidl_typesupport_introspection_c__assign_function__UpdateParams_Event__request,  // assign(index, value) function pointer
    zit6_interfaces__srv__UpdateParams_Event__rosidl_typesupport_introspection_c__resize_function__UpdateParams_Event__request  // resize(index) function pointer
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
    offsetof(zit6_interfaces__srv__UpdateParams_Event, response),  // bytes offset in struct
    NULL,  // default value
    zit6_interfaces__srv__UpdateParams_Event__rosidl_typesupport_introspection_c__size_function__UpdateParams_Event__response,  // size() function pointer
    zit6_interfaces__srv__UpdateParams_Event__rosidl_typesupport_introspection_c__get_const_function__UpdateParams_Event__response,  // get_const(index) function pointer
    zit6_interfaces__srv__UpdateParams_Event__rosidl_typesupport_introspection_c__get_function__UpdateParams_Event__response,  // get(index) function pointer
    zit6_interfaces__srv__UpdateParams_Event__rosidl_typesupport_introspection_c__fetch_function__UpdateParams_Event__response,  // fetch(index, &value) function pointer
    zit6_interfaces__srv__UpdateParams_Event__rosidl_typesupport_introspection_c__assign_function__UpdateParams_Event__response,  // assign(index, value) function pointer
    zit6_interfaces__srv__UpdateParams_Event__rosidl_typesupport_introspection_c__resize_function__UpdateParams_Event__response  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers zit6_interfaces__srv__UpdateParams_Event__rosidl_typesupport_introspection_c__UpdateParams_Event_message_members = {
  "zit6_interfaces__srv",  // message namespace
  "UpdateParams_Event",  // message name
  3,  // number of fields
  sizeof(zit6_interfaces__srv__UpdateParams_Event),
  false,  // has_any_key_member_
  zit6_interfaces__srv__UpdateParams_Event__rosidl_typesupport_introspection_c__UpdateParams_Event_message_member_array,  // message members
  zit6_interfaces__srv__UpdateParams_Event__rosidl_typesupport_introspection_c__UpdateParams_Event_init_function,  // function to initialize message memory (memory has to be allocated)
  zit6_interfaces__srv__UpdateParams_Event__rosidl_typesupport_introspection_c__UpdateParams_Event_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t zit6_interfaces__srv__UpdateParams_Event__rosidl_typesupport_introspection_c__UpdateParams_Event_message_type_support_handle = {
  0,
  &zit6_interfaces__srv__UpdateParams_Event__rosidl_typesupport_introspection_c__UpdateParams_Event_message_members,
  get_message_typesupport_handle_function,
  &zit6_interfaces__srv__UpdateParams_Event__get_type_hash,
  &zit6_interfaces__srv__UpdateParams_Event__get_type_description,
  &zit6_interfaces__srv__UpdateParams_Event__get_type_description_sources,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_zit6_interfaces
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, zit6_interfaces, srv, UpdateParams_Event)() {
  zit6_interfaces__srv__UpdateParams_Event__rosidl_typesupport_introspection_c__UpdateParams_Event_message_member_array[0].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, service_msgs, msg, ServiceEventInfo)();
  zit6_interfaces__srv__UpdateParams_Event__rosidl_typesupport_introspection_c__UpdateParams_Event_message_member_array[1].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, zit6_interfaces, srv, UpdateParams_Request)();
  zit6_interfaces__srv__UpdateParams_Event__rosidl_typesupport_introspection_c__UpdateParams_Event_message_member_array[2].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, zit6_interfaces, srv, UpdateParams_Response)();
  if (!zit6_interfaces__srv__UpdateParams_Event__rosidl_typesupport_introspection_c__UpdateParams_Event_message_type_support_handle.typesupport_identifier) {
    zit6_interfaces__srv__UpdateParams_Event__rosidl_typesupport_introspection_c__UpdateParams_Event_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &zit6_interfaces__srv__UpdateParams_Event__rosidl_typesupport_introspection_c__UpdateParams_Event_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif

#include "rosidl_runtime_c/service_type_support_struct.h"
// already included above
// #include "zit6_interfaces/msg/rosidl_typesupport_introspection_c__visibility_control.h"
// already included above
// #include "zit6_interfaces/srv/detail/update_params__rosidl_typesupport_introspection_c.h"
// already included above
// #include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/service_introspection.h"

// this is intentionally not const to allow initialization later to prevent an initialization race
static rosidl_typesupport_introspection_c__ServiceMembers zit6_interfaces__srv__detail__update_params__rosidl_typesupport_introspection_c__UpdateParams_service_members = {
  "zit6_interfaces__srv",  // service namespace
  "UpdateParams",  // service name
  // the following fields are initialized below on first access
  NULL,  // request message
  // zit6_interfaces__srv__detail__update_params__rosidl_typesupport_introspection_c__UpdateParams_Request_message_type_support_handle,
  NULL,  // response message
  // zit6_interfaces__srv__detail__update_params__rosidl_typesupport_introspection_c__UpdateParams_Response_message_type_support_handle
  NULL  // event_message
  // zit6_interfaces__srv__detail__update_params__rosidl_typesupport_introspection_c__UpdateParams_Response_message_type_support_handle
};


static rosidl_service_type_support_t zit6_interfaces__srv__detail__update_params__rosidl_typesupport_introspection_c__UpdateParams_service_type_support_handle = {
  0,
  &zit6_interfaces__srv__detail__update_params__rosidl_typesupport_introspection_c__UpdateParams_service_members,
  get_service_typesupport_handle_function,
  &zit6_interfaces__srv__UpdateParams_Request__rosidl_typesupport_introspection_c__UpdateParams_Request_message_type_support_handle,
  &zit6_interfaces__srv__UpdateParams_Response__rosidl_typesupport_introspection_c__UpdateParams_Response_message_type_support_handle,
  &zit6_interfaces__srv__UpdateParams_Event__rosidl_typesupport_introspection_c__UpdateParams_Event_message_type_support_handle,
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

// Forward declaration of message type support functions for service members
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, zit6_interfaces, srv, UpdateParams_Request)(void);

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, zit6_interfaces, srv, UpdateParams_Response)(void);

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, zit6_interfaces, srv, UpdateParams_Event)(void);

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_zit6_interfaces
const rosidl_service_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_SYMBOL_NAME(rosidl_typesupport_introspection_c, zit6_interfaces, srv, UpdateParams)(void) {
  if (!zit6_interfaces__srv__detail__update_params__rosidl_typesupport_introspection_c__UpdateParams_service_type_support_handle.typesupport_identifier) {
    zit6_interfaces__srv__detail__update_params__rosidl_typesupport_introspection_c__UpdateParams_service_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  rosidl_typesupport_introspection_c__ServiceMembers * service_members =
    (rosidl_typesupport_introspection_c__ServiceMembers *)zit6_interfaces__srv__detail__update_params__rosidl_typesupport_introspection_c__UpdateParams_service_type_support_handle.data;

  if (!service_members->request_members_) {
    service_members->request_members_ =
      (const rosidl_typesupport_introspection_c__MessageMembers *)
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, zit6_interfaces, srv, UpdateParams_Request)()->data;
  }
  if (!service_members->response_members_) {
    service_members->response_members_ =
      (const rosidl_typesupport_introspection_c__MessageMembers *)
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, zit6_interfaces, srv, UpdateParams_Response)()->data;
  }
  if (!service_members->event_members_) {
    service_members->event_members_ =
      (const rosidl_typesupport_introspection_c__MessageMembers *)
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, zit6_interfaces, srv, UpdateParams_Event)()->data;
  }

  return &zit6_interfaces__srv__detail__update_params__rosidl_typesupport_introspection_c__UpdateParams_service_type_support_handle;
}
