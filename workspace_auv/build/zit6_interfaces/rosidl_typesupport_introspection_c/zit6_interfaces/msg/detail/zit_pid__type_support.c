// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from zit6_interfaces:msg/ZitPid.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "zit6_interfaces/msg/detail/zit_pid__rosidl_typesupport_introspection_c.h"
#include "zit6_interfaces/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "zit6_interfaces/msg/detail/zit_pid__functions.h"
#include "zit6_interfaces/msg/detail/zit_pid__struct.h"


#ifdef __cplusplus
extern "C"
{
#endif

void zit6_interfaces__msg__ZitPid__rosidl_typesupport_introspection_c__ZitPid_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  zit6_interfaces__msg__ZitPid__init(message_memory);
}

void zit6_interfaces__msg__ZitPid__rosidl_typesupport_introspection_c__ZitPid_fini_function(void * message_memory)
{
  zit6_interfaces__msg__ZitPid__fini(message_memory);
}

static rosidl_typesupport_introspection_c__MessageMember zit6_interfaces__msg__ZitPid__rosidl_typesupport_introspection_c__ZitPid_message_member_array[9] = {
  {
    "axis",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_UINT8,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(zit6_interfaces__msg__ZitPid, axis),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "is_pos_ring",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_BOOLEAN,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(zit6_interfaces__msg__ZitPid, is_pos_ring),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "kp",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_FLOAT,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(zit6_interfaces__msg__ZitPid, kp),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "ki",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_FLOAT,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(zit6_interfaces__msg__ZitPid, ki),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "kd",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_FLOAT,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(zit6_interfaces__msg__ZitPid, kd),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "i_limit",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_FLOAT,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(zit6_interfaces__msg__ZitPid, i_limit),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "out_limit",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_FLOAT,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(zit6_interfaces__msg__ZitPid, out_limit),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "max_v",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_FLOAT,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(zit6_interfaces__msg__ZitPid, max_v),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "max_a",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_FLOAT,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(zit6_interfaces__msg__ZitPid, max_a),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers zit6_interfaces__msg__ZitPid__rosidl_typesupport_introspection_c__ZitPid_message_members = {
  "zit6_interfaces__msg",  // message namespace
  "ZitPid",  // message name
  9,  // number of fields
  sizeof(zit6_interfaces__msg__ZitPid),
  false,  // has_any_key_member_
  zit6_interfaces__msg__ZitPid__rosidl_typesupport_introspection_c__ZitPid_message_member_array,  // message members
  zit6_interfaces__msg__ZitPid__rosidl_typesupport_introspection_c__ZitPid_init_function,  // function to initialize message memory (memory has to be allocated)
  zit6_interfaces__msg__ZitPid__rosidl_typesupport_introspection_c__ZitPid_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t zit6_interfaces__msg__ZitPid__rosidl_typesupport_introspection_c__ZitPid_message_type_support_handle = {
  0,
  &zit6_interfaces__msg__ZitPid__rosidl_typesupport_introspection_c__ZitPid_message_members,
  get_message_typesupport_handle_function,
  &zit6_interfaces__msg__ZitPid__get_type_hash,
  &zit6_interfaces__msg__ZitPid__get_type_description,
  &zit6_interfaces__msg__ZitPid__get_type_description_sources,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_zit6_interfaces
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, zit6_interfaces, msg, ZitPid)() {
  if (!zit6_interfaces__msg__ZitPid__rosidl_typesupport_introspection_c__ZitPid_message_type_support_handle.typesupport_identifier) {
    zit6_interfaces__msg__ZitPid__rosidl_typesupport_introspection_c__ZitPid_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &zit6_interfaces__msg__ZitPid__rosidl_typesupport_introspection_c__ZitPid_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif
