// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from zit6_interfaces:msg/ZitStatus.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "zit6_interfaces/msg/detail/zit_status__rosidl_typesupport_introspection_c.h"
#include "zit6_interfaces/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "zit6_interfaces/msg/detail/zit_status__functions.h"
#include "zit6_interfaces/msg/detail/zit_status__struct.h"


#ifdef __cplusplus
extern "C"
{
#endif

void zit6_interfaces__msg__ZitStatus__rosidl_typesupport_introspection_c__ZitStatus_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  zit6_interfaces__msg__ZitStatus__init(message_memory);
}

void zit6_interfaces__msg__ZitStatus__rosidl_typesupport_introspection_c__ZitStatus_fini_function(void * message_memory)
{
  zit6_interfaces__msg__ZitStatus__fini(message_memory);
}

size_t zit6_interfaces__msg__ZitStatus__rosidl_typesupport_introspection_c__size_function__ZitStatus__forces(
  const void * untyped_member)
{
  (void)untyped_member;
  return 4;
}

const void * zit6_interfaces__msg__ZitStatus__rosidl_typesupport_introspection_c__get_const_function__ZitStatus__forces(
  const void * untyped_member, size_t index)
{
  const float * member =
    (const float *)(untyped_member);
  return &member[index];
}

void * zit6_interfaces__msg__ZitStatus__rosidl_typesupport_introspection_c__get_function__ZitStatus__forces(
  void * untyped_member, size_t index)
{
  float * member =
    (float *)(untyped_member);
  return &member[index];
}

void zit6_interfaces__msg__ZitStatus__rosidl_typesupport_introspection_c__fetch_function__ZitStatus__forces(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const float * item =
    ((const float *)
    zit6_interfaces__msg__ZitStatus__rosidl_typesupport_introspection_c__get_const_function__ZitStatus__forces(untyped_member, index));
  float * value =
    (float *)(untyped_value);
  *value = *item;
}

void zit6_interfaces__msg__ZitStatus__rosidl_typesupport_introspection_c__assign_function__ZitStatus__forces(
  void * untyped_member, size_t index, const void * untyped_value)
{
  float * item =
    ((float *)
    zit6_interfaces__msg__ZitStatus__rosidl_typesupport_introspection_c__get_function__ZitStatus__forces(untyped_member, index));
  const float * value =
    (const float *)(untyped_value);
  *item = *value;
}

static rosidl_typesupport_introspection_c__MessageMember zit6_interfaces__msg__ZitStatus__rosidl_typesupport_introspection_c__ZitStatus_message_member_array[9] = {
  {
    "is_armed",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_BOOLEAN,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(zit6_interfaces__msg__ZitStatus, is_armed),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "arm_mode",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_UINT8,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(zit6_interfaces__msg__ZitStatus, arm_mode),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "control_level",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_UINT8,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(zit6_interfaces__msg__ZitStatus, control_level),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "ins_state",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_UINT8,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(zit6_interfaces__msg__ZitStatus, ins_state),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "navigation_ready",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_BOOLEAN,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(zit6_interfaces__msg__ZitStatus, navigation_ready),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "forces",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_FLOAT,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is key
    true,  // is array
    4,  // array size
    false,  // is upper bound
    offsetof(zit6_interfaces__msg__ZitStatus, forces),  // bytes offset in struct
    NULL,  // default value
    zit6_interfaces__msg__ZitStatus__rosidl_typesupport_introspection_c__size_function__ZitStatus__forces,  // size() function pointer
    zit6_interfaces__msg__ZitStatus__rosidl_typesupport_introspection_c__get_const_function__ZitStatus__forces,  // get_const(index) function pointer
    zit6_interfaces__msg__ZitStatus__rosidl_typesupport_introspection_c__get_function__ZitStatus__forces,  // get(index) function pointer
    zit6_interfaces__msg__ZitStatus__rosidl_typesupport_introspection_c__fetch_function__ZitStatus__forces,  // fetch(index, &value) function pointer
    zit6_interfaces__msg__ZitStatus__rosidl_typesupport_introspection_c__assign_function__ZitStatus__forces,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "cycle_time_ms",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_FLOAT,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(zit6_interfaces__msg__ZitStatus, cycle_time_ms),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "battery_voltage",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_FLOAT,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(zit6_interfaces__msg__ZitStatus, battery_voltage),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "error_flags",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_UINT32,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(zit6_interfaces__msg__ZitStatus, error_flags),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers zit6_interfaces__msg__ZitStatus__rosidl_typesupport_introspection_c__ZitStatus_message_members = {
  "zit6_interfaces__msg",  // message namespace
  "ZitStatus",  // message name
  9,  // number of fields
  sizeof(zit6_interfaces__msg__ZitStatus),
  false,  // has_any_key_member_
  zit6_interfaces__msg__ZitStatus__rosidl_typesupport_introspection_c__ZitStatus_message_member_array,  // message members
  zit6_interfaces__msg__ZitStatus__rosidl_typesupport_introspection_c__ZitStatus_init_function,  // function to initialize message memory (memory has to be allocated)
  zit6_interfaces__msg__ZitStatus__rosidl_typesupport_introspection_c__ZitStatus_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t zit6_interfaces__msg__ZitStatus__rosidl_typesupport_introspection_c__ZitStatus_message_type_support_handle = {
  0,
  &zit6_interfaces__msg__ZitStatus__rosidl_typesupport_introspection_c__ZitStatus_message_members,
  get_message_typesupport_handle_function,
  &zit6_interfaces__msg__ZitStatus__get_type_hash,
  &zit6_interfaces__msg__ZitStatus__get_type_description,
  &zit6_interfaces__msg__ZitStatus__get_type_description_sources,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_zit6_interfaces
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, zit6_interfaces, msg, ZitStatus)() {
  if (!zit6_interfaces__msg__ZitStatus__rosidl_typesupport_introspection_c__ZitStatus_message_type_support_handle.typesupport_identifier) {
    zit6_interfaces__msg__ZitStatus__rosidl_typesupport_introspection_c__ZitStatus_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &zit6_interfaces__msg__ZitStatus__rosidl_typesupport_introspection_c__ZitStatus_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif
