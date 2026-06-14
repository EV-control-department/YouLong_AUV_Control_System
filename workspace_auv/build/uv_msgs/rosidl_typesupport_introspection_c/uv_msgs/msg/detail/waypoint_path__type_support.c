// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from uv_msgs:msg/WaypointPath.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "uv_msgs/msg/detail/waypoint_path__rosidl_typesupport_introspection_c.h"
#include "uv_msgs/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "uv_msgs/msg/detail/waypoint_path__functions.h"
#include "uv_msgs/msg/detail/waypoint_path__struct.h"


// Include directives for member types
// Member `header`
#include "std_msgs/msg/header.h"
// Member `header`
#include "std_msgs/msg/detail/header__rosidl_typesupport_introspection_c.h"
// Member `waypoints`
#include "uv_msgs/msg/waypoint.h"
// Member `waypoints`
#include "uv_msgs/msg/detail/waypoint__rosidl_typesupport_introspection_c.h"

#ifdef __cplusplus
extern "C"
{
#endif

void uv_msgs__msg__WaypointPath__rosidl_typesupport_introspection_c__WaypointPath_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  uv_msgs__msg__WaypointPath__init(message_memory);
}

void uv_msgs__msg__WaypointPath__rosidl_typesupport_introspection_c__WaypointPath_fini_function(void * message_memory)
{
  uv_msgs__msg__WaypointPath__fini(message_memory);
}

size_t uv_msgs__msg__WaypointPath__rosidl_typesupport_introspection_c__size_function__WaypointPath__waypoints(
  const void * untyped_member)
{
  const uv_msgs__msg__Waypoint__Sequence * member =
    (const uv_msgs__msg__Waypoint__Sequence *)(untyped_member);
  return member->size;
}

const void * uv_msgs__msg__WaypointPath__rosidl_typesupport_introspection_c__get_const_function__WaypointPath__waypoints(
  const void * untyped_member, size_t index)
{
  const uv_msgs__msg__Waypoint__Sequence * member =
    (const uv_msgs__msg__Waypoint__Sequence *)(untyped_member);
  return &member->data[index];
}

void * uv_msgs__msg__WaypointPath__rosidl_typesupport_introspection_c__get_function__WaypointPath__waypoints(
  void * untyped_member, size_t index)
{
  uv_msgs__msg__Waypoint__Sequence * member =
    (uv_msgs__msg__Waypoint__Sequence *)(untyped_member);
  return &member->data[index];
}

void uv_msgs__msg__WaypointPath__rosidl_typesupport_introspection_c__fetch_function__WaypointPath__waypoints(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const uv_msgs__msg__Waypoint * item =
    ((const uv_msgs__msg__Waypoint *)
    uv_msgs__msg__WaypointPath__rosidl_typesupport_introspection_c__get_const_function__WaypointPath__waypoints(untyped_member, index));
  uv_msgs__msg__Waypoint * value =
    (uv_msgs__msg__Waypoint *)(untyped_value);
  *value = *item;
}

void uv_msgs__msg__WaypointPath__rosidl_typesupport_introspection_c__assign_function__WaypointPath__waypoints(
  void * untyped_member, size_t index, const void * untyped_value)
{
  uv_msgs__msg__Waypoint * item =
    ((uv_msgs__msg__Waypoint *)
    uv_msgs__msg__WaypointPath__rosidl_typesupport_introspection_c__get_function__WaypointPath__waypoints(untyped_member, index));
  const uv_msgs__msg__Waypoint * value =
    (const uv_msgs__msg__Waypoint *)(untyped_value);
  *item = *value;
}

bool uv_msgs__msg__WaypointPath__rosidl_typesupport_introspection_c__resize_function__WaypointPath__waypoints(
  void * untyped_member, size_t size)
{
  uv_msgs__msg__Waypoint__Sequence * member =
    (uv_msgs__msg__Waypoint__Sequence *)(untyped_member);
  uv_msgs__msg__Waypoint__Sequence__fini(member);
  return uv_msgs__msg__Waypoint__Sequence__init(member, size);
}

static rosidl_typesupport_introspection_c__MessageMember uv_msgs__msg__WaypointPath__rosidl_typesupport_introspection_c__WaypointPath_message_member_array[2] = {
  {
    "header",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(uv_msgs__msg__WaypointPath, header),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "waypoints",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is key
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(uv_msgs__msg__WaypointPath, waypoints),  // bytes offset in struct
    NULL,  // default value
    uv_msgs__msg__WaypointPath__rosidl_typesupport_introspection_c__size_function__WaypointPath__waypoints,  // size() function pointer
    uv_msgs__msg__WaypointPath__rosidl_typesupport_introspection_c__get_const_function__WaypointPath__waypoints,  // get_const(index) function pointer
    uv_msgs__msg__WaypointPath__rosidl_typesupport_introspection_c__get_function__WaypointPath__waypoints,  // get(index) function pointer
    uv_msgs__msg__WaypointPath__rosidl_typesupport_introspection_c__fetch_function__WaypointPath__waypoints,  // fetch(index, &value) function pointer
    uv_msgs__msg__WaypointPath__rosidl_typesupport_introspection_c__assign_function__WaypointPath__waypoints,  // assign(index, value) function pointer
    uv_msgs__msg__WaypointPath__rosidl_typesupport_introspection_c__resize_function__WaypointPath__waypoints  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers uv_msgs__msg__WaypointPath__rosidl_typesupport_introspection_c__WaypointPath_message_members = {
  "uv_msgs__msg",  // message namespace
  "WaypointPath",  // message name
  2,  // number of fields
  sizeof(uv_msgs__msg__WaypointPath),
  false,  // has_any_key_member_
  uv_msgs__msg__WaypointPath__rosidl_typesupport_introspection_c__WaypointPath_message_member_array,  // message members
  uv_msgs__msg__WaypointPath__rosidl_typesupport_introspection_c__WaypointPath_init_function,  // function to initialize message memory (memory has to be allocated)
  uv_msgs__msg__WaypointPath__rosidl_typesupport_introspection_c__WaypointPath_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t uv_msgs__msg__WaypointPath__rosidl_typesupport_introspection_c__WaypointPath_message_type_support_handle = {
  0,
  &uv_msgs__msg__WaypointPath__rosidl_typesupport_introspection_c__WaypointPath_message_members,
  get_message_typesupport_handle_function,
  &uv_msgs__msg__WaypointPath__get_type_hash,
  &uv_msgs__msg__WaypointPath__get_type_description,
  &uv_msgs__msg__WaypointPath__get_type_description_sources,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_uv_msgs
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, uv_msgs, msg, WaypointPath)() {
  uv_msgs__msg__WaypointPath__rosidl_typesupport_introspection_c__WaypointPath_message_member_array[0].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, std_msgs, msg, Header)();
  uv_msgs__msg__WaypointPath__rosidl_typesupport_introspection_c__WaypointPath_message_member_array[1].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, uv_msgs, msg, Waypoint)();
  if (!uv_msgs__msg__WaypointPath__rosidl_typesupport_introspection_c__WaypointPath_message_type_support_handle.typesupport_identifier) {
    uv_msgs__msg__WaypointPath__rosidl_typesupport_introspection_c__WaypointPath_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &uv_msgs__msg__WaypointPath__rosidl_typesupport_introspection_c__WaypointPath_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif
