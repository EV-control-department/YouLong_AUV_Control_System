// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from uv_msgs:msg/ObjectPositionArray.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "uv_msgs/msg/detail/object_position_array__rosidl_typesupport_introspection_c.h"
#include "uv_msgs/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "uv_msgs/msg/detail/object_position_array__functions.h"
#include "uv_msgs/msg/detail/object_position_array__struct.h"


// Include directives for member types
// Member `header`
#include "std_msgs/msg/header.h"
// Member `header`
#include "std_msgs/msg/detail/header__rosidl_typesupport_introspection_c.h"
// Member `objects`
#include "uv_msgs/msg/object_position.h"
// Member `objects`
#include "uv_msgs/msg/detail/object_position__rosidl_typesupport_introspection_c.h"

#ifdef __cplusplus
extern "C"
{
#endif

void uv_msgs__msg__ObjectPositionArray__rosidl_typesupport_introspection_c__ObjectPositionArray_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  uv_msgs__msg__ObjectPositionArray__init(message_memory);
}

void uv_msgs__msg__ObjectPositionArray__rosidl_typesupport_introspection_c__ObjectPositionArray_fini_function(void * message_memory)
{
  uv_msgs__msg__ObjectPositionArray__fini(message_memory);
}

size_t uv_msgs__msg__ObjectPositionArray__rosidl_typesupport_introspection_c__size_function__ObjectPositionArray__objects(
  const void * untyped_member)
{
  const uv_msgs__msg__ObjectPosition__Sequence * member =
    (const uv_msgs__msg__ObjectPosition__Sequence *)(untyped_member);
  return member->size;
}

const void * uv_msgs__msg__ObjectPositionArray__rosidl_typesupport_introspection_c__get_const_function__ObjectPositionArray__objects(
  const void * untyped_member, size_t index)
{
  const uv_msgs__msg__ObjectPosition__Sequence * member =
    (const uv_msgs__msg__ObjectPosition__Sequence *)(untyped_member);
  return &member->data[index];
}

void * uv_msgs__msg__ObjectPositionArray__rosidl_typesupport_introspection_c__get_function__ObjectPositionArray__objects(
  void * untyped_member, size_t index)
{
  uv_msgs__msg__ObjectPosition__Sequence * member =
    (uv_msgs__msg__ObjectPosition__Sequence *)(untyped_member);
  return &member->data[index];
}

void uv_msgs__msg__ObjectPositionArray__rosidl_typesupport_introspection_c__fetch_function__ObjectPositionArray__objects(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const uv_msgs__msg__ObjectPosition * item =
    ((const uv_msgs__msg__ObjectPosition *)
    uv_msgs__msg__ObjectPositionArray__rosidl_typesupport_introspection_c__get_const_function__ObjectPositionArray__objects(untyped_member, index));
  uv_msgs__msg__ObjectPosition * value =
    (uv_msgs__msg__ObjectPosition *)(untyped_value);
  *value = *item;
}

void uv_msgs__msg__ObjectPositionArray__rosidl_typesupport_introspection_c__assign_function__ObjectPositionArray__objects(
  void * untyped_member, size_t index, const void * untyped_value)
{
  uv_msgs__msg__ObjectPosition * item =
    ((uv_msgs__msg__ObjectPosition *)
    uv_msgs__msg__ObjectPositionArray__rosidl_typesupport_introspection_c__get_function__ObjectPositionArray__objects(untyped_member, index));
  const uv_msgs__msg__ObjectPosition * value =
    (const uv_msgs__msg__ObjectPosition *)(untyped_value);
  *item = *value;
}

bool uv_msgs__msg__ObjectPositionArray__rosidl_typesupport_introspection_c__resize_function__ObjectPositionArray__objects(
  void * untyped_member, size_t size)
{
  uv_msgs__msg__ObjectPosition__Sequence * member =
    (uv_msgs__msg__ObjectPosition__Sequence *)(untyped_member);
  uv_msgs__msg__ObjectPosition__Sequence__fini(member);
  return uv_msgs__msg__ObjectPosition__Sequence__init(member, size);
}

static rosidl_typesupport_introspection_c__MessageMember uv_msgs__msg__ObjectPositionArray__rosidl_typesupport_introspection_c__ObjectPositionArray_message_member_array[2] = {
  {
    "header",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(uv_msgs__msg__ObjectPositionArray, header),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "objects",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is key
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(uv_msgs__msg__ObjectPositionArray, objects),  // bytes offset in struct
    NULL,  // default value
    uv_msgs__msg__ObjectPositionArray__rosidl_typesupport_introspection_c__size_function__ObjectPositionArray__objects,  // size() function pointer
    uv_msgs__msg__ObjectPositionArray__rosidl_typesupport_introspection_c__get_const_function__ObjectPositionArray__objects,  // get_const(index) function pointer
    uv_msgs__msg__ObjectPositionArray__rosidl_typesupport_introspection_c__get_function__ObjectPositionArray__objects,  // get(index) function pointer
    uv_msgs__msg__ObjectPositionArray__rosidl_typesupport_introspection_c__fetch_function__ObjectPositionArray__objects,  // fetch(index, &value) function pointer
    uv_msgs__msg__ObjectPositionArray__rosidl_typesupport_introspection_c__assign_function__ObjectPositionArray__objects,  // assign(index, value) function pointer
    uv_msgs__msg__ObjectPositionArray__rosidl_typesupport_introspection_c__resize_function__ObjectPositionArray__objects  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers uv_msgs__msg__ObjectPositionArray__rosidl_typesupport_introspection_c__ObjectPositionArray_message_members = {
  "uv_msgs__msg",  // message namespace
  "ObjectPositionArray",  // message name
  2,  // number of fields
  sizeof(uv_msgs__msg__ObjectPositionArray),
  false,  // has_any_key_member_
  uv_msgs__msg__ObjectPositionArray__rosidl_typesupport_introspection_c__ObjectPositionArray_message_member_array,  // message members
  uv_msgs__msg__ObjectPositionArray__rosidl_typesupport_introspection_c__ObjectPositionArray_init_function,  // function to initialize message memory (memory has to be allocated)
  uv_msgs__msg__ObjectPositionArray__rosidl_typesupport_introspection_c__ObjectPositionArray_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t uv_msgs__msg__ObjectPositionArray__rosidl_typesupport_introspection_c__ObjectPositionArray_message_type_support_handle = {
  0,
  &uv_msgs__msg__ObjectPositionArray__rosidl_typesupport_introspection_c__ObjectPositionArray_message_members,
  get_message_typesupport_handle_function,
  &uv_msgs__msg__ObjectPositionArray__get_type_hash,
  &uv_msgs__msg__ObjectPositionArray__get_type_description,
  &uv_msgs__msg__ObjectPositionArray__get_type_description_sources,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_uv_msgs
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, uv_msgs, msg, ObjectPositionArray)() {
  uv_msgs__msg__ObjectPositionArray__rosidl_typesupport_introspection_c__ObjectPositionArray_message_member_array[0].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, std_msgs, msg, Header)();
  uv_msgs__msg__ObjectPositionArray__rosidl_typesupport_introspection_c__ObjectPositionArray_message_member_array[1].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, uv_msgs, msg, ObjectPosition)();
  if (!uv_msgs__msg__ObjectPositionArray__rosidl_typesupport_introspection_c__ObjectPositionArray_message_type_support_handle.typesupport_identifier) {
    uv_msgs__msg__ObjectPositionArray__rosidl_typesupport_introspection_c__ObjectPositionArray_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &uv_msgs__msg__ObjectPositionArray__rosidl_typesupport_introspection_c__ObjectPositionArray_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif
