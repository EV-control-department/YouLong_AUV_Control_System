// generated from rosidl_typesupport_introspection_cpp/resource/idl__type_support.cpp.em
// with input from uv_msgs:msg/ObjectPositionArray.idl
// generated code does not contain a copyright notice

#include "array"
#include "cstddef"
#include "string"
#include "vector"
#include "rosidl_runtime_c/message_type_support_struct.h"
#include "rosidl_typesupport_cpp/message_type_support.hpp"
#include "rosidl_typesupport_interface/macros.h"
#include "uv_msgs/msg/detail/object_position_array__functions.h"
#include "uv_msgs/msg/detail/object_position_array__struct.hpp"
#include "rosidl_typesupport_introspection_cpp/field_types.hpp"
#include "rosidl_typesupport_introspection_cpp/identifier.hpp"
#include "rosidl_typesupport_introspection_cpp/message_introspection.hpp"
#include "rosidl_typesupport_introspection_cpp/message_type_support_decl.hpp"
#include "rosidl_typesupport_introspection_cpp/visibility_control.h"

namespace uv_msgs
{

namespace msg
{

namespace rosidl_typesupport_introspection_cpp
{

void ObjectPositionArray_init_function(
  void * message_memory, rosidl_runtime_cpp::MessageInitialization _init)
{
  new (message_memory) uv_msgs::msg::ObjectPositionArray(_init);
}

void ObjectPositionArray_fini_function(void * message_memory)
{
  auto typed_message = static_cast<uv_msgs::msg::ObjectPositionArray *>(message_memory);
  typed_message->~ObjectPositionArray();
}

size_t size_function__ObjectPositionArray__objects(const void * untyped_member)
{
  const auto * member = reinterpret_cast<const std::vector<uv_msgs::msg::ObjectPosition> *>(untyped_member);
  return member->size();
}

const void * get_const_function__ObjectPositionArray__objects(const void * untyped_member, size_t index)
{
  const auto & member =
    *reinterpret_cast<const std::vector<uv_msgs::msg::ObjectPosition> *>(untyped_member);
  return &member[index];
}

void * get_function__ObjectPositionArray__objects(void * untyped_member, size_t index)
{
  auto & member =
    *reinterpret_cast<std::vector<uv_msgs::msg::ObjectPosition> *>(untyped_member);
  return &member[index];
}

void fetch_function__ObjectPositionArray__objects(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const auto & item = *reinterpret_cast<const uv_msgs::msg::ObjectPosition *>(
    get_const_function__ObjectPositionArray__objects(untyped_member, index));
  auto & value = *reinterpret_cast<uv_msgs::msg::ObjectPosition *>(untyped_value);
  value = item;
}

void assign_function__ObjectPositionArray__objects(
  void * untyped_member, size_t index, const void * untyped_value)
{
  auto & item = *reinterpret_cast<uv_msgs::msg::ObjectPosition *>(
    get_function__ObjectPositionArray__objects(untyped_member, index));
  const auto & value = *reinterpret_cast<const uv_msgs::msg::ObjectPosition *>(untyped_value);
  item = value;
}

void resize_function__ObjectPositionArray__objects(void * untyped_member, size_t size)
{
  auto * member =
    reinterpret_cast<std::vector<uv_msgs::msg::ObjectPosition> *>(untyped_member);
  member->resize(size);
}

static const ::rosidl_typesupport_introspection_cpp::MessageMember ObjectPositionArray_message_member_array[2] = {
  {
    "header",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    ::rosidl_typesupport_introspection_cpp::get_message_type_support_handle<std_msgs::msg::Header>(),  // members of sub message
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(uv_msgs::msg::ObjectPositionArray, header),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr,  // fetch(index, &value) function pointer
    nullptr,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "objects",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    ::rosidl_typesupport_introspection_cpp::get_message_type_support_handle<uv_msgs::msg::ObjectPosition>(),  // members of sub message
    false,  // is key
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(uv_msgs::msg::ObjectPositionArray, objects),  // bytes offset in struct
    nullptr,  // default value
    size_function__ObjectPositionArray__objects,  // size() function pointer
    get_const_function__ObjectPositionArray__objects,  // get_const(index) function pointer
    get_function__ObjectPositionArray__objects,  // get(index) function pointer
    fetch_function__ObjectPositionArray__objects,  // fetch(index, &value) function pointer
    assign_function__ObjectPositionArray__objects,  // assign(index, value) function pointer
    resize_function__ObjectPositionArray__objects  // resize(index) function pointer
  }
};

static const ::rosidl_typesupport_introspection_cpp::MessageMembers ObjectPositionArray_message_members = {
  "uv_msgs::msg",  // message namespace
  "ObjectPositionArray",  // message name
  2,  // number of fields
  sizeof(uv_msgs::msg::ObjectPositionArray),
  false,  // has_any_key_member_
  ObjectPositionArray_message_member_array,  // message members
  ObjectPositionArray_init_function,  // function to initialize message memory (memory has to be allocated)
  ObjectPositionArray_fini_function  // function to terminate message instance (will not free memory)
};

static const rosidl_message_type_support_t ObjectPositionArray_message_type_support_handle = {
  ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  &ObjectPositionArray_message_members,
  get_message_typesupport_handle_function,
  &uv_msgs__msg__ObjectPositionArray__get_type_hash,
  &uv_msgs__msg__ObjectPositionArray__get_type_description,
  &uv_msgs__msg__ObjectPositionArray__get_type_description_sources,
};

}  // namespace rosidl_typesupport_introspection_cpp

}  // namespace msg

}  // namespace uv_msgs


namespace rosidl_typesupport_introspection_cpp
{

template<>
ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
get_message_type_support_handle<uv_msgs::msg::ObjectPositionArray>()
{
  return &::uv_msgs::msg::rosidl_typesupport_introspection_cpp::ObjectPositionArray_message_type_support_handle;
}

}  // namespace rosidl_typesupport_introspection_cpp

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, uv_msgs, msg, ObjectPositionArray)() {
  return &::uv_msgs::msg::rosidl_typesupport_introspection_cpp::ObjectPositionArray_message_type_support_handle;
}

#ifdef __cplusplus
}
#endif
