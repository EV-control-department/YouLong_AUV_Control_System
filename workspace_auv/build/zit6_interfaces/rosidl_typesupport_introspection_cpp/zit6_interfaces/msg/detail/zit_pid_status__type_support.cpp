// generated from rosidl_typesupport_introspection_cpp/resource/idl__type_support.cpp.em
// with input from zit6_interfaces:msg/ZitPidStatus.idl
// generated code does not contain a copyright notice

#include "array"
#include "cstddef"
#include "string"
#include "vector"
#include "rosidl_runtime_c/message_type_support_struct.h"
#include "rosidl_typesupport_cpp/message_type_support.hpp"
#include "rosidl_typesupport_interface/macros.h"
#include "zit6_interfaces/msg/detail/zit_pid_status__functions.h"
#include "zit6_interfaces/msg/detail/zit_pid_status__struct.hpp"
#include "rosidl_typesupport_introspection_cpp/field_types.hpp"
#include "rosidl_typesupport_introspection_cpp/identifier.hpp"
#include "rosidl_typesupport_introspection_cpp/message_introspection.hpp"
#include "rosidl_typesupport_introspection_cpp/message_type_support_decl.hpp"
#include "rosidl_typesupport_introspection_cpp/visibility_control.h"

namespace zit6_interfaces
{

namespace msg
{

namespace rosidl_typesupport_introspection_cpp
{

void ZitPidStatus_init_function(
  void * message_memory, rosidl_runtime_cpp::MessageInitialization _init)
{
  new (message_memory) zit6_interfaces::msg::ZitPidStatus(_init);
}

void ZitPidStatus_fini_function(void * message_memory)
{
  auto typed_message = static_cast<zit6_interfaces::msg::ZitPidStatus *>(message_memory);
  typed_message->~ZitPidStatus();
}

size_t size_function__ZitPidStatus__pos_kp(const void * untyped_member)
{
  (void)untyped_member;
  return 4;
}

const void * get_const_function__ZitPidStatus__pos_kp(const void * untyped_member, size_t index)
{
  const auto & member =
    *reinterpret_cast<const std::array<float, 4> *>(untyped_member);
  return &member[index];
}

void * get_function__ZitPidStatus__pos_kp(void * untyped_member, size_t index)
{
  auto & member =
    *reinterpret_cast<std::array<float, 4> *>(untyped_member);
  return &member[index];
}

void fetch_function__ZitPidStatus__pos_kp(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const auto & item = *reinterpret_cast<const float *>(
    get_const_function__ZitPidStatus__pos_kp(untyped_member, index));
  auto & value = *reinterpret_cast<float *>(untyped_value);
  value = item;
}

void assign_function__ZitPidStatus__pos_kp(
  void * untyped_member, size_t index, const void * untyped_value)
{
  auto & item = *reinterpret_cast<float *>(
    get_function__ZitPidStatus__pos_kp(untyped_member, index));
  const auto & value = *reinterpret_cast<const float *>(untyped_value);
  item = value;
}

size_t size_function__ZitPidStatus__pos_max_v(const void * untyped_member)
{
  (void)untyped_member;
  return 4;
}

const void * get_const_function__ZitPidStatus__pos_max_v(const void * untyped_member, size_t index)
{
  const auto & member =
    *reinterpret_cast<const std::array<float, 4> *>(untyped_member);
  return &member[index];
}

void * get_function__ZitPidStatus__pos_max_v(void * untyped_member, size_t index)
{
  auto & member =
    *reinterpret_cast<std::array<float, 4> *>(untyped_member);
  return &member[index];
}

void fetch_function__ZitPidStatus__pos_max_v(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const auto & item = *reinterpret_cast<const float *>(
    get_const_function__ZitPidStatus__pos_max_v(untyped_member, index));
  auto & value = *reinterpret_cast<float *>(untyped_value);
  value = item;
}

void assign_function__ZitPidStatus__pos_max_v(
  void * untyped_member, size_t index, const void * untyped_value)
{
  auto & item = *reinterpret_cast<float *>(
    get_function__ZitPidStatus__pos_max_v(untyped_member, index));
  const auto & value = *reinterpret_cast<const float *>(untyped_value);
  item = value;
}

size_t size_function__ZitPidStatus__pos_max_a(const void * untyped_member)
{
  (void)untyped_member;
  return 4;
}

const void * get_const_function__ZitPidStatus__pos_max_a(const void * untyped_member, size_t index)
{
  const auto & member =
    *reinterpret_cast<const std::array<float, 4> *>(untyped_member);
  return &member[index];
}

void * get_function__ZitPidStatus__pos_max_a(void * untyped_member, size_t index)
{
  auto & member =
    *reinterpret_cast<std::array<float, 4> *>(untyped_member);
  return &member[index];
}

void fetch_function__ZitPidStatus__pos_max_a(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const auto & item = *reinterpret_cast<const float *>(
    get_const_function__ZitPidStatus__pos_max_a(untyped_member, index));
  auto & value = *reinterpret_cast<float *>(untyped_value);
  value = item;
}

void assign_function__ZitPidStatus__pos_max_a(
  void * untyped_member, size_t index, const void * untyped_value)
{
  auto & item = *reinterpret_cast<float *>(
    get_function__ZitPidStatus__pos_max_a(untyped_member, index));
  const auto & value = *reinterpret_cast<const float *>(untyped_value);
  item = value;
}

size_t size_function__ZitPidStatus__pos_out_limit(const void * untyped_member)
{
  (void)untyped_member;
  return 4;
}

const void * get_const_function__ZitPidStatus__pos_out_limit(const void * untyped_member, size_t index)
{
  const auto & member =
    *reinterpret_cast<const std::array<float, 4> *>(untyped_member);
  return &member[index];
}

void * get_function__ZitPidStatus__pos_out_limit(void * untyped_member, size_t index)
{
  auto & member =
    *reinterpret_cast<std::array<float, 4> *>(untyped_member);
  return &member[index];
}

void fetch_function__ZitPidStatus__pos_out_limit(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const auto & item = *reinterpret_cast<const float *>(
    get_const_function__ZitPidStatus__pos_out_limit(untyped_member, index));
  auto & value = *reinterpret_cast<float *>(untyped_value);
  value = item;
}

void assign_function__ZitPidStatus__pos_out_limit(
  void * untyped_member, size_t index, const void * untyped_value)
{
  auto & item = *reinterpret_cast<float *>(
    get_function__ZitPidStatus__pos_out_limit(untyped_member, index));
  const auto & value = *reinterpret_cast<const float *>(untyped_value);
  item = value;
}

size_t size_function__ZitPidStatus__vel_kp(const void * untyped_member)
{
  (void)untyped_member;
  return 4;
}

const void * get_const_function__ZitPidStatus__vel_kp(const void * untyped_member, size_t index)
{
  const auto & member =
    *reinterpret_cast<const std::array<float, 4> *>(untyped_member);
  return &member[index];
}

void * get_function__ZitPidStatus__vel_kp(void * untyped_member, size_t index)
{
  auto & member =
    *reinterpret_cast<std::array<float, 4> *>(untyped_member);
  return &member[index];
}

void fetch_function__ZitPidStatus__vel_kp(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const auto & item = *reinterpret_cast<const float *>(
    get_const_function__ZitPidStatus__vel_kp(untyped_member, index));
  auto & value = *reinterpret_cast<float *>(untyped_value);
  value = item;
}

void assign_function__ZitPidStatus__vel_kp(
  void * untyped_member, size_t index, const void * untyped_value)
{
  auto & item = *reinterpret_cast<float *>(
    get_function__ZitPidStatus__vel_kp(untyped_member, index));
  const auto & value = *reinterpret_cast<const float *>(untyped_value);
  item = value;
}

size_t size_function__ZitPidStatus__vel_ki(const void * untyped_member)
{
  (void)untyped_member;
  return 4;
}

const void * get_const_function__ZitPidStatus__vel_ki(const void * untyped_member, size_t index)
{
  const auto & member =
    *reinterpret_cast<const std::array<float, 4> *>(untyped_member);
  return &member[index];
}

void * get_function__ZitPidStatus__vel_ki(void * untyped_member, size_t index)
{
  auto & member =
    *reinterpret_cast<std::array<float, 4> *>(untyped_member);
  return &member[index];
}

void fetch_function__ZitPidStatus__vel_ki(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const auto & item = *reinterpret_cast<const float *>(
    get_const_function__ZitPidStatus__vel_ki(untyped_member, index));
  auto & value = *reinterpret_cast<float *>(untyped_value);
  value = item;
}

void assign_function__ZitPidStatus__vel_ki(
  void * untyped_member, size_t index, const void * untyped_value)
{
  auto & item = *reinterpret_cast<float *>(
    get_function__ZitPidStatus__vel_ki(untyped_member, index));
  const auto & value = *reinterpret_cast<const float *>(untyped_value);
  item = value;
}

size_t size_function__ZitPidStatus__vel_kd(const void * untyped_member)
{
  (void)untyped_member;
  return 4;
}

const void * get_const_function__ZitPidStatus__vel_kd(const void * untyped_member, size_t index)
{
  const auto & member =
    *reinterpret_cast<const std::array<float, 4> *>(untyped_member);
  return &member[index];
}

void * get_function__ZitPidStatus__vel_kd(void * untyped_member, size_t index)
{
  auto & member =
    *reinterpret_cast<std::array<float, 4> *>(untyped_member);
  return &member[index];
}

void fetch_function__ZitPidStatus__vel_kd(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const auto & item = *reinterpret_cast<const float *>(
    get_const_function__ZitPidStatus__vel_kd(untyped_member, index));
  auto & value = *reinterpret_cast<float *>(untyped_value);
  value = item;
}

void assign_function__ZitPidStatus__vel_kd(
  void * untyped_member, size_t index, const void * untyped_value)
{
  auto & item = *reinterpret_cast<float *>(
    get_function__ZitPidStatus__vel_kd(untyped_member, index));
  const auto & value = *reinterpret_cast<const float *>(untyped_value);
  item = value;
}

size_t size_function__ZitPidStatus__vel_i_limit(const void * untyped_member)
{
  (void)untyped_member;
  return 4;
}

const void * get_const_function__ZitPidStatus__vel_i_limit(const void * untyped_member, size_t index)
{
  const auto & member =
    *reinterpret_cast<const std::array<float, 4> *>(untyped_member);
  return &member[index];
}

void * get_function__ZitPidStatus__vel_i_limit(void * untyped_member, size_t index)
{
  auto & member =
    *reinterpret_cast<std::array<float, 4> *>(untyped_member);
  return &member[index];
}

void fetch_function__ZitPidStatus__vel_i_limit(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const auto & item = *reinterpret_cast<const float *>(
    get_const_function__ZitPidStatus__vel_i_limit(untyped_member, index));
  auto & value = *reinterpret_cast<float *>(untyped_value);
  value = item;
}

void assign_function__ZitPidStatus__vel_i_limit(
  void * untyped_member, size_t index, const void * untyped_value)
{
  auto & item = *reinterpret_cast<float *>(
    get_function__ZitPidStatus__vel_i_limit(untyped_member, index));
  const auto & value = *reinterpret_cast<const float *>(untyped_value);
  item = value;
}

size_t size_function__ZitPidStatus__vel_out_limit(const void * untyped_member)
{
  (void)untyped_member;
  return 4;
}

const void * get_const_function__ZitPidStatus__vel_out_limit(const void * untyped_member, size_t index)
{
  const auto & member =
    *reinterpret_cast<const std::array<float, 4> *>(untyped_member);
  return &member[index];
}

void * get_function__ZitPidStatus__vel_out_limit(void * untyped_member, size_t index)
{
  auto & member =
    *reinterpret_cast<std::array<float, 4> *>(untyped_member);
  return &member[index];
}

void fetch_function__ZitPidStatus__vel_out_limit(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const auto & item = *reinterpret_cast<const float *>(
    get_const_function__ZitPidStatus__vel_out_limit(untyped_member, index));
  auto & value = *reinterpret_cast<float *>(untyped_value);
  value = item;
}

void assign_function__ZitPidStatus__vel_out_limit(
  void * untyped_member, size_t index, const void * untyped_value)
{
  auto & item = *reinterpret_cast<float *>(
    get_function__ZitPidStatus__vel_out_limit(untyped_member, index));
  const auto & value = *reinterpret_cast<const float *>(untyped_value);
  item = value;
}

static const ::rosidl_typesupport_introspection_cpp::MessageMember ZitPidStatus_message_member_array[9] = {
  {
    "pos_kp",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_FLOAT,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    false,  // is key
    true,  // is array
    4,  // array size
    false,  // is upper bound
    offsetof(zit6_interfaces::msg::ZitPidStatus, pos_kp),  // bytes offset in struct
    nullptr,  // default value
    size_function__ZitPidStatus__pos_kp,  // size() function pointer
    get_const_function__ZitPidStatus__pos_kp,  // get_const(index) function pointer
    get_function__ZitPidStatus__pos_kp,  // get(index) function pointer
    fetch_function__ZitPidStatus__pos_kp,  // fetch(index, &value) function pointer
    assign_function__ZitPidStatus__pos_kp,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "pos_max_v",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_FLOAT,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    false,  // is key
    true,  // is array
    4,  // array size
    false,  // is upper bound
    offsetof(zit6_interfaces::msg::ZitPidStatus, pos_max_v),  // bytes offset in struct
    nullptr,  // default value
    size_function__ZitPidStatus__pos_max_v,  // size() function pointer
    get_const_function__ZitPidStatus__pos_max_v,  // get_const(index) function pointer
    get_function__ZitPidStatus__pos_max_v,  // get(index) function pointer
    fetch_function__ZitPidStatus__pos_max_v,  // fetch(index, &value) function pointer
    assign_function__ZitPidStatus__pos_max_v,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "pos_max_a",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_FLOAT,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    false,  // is key
    true,  // is array
    4,  // array size
    false,  // is upper bound
    offsetof(zit6_interfaces::msg::ZitPidStatus, pos_max_a),  // bytes offset in struct
    nullptr,  // default value
    size_function__ZitPidStatus__pos_max_a,  // size() function pointer
    get_const_function__ZitPidStatus__pos_max_a,  // get_const(index) function pointer
    get_function__ZitPidStatus__pos_max_a,  // get(index) function pointer
    fetch_function__ZitPidStatus__pos_max_a,  // fetch(index, &value) function pointer
    assign_function__ZitPidStatus__pos_max_a,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "pos_out_limit",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_FLOAT,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    false,  // is key
    true,  // is array
    4,  // array size
    false,  // is upper bound
    offsetof(zit6_interfaces::msg::ZitPidStatus, pos_out_limit),  // bytes offset in struct
    nullptr,  // default value
    size_function__ZitPidStatus__pos_out_limit,  // size() function pointer
    get_const_function__ZitPidStatus__pos_out_limit,  // get_const(index) function pointer
    get_function__ZitPidStatus__pos_out_limit,  // get(index) function pointer
    fetch_function__ZitPidStatus__pos_out_limit,  // fetch(index, &value) function pointer
    assign_function__ZitPidStatus__pos_out_limit,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "vel_kp",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_FLOAT,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    false,  // is key
    true,  // is array
    4,  // array size
    false,  // is upper bound
    offsetof(zit6_interfaces::msg::ZitPidStatus, vel_kp),  // bytes offset in struct
    nullptr,  // default value
    size_function__ZitPidStatus__vel_kp,  // size() function pointer
    get_const_function__ZitPidStatus__vel_kp,  // get_const(index) function pointer
    get_function__ZitPidStatus__vel_kp,  // get(index) function pointer
    fetch_function__ZitPidStatus__vel_kp,  // fetch(index, &value) function pointer
    assign_function__ZitPidStatus__vel_kp,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "vel_ki",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_FLOAT,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    false,  // is key
    true,  // is array
    4,  // array size
    false,  // is upper bound
    offsetof(zit6_interfaces::msg::ZitPidStatus, vel_ki),  // bytes offset in struct
    nullptr,  // default value
    size_function__ZitPidStatus__vel_ki,  // size() function pointer
    get_const_function__ZitPidStatus__vel_ki,  // get_const(index) function pointer
    get_function__ZitPidStatus__vel_ki,  // get(index) function pointer
    fetch_function__ZitPidStatus__vel_ki,  // fetch(index, &value) function pointer
    assign_function__ZitPidStatus__vel_ki,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "vel_kd",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_FLOAT,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    false,  // is key
    true,  // is array
    4,  // array size
    false,  // is upper bound
    offsetof(zit6_interfaces::msg::ZitPidStatus, vel_kd),  // bytes offset in struct
    nullptr,  // default value
    size_function__ZitPidStatus__vel_kd,  // size() function pointer
    get_const_function__ZitPidStatus__vel_kd,  // get_const(index) function pointer
    get_function__ZitPidStatus__vel_kd,  // get(index) function pointer
    fetch_function__ZitPidStatus__vel_kd,  // fetch(index, &value) function pointer
    assign_function__ZitPidStatus__vel_kd,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "vel_i_limit",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_FLOAT,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    false,  // is key
    true,  // is array
    4,  // array size
    false,  // is upper bound
    offsetof(zit6_interfaces::msg::ZitPidStatus, vel_i_limit),  // bytes offset in struct
    nullptr,  // default value
    size_function__ZitPidStatus__vel_i_limit,  // size() function pointer
    get_const_function__ZitPidStatus__vel_i_limit,  // get_const(index) function pointer
    get_function__ZitPidStatus__vel_i_limit,  // get(index) function pointer
    fetch_function__ZitPidStatus__vel_i_limit,  // fetch(index, &value) function pointer
    assign_function__ZitPidStatus__vel_i_limit,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "vel_out_limit",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_FLOAT,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    false,  // is key
    true,  // is array
    4,  // array size
    false,  // is upper bound
    offsetof(zit6_interfaces::msg::ZitPidStatus, vel_out_limit),  // bytes offset in struct
    nullptr,  // default value
    size_function__ZitPidStatus__vel_out_limit,  // size() function pointer
    get_const_function__ZitPidStatus__vel_out_limit,  // get_const(index) function pointer
    get_function__ZitPidStatus__vel_out_limit,  // get(index) function pointer
    fetch_function__ZitPidStatus__vel_out_limit,  // fetch(index, &value) function pointer
    assign_function__ZitPidStatus__vel_out_limit,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  }
};

static const ::rosidl_typesupport_introspection_cpp::MessageMembers ZitPidStatus_message_members = {
  "zit6_interfaces::msg",  // message namespace
  "ZitPidStatus",  // message name
  9,  // number of fields
  sizeof(zit6_interfaces::msg::ZitPidStatus),
  false,  // has_any_key_member_
  ZitPidStatus_message_member_array,  // message members
  ZitPidStatus_init_function,  // function to initialize message memory (memory has to be allocated)
  ZitPidStatus_fini_function  // function to terminate message instance (will not free memory)
};

static const rosidl_message_type_support_t ZitPidStatus_message_type_support_handle = {
  ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  &ZitPidStatus_message_members,
  get_message_typesupport_handle_function,
  &zit6_interfaces__msg__ZitPidStatus__get_type_hash,
  &zit6_interfaces__msg__ZitPidStatus__get_type_description,
  &zit6_interfaces__msg__ZitPidStatus__get_type_description_sources,
};

}  // namespace rosidl_typesupport_introspection_cpp

}  // namespace msg

}  // namespace zit6_interfaces


namespace rosidl_typesupport_introspection_cpp
{

template<>
ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
get_message_type_support_handle<zit6_interfaces::msg::ZitPidStatus>()
{
  return &::zit6_interfaces::msg::rosidl_typesupport_introspection_cpp::ZitPidStatus_message_type_support_handle;
}

}  // namespace rosidl_typesupport_introspection_cpp

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, zit6_interfaces, msg, ZitPidStatus)() {
  return &::zit6_interfaces::msg::rosidl_typesupport_introspection_cpp::ZitPidStatus_message_type_support_handle;
}

#ifdef __cplusplus
}
#endif
