// generated from rosidl_typesupport_fastrtps_c/resource/idl__rosidl_typesupport_fastrtps_c.h.em
// with input from uv_msgs:msg/Waypoint.idl
// generated code does not contain a copyright notice
#ifndef UV_MSGS__MSG__DETAIL__WAYPOINT__ROSIDL_TYPESUPPORT_FASTRTPS_C_H_
#define UV_MSGS__MSG__DETAIL__WAYPOINT__ROSIDL_TYPESUPPORT_FASTRTPS_C_H_


#include <stddef.h>
#include "rosidl_runtime_c/message_type_support_struct.h"
#include "rosidl_typesupport_interface/macros.h"
#include "uv_msgs/msg/rosidl_typesupport_fastrtps_c__visibility_control.h"
#include "uv_msgs/msg/detail/waypoint__struct.h"
#include "fastcdr/Cdr.h"

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_uv_msgs
bool cdr_serialize_uv_msgs__msg__Waypoint(
  const uv_msgs__msg__Waypoint * ros_message,
  eprosima::fastcdr::Cdr & cdr);

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_uv_msgs
bool cdr_deserialize_uv_msgs__msg__Waypoint(
  eprosima::fastcdr::Cdr &,
  uv_msgs__msg__Waypoint * ros_message);

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_uv_msgs
size_t get_serialized_size_uv_msgs__msg__Waypoint(
  const void * untyped_ros_message,
  size_t current_alignment);

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_uv_msgs
size_t max_serialized_size_uv_msgs__msg__Waypoint(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment);

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_uv_msgs
bool cdr_serialize_key_uv_msgs__msg__Waypoint(
  const uv_msgs__msg__Waypoint * ros_message,
  eprosima::fastcdr::Cdr & cdr);

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_uv_msgs
size_t get_serialized_size_key_uv_msgs__msg__Waypoint(
  const void * untyped_ros_message,
  size_t current_alignment);

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_uv_msgs
size_t max_serialized_size_key_uv_msgs__msg__Waypoint(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment);

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_uv_msgs
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, uv_msgs, msg, Waypoint)();

#ifdef __cplusplus
}
#endif

#endif  // UV_MSGS__MSG__DETAIL__WAYPOINT__ROSIDL_TYPESUPPORT_FASTRTPS_C_H_
