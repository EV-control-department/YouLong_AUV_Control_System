// generated from rosidl_typesupport_fastrtps_cpp/resource/idl__type_support.cpp.em
// with input from uv_msgs:msg/PidState.idl
// generated code does not contain a copyright notice
#include "uv_msgs/msg/detail/pid_state__rosidl_typesupport_fastrtps_cpp.hpp"
#include "uv_msgs/msg/detail/pid_state__functions.h"
#include "uv_msgs/msg/detail/pid_state__struct.hpp"

#include <cstddef>
#include <limits>
#include <stdexcept>
#include <string>
#include "rosidl_typesupport_cpp/message_type_support.hpp"
#include "rosidl_typesupport_fastrtps_cpp/identifier.hpp"
#include "rosidl_typesupport_fastrtps_cpp/message_type_support.h"
#include "rosidl_typesupport_fastrtps_cpp/message_type_support_decl.hpp"
#include "rosidl_typesupport_fastrtps_cpp/serialization_helpers.hpp"
#include "rosidl_typesupport_fastrtps_cpp/wstring_conversion.hpp"
#include "fastcdr/Cdr.h"


// forward declaration of message dependencies and their conversion functions
namespace uv_msgs
{
namespace msg
{
namespace typesupport_fastrtps_cpp
{
bool cdr_serialize(
  const uv_msgs::msg::PidGains &,
  eprosima::fastcdr::Cdr &);
bool cdr_deserialize(
  eprosima::fastcdr::Cdr &,
  uv_msgs::msg::PidGains &);
size_t get_serialized_size(
  const uv_msgs::msg::PidGains &,
  size_t current_alignment);
size_t
max_serialized_size_PidGains(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment);
bool cdr_serialize_key(
  const uv_msgs::msg::PidGains &,
  eprosima::fastcdr::Cdr &);
size_t get_serialized_size_key(
  const uv_msgs::msg::PidGains &,
  size_t current_alignment);
size_t
max_serialized_size_key_PidGains(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment);
}  // namespace typesupport_fastrtps_cpp
}  // namespace msg
}  // namespace uv_msgs

// functions for uv_msgs::msg::PidGains already declared above

// functions for uv_msgs::msg::PidGains already declared above

// functions for uv_msgs::msg::PidGains already declared above

// functions for uv_msgs::msg::PidGains already declared above

// functions for uv_msgs::msg::PidGains already declared above


namespace uv_msgs
{

namespace msg
{

namespace typesupport_fastrtps_cpp
{


bool
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_uv_msgs
cdr_serialize(
  const uv_msgs::msg::PidState & ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  // Member: speed_mag
  uv_msgs::msg::typesupport_fastrtps_cpp::cdr_serialize(
    ros_message.speed_mag,
    cdr);

  // Member: vel_x
  uv_msgs::msg::typesupport_fastrtps_cpp::cdr_serialize(
    ros_message.vel_x,
    cdr);

  // Member: vel_y
  uv_msgs::msg::typesupport_fastrtps_cpp::cdr_serialize(
    ros_message.vel_y,
    cdr);

  // Member: pos_z
  uv_msgs::msg::typesupport_fastrtps_cpp::cdr_serialize(
    ros_message.pos_z,
    cdr);

  // Member: yaw
  uv_msgs::msg::typesupport_fastrtps_cpp::cdr_serialize(
    ros_message.yaw,
    cdr);

  // Member: yaw_rate
  uv_msgs::msg::typesupport_fastrtps_cpp::cdr_serialize(
    ros_message.yaw_rate,
    cdr);

  // Member: error_length
  cdr << ros_message.error_length;

  // Member: error_angle_deg
  cdr << ros_message.error_angle_deg;

  // Member: speed_cmd
  cdr << ros_message.speed_cmd;

  // Member: vx_body_cmd
  cdr << ros_message.vx_body_cmd;

  // Member: vy_body_cmd
  cdr << ros_message.vy_body_cmd;

  return true;
}

bool
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_uv_msgs
cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  uv_msgs::msg::PidState & ros_message)
{
  // Member: speed_mag
  uv_msgs::msg::typesupport_fastrtps_cpp::cdr_deserialize(
    cdr, ros_message.speed_mag);

  // Member: vel_x
  uv_msgs::msg::typesupport_fastrtps_cpp::cdr_deserialize(
    cdr, ros_message.vel_x);

  // Member: vel_y
  uv_msgs::msg::typesupport_fastrtps_cpp::cdr_deserialize(
    cdr, ros_message.vel_y);

  // Member: pos_z
  uv_msgs::msg::typesupport_fastrtps_cpp::cdr_deserialize(
    cdr, ros_message.pos_z);

  // Member: yaw
  uv_msgs::msg::typesupport_fastrtps_cpp::cdr_deserialize(
    cdr, ros_message.yaw);

  // Member: yaw_rate
  uv_msgs::msg::typesupport_fastrtps_cpp::cdr_deserialize(
    cdr, ros_message.yaw_rate);

  // Member: error_length
  cdr >> ros_message.error_length;

  // Member: error_angle_deg
  cdr >> ros_message.error_angle_deg;

  // Member: speed_cmd
  cdr >> ros_message.speed_cmd;

  // Member: vx_body_cmd
  cdr >> ros_message.vx_body_cmd;

  // Member: vy_body_cmd
  cdr >> ros_message.vy_body_cmd;

  return true;
}  // NOLINT(readability/fn_size)


size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_uv_msgs
get_serialized_size(
  const uv_msgs::msg::PidState & ros_message,
  size_t current_alignment)
{
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;

  // Member: speed_mag
  current_alignment +=
    uv_msgs::msg::typesupport_fastrtps_cpp::get_serialized_size(
    ros_message.speed_mag, current_alignment);

  // Member: vel_x
  current_alignment +=
    uv_msgs::msg::typesupport_fastrtps_cpp::get_serialized_size(
    ros_message.vel_x, current_alignment);

  // Member: vel_y
  current_alignment +=
    uv_msgs::msg::typesupport_fastrtps_cpp::get_serialized_size(
    ros_message.vel_y, current_alignment);

  // Member: pos_z
  current_alignment +=
    uv_msgs::msg::typesupport_fastrtps_cpp::get_serialized_size(
    ros_message.pos_z, current_alignment);

  // Member: yaw
  current_alignment +=
    uv_msgs::msg::typesupport_fastrtps_cpp::get_serialized_size(
    ros_message.yaw, current_alignment);

  // Member: yaw_rate
  current_alignment +=
    uv_msgs::msg::typesupport_fastrtps_cpp::get_serialized_size(
    ros_message.yaw_rate, current_alignment);

  // Member: error_length
  {
    size_t item_size = sizeof(ros_message.error_length);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  // Member: error_angle_deg
  {
    size_t item_size = sizeof(ros_message.error_angle_deg);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  // Member: speed_cmd
  {
    size_t item_size = sizeof(ros_message.speed_cmd);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  // Member: vx_body_cmd
  {
    size_t item_size = sizeof(ros_message.vx_body_cmd);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  // Member: vy_body_cmd
  {
    size_t item_size = sizeof(ros_message.vy_body_cmd);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  return current_alignment - initial_alignment;
}


size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_uv_msgs
max_serialized_size_PidState(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment)
{
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  size_t last_member_size = 0;
  (void)last_member_size;
  (void)padding;
  (void)wchar_size;

  full_bounded = true;
  is_plain = true;

  // Member: speed_mag
  {
    size_t array_size = 1;
    last_member_size = 0;
    for (size_t index = 0; index < array_size; ++index) {
      bool inner_full_bounded;
      bool inner_is_plain;
      size_t inner_size =
        uv_msgs::msg::typesupport_fastrtps_cpp::max_serialized_size_PidGains(
        inner_full_bounded, inner_is_plain, current_alignment);
      last_member_size += inner_size;
      current_alignment += inner_size;
      full_bounded &= inner_full_bounded;
      is_plain &= inner_is_plain;
    }
  }
  // Member: vel_x
  {
    size_t array_size = 1;
    last_member_size = 0;
    for (size_t index = 0; index < array_size; ++index) {
      bool inner_full_bounded;
      bool inner_is_plain;
      size_t inner_size =
        uv_msgs::msg::typesupport_fastrtps_cpp::max_serialized_size_PidGains(
        inner_full_bounded, inner_is_plain, current_alignment);
      last_member_size += inner_size;
      current_alignment += inner_size;
      full_bounded &= inner_full_bounded;
      is_plain &= inner_is_plain;
    }
  }
  // Member: vel_y
  {
    size_t array_size = 1;
    last_member_size = 0;
    for (size_t index = 0; index < array_size; ++index) {
      bool inner_full_bounded;
      bool inner_is_plain;
      size_t inner_size =
        uv_msgs::msg::typesupport_fastrtps_cpp::max_serialized_size_PidGains(
        inner_full_bounded, inner_is_plain, current_alignment);
      last_member_size += inner_size;
      current_alignment += inner_size;
      full_bounded &= inner_full_bounded;
      is_plain &= inner_is_plain;
    }
  }
  // Member: pos_z
  {
    size_t array_size = 1;
    last_member_size = 0;
    for (size_t index = 0; index < array_size; ++index) {
      bool inner_full_bounded;
      bool inner_is_plain;
      size_t inner_size =
        uv_msgs::msg::typesupport_fastrtps_cpp::max_serialized_size_PidGains(
        inner_full_bounded, inner_is_plain, current_alignment);
      last_member_size += inner_size;
      current_alignment += inner_size;
      full_bounded &= inner_full_bounded;
      is_plain &= inner_is_plain;
    }
  }
  // Member: yaw
  {
    size_t array_size = 1;
    last_member_size = 0;
    for (size_t index = 0; index < array_size; ++index) {
      bool inner_full_bounded;
      bool inner_is_plain;
      size_t inner_size =
        uv_msgs::msg::typesupport_fastrtps_cpp::max_serialized_size_PidGains(
        inner_full_bounded, inner_is_plain, current_alignment);
      last_member_size += inner_size;
      current_alignment += inner_size;
      full_bounded &= inner_full_bounded;
      is_plain &= inner_is_plain;
    }
  }
  // Member: yaw_rate
  {
    size_t array_size = 1;
    last_member_size = 0;
    for (size_t index = 0; index < array_size; ++index) {
      bool inner_full_bounded;
      bool inner_is_plain;
      size_t inner_size =
        uv_msgs::msg::typesupport_fastrtps_cpp::max_serialized_size_PidGains(
        inner_full_bounded, inner_is_plain, current_alignment);
      last_member_size += inner_size;
      current_alignment += inner_size;
      full_bounded &= inner_full_bounded;
      is_plain &= inner_is_plain;
    }
  }
  // Member: error_length
  {
    size_t array_size = 1;
    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }
  // Member: error_angle_deg
  {
    size_t array_size = 1;
    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }
  // Member: speed_cmd
  {
    size_t array_size = 1;
    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }
  // Member: vx_body_cmd
  {
    size_t array_size = 1;
    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }
  // Member: vy_body_cmd
  {
    size_t array_size = 1;
    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  size_t ret_val = current_alignment - initial_alignment;
  if (is_plain) {
    // All members are plain, and type is not empty.
    // We still need to check that the in-memory alignment
    // is the same as the CDR mandated alignment.
    using DataType = uv_msgs::msg::PidState;
    is_plain =
      (
      offsetof(DataType, vy_body_cmd) +
      last_member_size
      ) == ret_val;
  }

  return ret_val;
}

bool
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_uv_msgs
cdr_serialize_key(
  const uv_msgs::msg::PidState & ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  // Member: speed_mag
  uv_msgs::msg::typesupport_fastrtps_cpp::cdr_serialize_key(
    ros_message.speed_mag,
    cdr);

  // Member: vel_x
  uv_msgs::msg::typesupport_fastrtps_cpp::cdr_serialize_key(
    ros_message.vel_x,
    cdr);

  // Member: vel_y
  uv_msgs::msg::typesupport_fastrtps_cpp::cdr_serialize_key(
    ros_message.vel_y,
    cdr);

  // Member: pos_z
  uv_msgs::msg::typesupport_fastrtps_cpp::cdr_serialize_key(
    ros_message.pos_z,
    cdr);

  // Member: yaw
  uv_msgs::msg::typesupport_fastrtps_cpp::cdr_serialize_key(
    ros_message.yaw,
    cdr);

  // Member: yaw_rate
  uv_msgs::msg::typesupport_fastrtps_cpp::cdr_serialize_key(
    ros_message.yaw_rate,
    cdr);

  // Member: error_length
  cdr << ros_message.error_length;

  // Member: error_angle_deg
  cdr << ros_message.error_angle_deg;

  // Member: speed_cmd
  cdr << ros_message.speed_cmd;

  // Member: vx_body_cmd
  cdr << ros_message.vx_body_cmd;

  // Member: vy_body_cmd
  cdr << ros_message.vy_body_cmd;

  return true;
}

size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_uv_msgs
get_serialized_size_key(
  const uv_msgs::msg::PidState & ros_message,
  size_t current_alignment)
{
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;

  // Member: speed_mag
  current_alignment +=
    uv_msgs::msg::typesupport_fastrtps_cpp::get_serialized_size_key(
    ros_message.speed_mag, current_alignment);

  // Member: vel_x
  current_alignment +=
    uv_msgs::msg::typesupport_fastrtps_cpp::get_serialized_size_key(
    ros_message.vel_x, current_alignment);

  // Member: vel_y
  current_alignment +=
    uv_msgs::msg::typesupport_fastrtps_cpp::get_serialized_size_key(
    ros_message.vel_y, current_alignment);

  // Member: pos_z
  current_alignment +=
    uv_msgs::msg::typesupport_fastrtps_cpp::get_serialized_size_key(
    ros_message.pos_z, current_alignment);

  // Member: yaw
  current_alignment +=
    uv_msgs::msg::typesupport_fastrtps_cpp::get_serialized_size_key(
    ros_message.yaw, current_alignment);

  // Member: yaw_rate
  current_alignment +=
    uv_msgs::msg::typesupport_fastrtps_cpp::get_serialized_size_key(
    ros_message.yaw_rate, current_alignment);

  // Member: error_length
  {
    size_t item_size = sizeof(ros_message.error_length);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  // Member: error_angle_deg
  {
    size_t item_size = sizeof(ros_message.error_angle_deg);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  // Member: speed_cmd
  {
    size_t item_size = sizeof(ros_message.speed_cmd);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  // Member: vx_body_cmd
  {
    size_t item_size = sizeof(ros_message.vx_body_cmd);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  // Member: vy_body_cmd
  {
    size_t item_size = sizeof(ros_message.vy_body_cmd);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  return current_alignment - initial_alignment;
}

size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_uv_msgs
max_serialized_size_key_PidState(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment)
{
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  size_t last_member_size = 0;
  (void)last_member_size;
  (void)padding;
  (void)wchar_size;

  full_bounded = true;
  is_plain = true;

  // Member: speed_mag
  {
    size_t array_size = 1;
    last_member_size = 0;
    for (size_t index = 0; index < array_size; ++index) {
      bool inner_full_bounded;
      bool inner_is_plain;
      size_t inner_size =
        uv_msgs::msg::typesupport_fastrtps_cpp::max_serialized_size_key_PidGains(
        inner_full_bounded, inner_is_plain, current_alignment);
      last_member_size += inner_size;
      current_alignment += inner_size;
      full_bounded &= inner_full_bounded;
      is_plain &= inner_is_plain;
    }
  }

  // Member: vel_x
  {
    size_t array_size = 1;
    last_member_size = 0;
    for (size_t index = 0; index < array_size; ++index) {
      bool inner_full_bounded;
      bool inner_is_plain;
      size_t inner_size =
        uv_msgs::msg::typesupport_fastrtps_cpp::max_serialized_size_key_PidGains(
        inner_full_bounded, inner_is_plain, current_alignment);
      last_member_size += inner_size;
      current_alignment += inner_size;
      full_bounded &= inner_full_bounded;
      is_plain &= inner_is_plain;
    }
  }

  // Member: vel_y
  {
    size_t array_size = 1;
    last_member_size = 0;
    for (size_t index = 0; index < array_size; ++index) {
      bool inner_full_bounded;
      bool inner_is_plain;
      size_t inner_size =
        uv_msgs::msg::typesupport_fastrtps_cpp::max_serialized_size_key_PidGains(
        inner_full_bounded, inner_is_plain, current_alignment);
      last_member_size += inner_size;
      current_alignment += inner_size;
      full_bounded &= inner_full_bounded;
      is_plain &= inner_is_plain;
    }
  }

  // Member: pos_z
  {
    size_t array_size = 1;
    last_member_size = 0;
    for (size_t index = 0; index < array_size; ++index) {
      bool inner_full_bounded;
      bool inner_is_plain;
      size_t inner_size =
        uv_msgs::msg::typesupport_fastrtps_cpp::max_serialized_size_key_PidGains(
        inner_full_bounded, inner_is_plain, current_alignment);
      last_member_size += inner_size;
      current_alignment += inner_size;
      full_bounded &= inner_full_bounded;
      is_plain &= inner_is_plain;
    }
  }

  // Member: yaw
  {
    size_t array_size = 1;
    last_member_size = 0;
    for (size_t index = 0; index < array_size; ++index) {
      bool inner_full_bounded;
      bool inner_is_plain;
      size_t inner_size =
        uv_msgs::msg::typesupport_fastrtps_cpp::max_serialized_size_key_PidGains(
        inner_full_bounded, inner_is_plain, current_alignment);
      last_member_size += inner_size;
      current_alignment += inner_size;
      full_bounded &= inner_full_bounded;
      is_plain &= inner_is_plain;
    }
  }

  // Member: yaw_rate
  {
    size_t array_size = 1;
    last_member_size = 0;
    for (size_t index = 0; index < array_size; ++index) {
      bool inner_full_bounded;
      bool inner_is_plain;
      size_t inner_size =
        uv_msgs::msg::typesupport_fastrtps_cpp::max_serialized_size_key_PidGains(
        inner_full_bounded, inner_is_plain, current_alignment);
      last_member_size += inner_size;
      current_alignment += inner_size;
      full_bounded &= inner_full_bounded;
      is_plain &= inner_is_plain;
    }
  }

  // Member: error_length
  {
    size_t array_size = 1;
    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  // Member: error_angle_deg
  {
    size_t array_size = 1;
    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  // Member: speed_cmd
  {
    size_t array_size = 1;
    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  // Member: vx_body_cmd
  {
    size_t array_size = 1;
    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  // Member: vy_body_cmd
  {
    size_t array_size = 1;
    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  size_t ret_val = current_alignment - initial_alignment;
  if (is_plain) {
    // All members are plain, and type is not empty.
    // We still need to check that the in-memory alignment
    // is the same as the CDR mandated alignment.
    using DataType = uv_msgs::msg::PidState;
    is_plain =
      (
      offsetof(DataType, vy_body_cmd) +
      last_member_size
      ) == ret_val;
  }

  return ret_val;
}


static bool _PidState__cdr_serialize(
  const void * untyped_ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  auto typed_message =
    static_cast<const uv_msgs::msg::PidState *>(
    untyped_ros_message);
  return cdr_serialize(*typed_message, cdr);
}

static bool _PidState__cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  void * untyped_ros_message)
{
  auto typed_message =
    static_cast<uv_msgs::msg::PidState *>(
    untyped_ros_message);
  return cdr_deserialize(cdr, *typed_message);
}

static uint32_t _PidState__get_serialized_size(
  const void * untyped_ros_message)
{
  auto typed_message =
    static_cast<const uv_msgs::msg::PidState *>(
    untyped_ros_message);
  return static_cast<uint32_t>(get_serialized_size(*typed_message, 0));
}

static size_t _PidState__max_serialized_size(char & bounds_info)
{
  bool full_bounded;
  bool is_plain;
  size_t ret_val;

  ret_val = max_serialized_size_PidState(full_bounded, is_plain, 0);

  bounds_info =
    is_plain ? ROSIDL_TYPESUPPORT_FASTRTPS_PLAIN_TYPE :
    full_bounded ? ROSIDL_TYPESUPPORT_FASTRTPS_BOUNDED_TYPE : ROSIDL_TYPESUPPORT_FASTRTPS_UNBOUNDED_TYPE;
  return ret_val;
}

static message_type_support_callbacks_t _PidState__callbacks = {
  "uv_msgs::msg",
  "PidState",
  _PidState__cdr_serialize,
  _PidState__cdr_deserialize,
  _PidState__get_serialized_size,
  _PidState__max_serialized_size,
  nullptr
};

static rosidl_message_type_support_t _PidState__handle = {
  rosidl_typesupport_fastrtps_cpp::typesupport_identifier,
  &_PidState__callbacks,
  get_message_typesupport_handle_function,
  &uv_msgs__msg__PidState__get_type_hash,
  &uv_msgs__msg__PidState__get_type_description,
  &uv_msgs__msg__PidState__get_type_description_sources,
};

}  // namespace typesupport_fastrtps_cpp

}  // namespace msg

}  // namespace uv_msgs

namespace rosidl_typesupport_fastrtps_cpp
{

template<>
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_EXPORT_uv_msgs
const rosidl_message_type_support_t *
get_message_type_support_handle<uv_msgs::msg::PidState>()
{
  return &uv_msgs::msg::typesupport_fastrtps_cpp::_PidState__handle;
}

}  // namespace rosidl_typesupport_fastrtps_cpp

#ifdef __cplusplus
extern "C"
{
#endif

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_cpp, uv_msgs, msg, PidState)() {
  return &uv_msgs::msg::typesupport_fastrtps_cpp::_PidState__handle;
}

#ifdef __cplusplus
}
#endif
