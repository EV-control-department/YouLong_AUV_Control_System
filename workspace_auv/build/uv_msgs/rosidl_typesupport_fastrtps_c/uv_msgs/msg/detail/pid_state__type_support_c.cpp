// generated from rosidl_typesupport_fastrtps_c/resource/idl__type_support_c.cpp.em
// with input from uv_msgs:msg/PidState.idl
// generated code does not contain a copyright notice
#include "uv_msgs/msg/detail/pid_state__rosidl_typesupport_fastrtps_c.h"


#include <cassert>
#include <cstddef>
#include <limits>
#include <string>
#include "rosidl_typesupport_fastrtps_c/identifier.h"
#include "rosidl_typesupport_fastrtps_c/serialization_helpers.hpp"
#include "rosidl_typesupport_fastrtps_c/wstring_conversion.hpp"
#include "rosidl_typesupport_fastrtps_cpp/message_type_support.h"
#include "uv_msgs/msg/rosidl_typesupport_fastrtps_c__visibility_control.h"
#include "uv_msgs/msg/detail/pid_state__struct.h"
#include "uv_msgs/msg/detail/pid_state__functions.h"
#include "fastcdr/Cdr.h"

#ifndef _WIN32
# pragma GCC diagnostic push
# pragma GCC diagnostic ignored "-Wunused-parameter"
# ifdef __clang__
#  pragma clang diagnostic ignored "-Wdeprecated-register"
#  pragma clang diagnostic ignored "-Wreturn-type-c-linkage"
# endif
#endif
#ifndef _WIN32
# pragma GCC diagnostic pop
#endif

// includes and forward declarations of message dependencies and their conversion functions

#if defined(__cplusplus)
extern "C"
{
#endif

#include "uv_msgs/msg/detail/pid_gains__functions.h"  // pos_z, speed_mag, vel_x, vel_y, yaw, yaw_rate

// forward declare type support functions

bool cdr_serialize_uv_msgs__msg__PidGains(
  const uv_msgs__msg__PidGains * ros_message,
  eprosima::fastcdr::Cdr & cdr);

bool cdr_deserialize_uv_msgs__msg__PidGains(
  eprosima::fastcdr::Cdr & cdr,
  uv_msgs__msg__PidGains * ros_message);

size_t get_serialized_size_uv_msgs__msg__PidGains(
  const void * untyped_ros_message,
  size_t current_alignment);

size_t max_serialized_size_uv_msgs__msg__PidGains(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment);

bool cdr_serialize_key_uv_msgs__msg__PidGains(
  const uv_msgs__msg__PidGains * ros_message,
  eprosima::fastcdr::Cdr & cdr);

size_t get_serialized_size_key_uv_msgs__msg__PidGains(
  const void * untyped_ros_message,
  size_t current_alignment);

size_t max_serialized_size_key_uv_msgs__msg__PidGains(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment);

const rosidl_message_type_support_t *
  ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, uv_msgs, msg, PidGains)();


using _PidState__ros_msg_type = uv_msgs__msg__PidState;


ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_uv_msgs
bool cdr_serialize_uv_msgs__msg__PidState(
  const uv_msgs__msg__PidState * ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  // Field name: speed_mag
  {
    cdr_serialize_uv_msgs__msg__PidGains(
      &ros_message->speed_mag, cdr);
  }

  // Field name: vel_x
  {
    cdr_serialize_uv_msgs__msg__PidGains(
      &ros_message->vel_x, cdr);
  }

  // Field name: vel_y
  {
    cdr_serialize_uv_msgs__msg__PidGains(
      &ros_message->vel_y, cdr);
  }

  // Field name: pos_z
  {
    cdr_serialize_uv_msgs__msg__PidGains(
      &ros_message->pos_z, cdr);
  }

  // Field name: yaw
  {
    cdr_serialize_uv_msgs__msg__PidGains(
      &ros_message->yaw, cdr);
  }

  // Field name: yaw_rate
  {
    cdr_serialize_uv_msgs__msg__PidGains(
      &ros_message->yaw_rate, cdr);
  }

  // Field name: error_length
  {
    cdr << ros_message->error_length;
  }

  // Field name: error_angle_deg
  {
    cdr << ros_message->error_angle_deg;
  }

  // Field name: speed_cmd
  {
    cdr << ros_message->speed_cmd;
  }

  // Field name: vx_body_cmd
  {
    cdr << ros_message->vx_body_cmd;
  }

  // Field name: vy_body_cmd
  {
    cdr << ros_message->vy_body_cmd;
  }

  return true;
}

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_uv_msgs
bool cdr_deserialize_uv_msgs__msg__PidState(
  eprosima::fastcdr::Cdr & cdr,
  uv_msgs__msg__PidState * ros_message)
{
  // Field name: speed_mag
  {
    cdr_deserialize_uv_msgs__msg__PidGains(cdr, &ros_message->speed_mag);
  }

  // Field name: vel_x
  {
    cdr_deserialize_uv_msgs__msg__PidGains(cdr, &ros_message->vel_x);
  }

  // Field name: vel_y
  {
    cdr_deserialize_uv_msgs__msg__PidGains(cdr, &ros_message->vel_y);
  }

  // Field name: pos_z
  {
    cdr_deserialize_uv_msgs__msg__PidGains(cdr, &ros_message->pos_z);
  }

  // Field name: yaw
  {
    cdr_deserialize_uv_msgs__msg__PidGains(cdr, &ros_message->yaw);
  }

  // Field name: yaw_rate
  {
    cdr_deserialize_uv_msgs__msg__PidGains(cdr, &ros_message->yaw_rate);
  }

  // Field name: error_length
  {
    cdr >> ros_message->error_length;
  }

  // Field name: error_angle_deg
  {
    cdr >> ros_message->error_angle_deg;
  }

  // Field name: speed_cmd
  {
    cdr >> ros_message->speed_cmd;
  }

  // Field name: vx_body_cmd
  {
    cdr >> ros_message->vx_body_cmd;
  }

  // Field name: vy_body_cmd
  {
    cdr >> ros_message->vy_body_cmd;
  }

  return true;
}  // NOLINT(readability/fn_size)


ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_uv_msgs
size_t get_serialized_size_uv_msgs__msg__PidState(
  const void * untyped_ros_message,
  size_t current_alignment)
{
  const _PidState__ros_msg_type * ros_message = static_cast<const _PidState__ros_msg_type *>(untyped_ros_message);
  (void)ros_message;
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;

  // Field name: speed_mag
  current_alignment += get_serialized_size_uv_msgs__msg__PidGains(
    &(ros_message->speed_mag), current_alignment);

  // Field name: vel_x
  current_alignment += get_serialized_size_uv_msgs__msg__PidGains(
    &(ros_message->vel_x), current_alignment);

  // Field name: vel_y
  current_alignment += get_serialized_size_uv_msgs__msg__PidGains(
    &(ros_message->vel_y), current_alignment);

  // Field name: pos_z
  current_alignment += get_serialized_size_uv_msgs__msg__PidGains(
    &(ros_message->pos_z), current_alignment);

  // Field name: yaw
  current_alignment += get_serialized_size_uv_msgs__msg__PidGains(
    &(ros_message->yaw), current_alignment);

  // Field name: yaw_rate
  current_alignment += get_serialized_size_uv_msgs__msg__PidGains(
    &(ros_message->yaw_rate), current_alignment);

  // Field name: error_length
  {
    size_t item_size = sizeof(ros_message->error_length);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  // Field name: error_angle_deg
  {
    size_t item_size = sizeof(ros_message->error_angle_deg);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  // Field name: speed_cmd
  {
    size_t item_size = sizeof(ros_message->speed_cmd);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  // Field name: vx_body_cmd
  {
    size_t item_size = sizeof(ros_message->vx_body_cmd);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  // Field name: vy_body_cmd
  {
    size_t item_size = sizeof(ros_message->vy_body_cmd);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  return current_alignment - initial_alignment;
}


ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_uv_msgs
size_t max_serialized_size_uv_msgs__msg__PidState(
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

  // Field name: speed_mag
  {
    size_t array_size = 1;
    last_member_size = 0;
    for (size_t index = 0; index < array_size; ++index) {
      bool inner_full_bounded;
      bool inner_is_plain;
      size_t inner_size;
      inner_size =
        max_serialized_size_uv_msgs__msg__PidGains(
        inner_full_bounded, inner_is_plain, current_alignment);
      last_member_size += inner_size;
      current_alignment += inner_size;
      full_bounded &= inner_full_bounded;
      is_plain &= inner_is_plain;
    }
  }

  // Field name: vel_x
  {
    size_t array_size = 1;
    last_member_size = 0;
    for (size_t index = 0; index < array_size; ++index) {
      bool inner_full_bounded;
      bool inner_is_plain;
      size_t inner_size;
      inner_size =
        max_serialized_size_uv_msgs__msg__PidGains(
        inner_full_bounded, inner_is_plain, current_alignment);
      last_member_size += inner_size;
      current_alignment += inner_size;
      full_bounded &= inner_full_bounded;
      is_plain &= inner_is_plain;
    }
  }

  // Field name: vel_y
  {
    size_t array_size = 1;
    last_member_size = 0;
    for (size_t index = 0; index < array_size; ++index) {
      bool inner_full_bounded;
      bool inner_is_plain;
      size_t inner_size;
      inner_size =
        max_serialized_size_uv_msgs__msg__PidGains(
        inner_full_bounded, inner_is_plain, current_alignment);
      last_member_size += inner_size;
      current_alignment += inner_size;
      full_bounded &= inner_full_bounded;
      is_plain &= inner_is_plain;
    }
  }

  // Field name: pos_z
  {
    size_t array_size = 1;
    last_member_size = 0;
    for (size_t index = 0; index < array_size; ++index) {
      bool inner_full_bounded;
      bool inner_is_plain;
      size_t inner_size;
      inner_size =
        max_serialized_size_uv_msgs__msg__PidGains(
        inner_full_bounded, inner_is_plain, current_alignment);
      last_member_size += inner_size;
      current_alignment += inner_size;
      full_bounded &= inner_full_bounded;
      is_plain &= inner_is_plain;
    }
  }

  // Field name: yaw
  {
    size_t array_size = 1;
    last_member_size = 0;
    for (size_t index = 0; index < array_size; ++index) {
      bool inner_full_bounded;
      bool inner_is_plain;
      size_t inner_size;
      inner_size =
        max_serialized_size_uv_msgs__msg__PidGains(
        inner_full_bounded, inner_is_plain, current_alignment);
      last_member_size += inner_size;
      current_alignment += inner_size;
      full_bounded &= inner_full_bounded;
      is_plain &= inner_is_plain;
    }
  }

  // Field name: yaw_rate
  {
    size_t array_size = 1;
    last_member_size = 0;
    for (size_t index = 0; index < array_size; ++index) {
      bool inner_full_bounded;
      bool inner_is_plain;
      size_t inner_size;
      inner_size =
        max_serialized_size_uv_msgs__msg__PidGains(
        inner_full_bounded, inner_is_plain, current_alignment);
      last_member_size += inner_size;
      current_alignment += inner_size;
      full_bounded &= inner_full_bounded;
      is_plain &= inner_is_plain;
    }
  }

  // Field name: error_length
  {
    size_t array_size = 1;
    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  // Field name: error_angle_deg
  {
    size_t array_size = 1;
    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  // Field name: speed_cmd
  {
    size_t array_size = 1;
    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  // Field name: vx_body_cmd
  {
    size_t array_size = 1;
    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  // Field name: vy_body_cmd
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
    using DataType = uv_msgs__msg__PidState;
    is_plain =
      (
      offsetof(DataType, vy_body_cmd) +
      last_member_size
      ) == ret_val;
  }
  return ret_val;
}

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_uv_msgs
bool cdr_serialize_key_uv_msgs__msg__PidState(
  const uv_msgs__msg__PidState * ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  // Field name: speed_mag
  {
    cdr_serialize_key_uv_msgs__msg__PidGains(
      &ros_message->speed_mag, cdr);
  }

  // Field name: vel_x
  {
    cdr_serialize_key_uv_msgs__msg__PidGains(
      &ros_message->vel_x, cdr);
  }

  // Field name: vel_y
  {
    cdr_serialize_key_uv_msgs__msg__PidGains(
      &ros_message->vel_y, cdr);
  }

  // Field name: pos_z
  {
    cdr_serialize_key_uv_msgs__msg__PidGains(
      &ros_message->pos_z, cdr);
  }

  // Field name: yaw
  {
    cdr_serialize_key_uv_msgs__msg__PidGains(
      &ros_message->yaw, cdr);
  }

  // Field name: yaw_rate
  {
    cdr_serialize_key_uv_msgs__msg__PidGains(
      &ros_message->yaw_rate, cdr);
  }

  // Field name: error_length
  {
    cdr << ros_message->error_length;
  }

  // Field name: error_angle_deg
  {
    cdr << ros_message->error_angle_deg;
  }

  // Field name: speed_cmd
  {
    cdr << ros_message->speed_cmd;
  }

  // Field name: vx_body_cmd
  {
    cdr << ros_message->vx_body_cmd;
  }

  // Field name: vy_body_cmd
  {
    cdr << ros_message->vy_body_cmd;
  }

  return true;
}

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_uv_msgs
size_t get_serialized_size_key_uv_msgs__msg__PidState(
  const void * untyped_ros_message,
  size_t current_alignment)
{
  const _PidState__ros_msg_type * ros_message = static_cast<const _PidState__ros_msg_type *>(untyped_ros_message);
  (void)ros_message;

  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;

  // Field name: speed_mag
  current_alignment += get_serialized_size_key_uv_msgs__msg__PidGains(
    &(ros_message->speed_mag), current_alignment);

  // Field name: vel_x
  current_alignment += get_serialized_size_key_uv_msgs__msg__PidGains(
    &(ros_message->vel_x), current_alignment);

  // Field name: vel_y
  current_alignment += get_serialized_size_key_uv_msgs__msg__PidGains(
    &(ros_message->vel_y), current_alignment);

  // Field name: pos_z
  current_alignment += get_serialized_size_key_uv_msgs__msg__PidGains(
    &(ros_message->pos_z), current_alignment);

  // Field name: yaw
  current_alignment += get_serialized_size_key_uv_msgs__msg__PidGains(
    &(ros_message->yaw), current_alignment);

  // Field name: yaw_rate
  current_alignment += get_serialized_size_key_uv_msgs__msg__PidGains(
    &(ros_message->yaw_rate), current_alignment);

  // Field name: error_length
  {
    size_t item_size = sizeof(ros_message->error_length);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  // Field name: error_angle_deg
  {
    size_t item_size = sizeof(ros_message->error_angle_deg);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  // Field name: speed_cmd
  {
    size_t item_size = sizeof(ros_message->speed_cmd);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  // Field name: vx_body_cmd
  {
    size_t item_size = sizeof(ros_message->vx_body_cmd);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  // Field name: vy_body_cmd
  {
    size_t item_size = sizeof(ros_message->vy_body_cmd);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  return current_alignment - initial_alignment;
}

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_uv_msgs
size_t max_serialized_size_key_uv_msgs__msg__PidState(
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
  // Field name: speed_mag
  {
    size_t array_size = 1;
    last_member_size = 0;
    for (size_t index = 0; index < array_size; ++index) {
      bool inner_full_bounded;
      bool inner_is_plain;
      size_t inner_size;
      inner_size =
        max_serialized_size_key_uv_msgs__msg__PidGains(
        inner_full_bounded, inner_is_plain, current_alignment);
      last_member_size += inner_size;
      current_alignment += inner_size;
      full_bounded &= inner_full_bounded;
      is_plain &= inner_is_plain;
    }
  }

  // Field name: vel_x
  {
    size_t array_size = 1;
    last_member_size = 0;
    for (size_t index = 0; index < array_size; ++index) {
      bool inner_full_bounded;
      bool inner_is_plain;
      size_t inner_size;
      inner_size =
        max_serialized_size_key_uv_msgs__msg__PidGains(
        inner_full_bounded, inner_is_plain, current_alignment);
      last_member_size += inner_size;
      current_alignment += inner_size;
      full_bounded &= inner_full_bounded;
      is_plain &= inner_is_plain;
    }
  }

  // Field name: vel_y
  {
    size_t array_size = 1;
    last_member_size = 0;
    for (size_t index = 0; index < array_size; ++index) {
      bool inner_full_bounded;
      bool inner_is_plain;
      size_t inner_size;
      inner_size =
        max_serialized_size_key_uv_msgs__msg__PidGains(
        inner_full_bounded, inner_is_plain, current_alignment);
      last_member_size += inner_size;
      current_alignment += inner_size;
      full_bounded &= inner_full_bounded;
      is_plain &= inner_is_plain;
    }
  }

  // Field name: pos_z
  {
    size_t array_size = 1;
    last_member_size = 0;
    for (size_t index = 0; index < array_size; ++index) {
      bool inner_full_bounded;
      bool inner_is_plain;
      size_t inner_size;
      inner_size =
        max_serialized_size_key_uv_msgs__msg__PidGains(
        inner_full_bounded, inner_is_plain, current_alignment);
      last_member_size += inner_size;
      current_alignment += inner_size;
      full_bounded &= inner_full_bounded;
      is_plain &= inner_is_plain;
    }
  }

  // Field name: yaw
  {
    size_t array_size = 1;
    last_member_size = 0;
    for (size_t index = 0; index < array_size; ++index) {
      bool inner_full_bounded;
      bool inner_is_plain;
      size_t inner_size;
      inner_size =
        max_serialized_size_key_uv_msgs__msg__PidGains(
        inner_full_bounded, inner_is_plain, current_alignment);
      last_member_size += inner_size;
      current_alignment += inner_size;
      full_bounded &= inner_full_bounded;
      is_plain &= inner_is_plain;
    }
  }

  // Field name: yaw_rate
  {
    size_t array_size = 1;
    last_member_size = 0;
    for (size_t index = 0; index < array_size; ++index) {
      bool inner_full_bounded;
      bool inner_is_plain;
      size_t inner_size;
      inner_size =
        max_serialized_size_key_uv_msgs__msg__PidGains(
        inner_full_bounded, inner_is_plain, current_alignment);
      last_member_size += inner_size;
      current_alignment += inner_size;
      full_bounded &= inner_full_bounded;
      is_plain &= inner_is_plain;
    }
  }

  // Field name: error_length
  {
    size_t array_size = 1;
    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  // Field name: error_angle_deg
  {
    size_t array_size = 1;
    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  // Field name: speed_cmd
  {
    size_t array_size = 1;
    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  // Field name: vx_body_cmd
  {
    size_t array_size = 1;
    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  // Field name: vy_body_cmd
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
    using DataType = uv_msgs__msg__PidState;
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
  if (!untyped_ros_message) {
    fprintf(stderr, "ros message handle is null\n");
    return false;
  }
  const uv_msgs__msg__PidState * ros_message = static_cast<const uv_msgs__msg__PidState *>(untyped_ros_message);
  (void)ros_message;
  return cdr_serialize_uv_msgs__msg__PidState(ros_message, cdr);
}

static bool _PidState__cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  void * untyped_ros_message)
{
  if (!untyped_ros_message) {
    fprintf(stderr, "ros message handle is null\n");
    return false;
  }
  uv_msgs__msg__PidState * ros_message = static_cast<uv_msgs__msg__PidState *>(untyped_ros_message);
  (void)ros_message;
  return cdr_deserialize_uv_msgs__msg__PidState(cdr, ros_message);
}

static uint32_t _PidState__get_serialized_size(const void * untyped_ros_message)
{
  return static_cast<uint32_t>(
    get_serialized_size_uv_msgs__msg__PidState(
      untyped_ros_message, 0));
}

static size_t _PidState__max_serialized_size(char & bounds_info)
{
  bool full_bounded;
  bool is_plain;
  size_t ret_val;

  ret_val = max_serialized_size_uv_msgs__msg__PidState(
    full_bounded, is_plain, 0);

  bounds_info =
    is_plain ? ROSIDL_TYPESUPPORT_FASTRTPS_PLAIN_TYPE :
    full_bounded ? ROSIDL_TYPESUPPORT_FASTRTPS_BOUNDED_TYPE : ROSIDL_TYPESUPPORT_FASTRTPS_UNBOUNDED_TYPE;
  return ret_val;
}


static message_type_support_callbacks_t __callbacks_PidState = {
  "uv_msgs::msg",
  "PidState",
  _PidState__cdr_serialize,
  _PidState__cdr_deserialize,
  _PidState__get_serialized_size,
  _PidState__max_serialized_size,
  nullptr
};

static rosidl_message_type_support_t _PidState__type_support = {
  rosidl_typesupport_fastrtps_c__identifier,
  &__callbacks_PidState,
  get_message_typesupport_handle_function,
  &uv_msgs__msg__PidState__get_type_hash,
  &uv_msgs__msg__PidState__get_type_description,
  &uv_msgs__msg__PidState__get_type_description_sources,
};

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, uv_msgs, msg, PidState)() {
  return &_PidState__type_support;
}

#if defined(__cplusplus)
}
#endif
