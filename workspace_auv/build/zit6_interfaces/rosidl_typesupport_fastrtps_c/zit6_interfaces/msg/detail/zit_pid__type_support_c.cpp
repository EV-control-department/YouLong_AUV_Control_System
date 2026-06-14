// generated from rosidl_typesupport_fastrtps_c/resource/idl__type_support_c.cpp.em
// with input from zit6_interfaces:msg/ZitPid.idl
// generated code does not contain a copyright notice
#include "zit6_interfaces/msg/detail/zit_pid__rosidl_typesupport_fastrtps_c.h"


#include <cassert>
#include <cstddef>
#include <limits>
#include <string>
#include "rosidl_typesupport_fastrtps_c/identifier.h"
#include "rosidl_typesupport_fastrtps_c/serialization_helpers.hpp"
#include "rosidl_typesupport_fastrtps_c/wstring_conversion.hpp"
#include "rosidl_typesupport_fastrtps_cpp/message_type_support.h"
#include "zit6_interfaces/msg/rosidl_typesupport_fastrtps_c__visibility_control.h"
#include "zit6_interfaces/msg/detail/zit_pid__struct.h"
#include "zit6_interfaces/msg/detail/zit_pid__functions.h"
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


// forward declare type support functions


using _ZitPid__ros_msg_type = zit6_interfaces__msg__ZitPid;


ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_zit6_interfaces
bool cdr_serialize_zit6_interfaces__msg__ZitPid(
  const zit6_interfaces__msg__ZitPid * ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  // Field name: axis
  {
    cdr << ros_message->axis;
  }

  // Field name: is_pos_ring
  {
    cdr << (ros_message->is_pos_ring ? true : false);
  }

  // Field name: kp
  {
    cdr << ros_message->kp;
  }

  // Field name: ki
  {
    cdr << ros_message->ki;
  }

  // Field name: kd
  {
    cdr << ros_message->kd;
  }

  // Field name: i_limit
  {
    cdr << ros_message->i_limit;
  }

  // Field name: out_limit
  {
    cdr << ros_message->out_limit;
  }

  // Field name: max_v
  {
    cdr << ros_message->max_v;
  }

  // Field name: max_a
  {
    cdr << ros_message->max_a;
  }

  return true;
}

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_zit6_interfaces
bool cdr_deserialize_zit6_interfaces__msg__ZitPid(
  eprosima::fastcdr::Cdr & cdr,
  zit6_interfaces__msg__ZitPid * ros_message)
{
  // Field name: axis
  {
    cdr >> ros_message->axis;
  }

  // Field name: is_pos_ring
  {
    uint8_t tmp;
    cdr >> tmp;
    ros_message->is_pos_ring = tmp ? true : false;
  }

  // Field name: kp
  {
    cdr >> ros_message->kp;
  }

  // Field name: ki
  {
    cdr >> ros_message->ki;
  }

  // Field name: kd
  {
    cdr >> ros_message->kd;
  }

  // Field name: i_limit
  {
    cdr >> ros_message->i_limit;
  }

  // Field name: out_limit
  {
    cdr >> ros_message->out_limit;
  }

  // Field name: max_v
  {
    cdr >> ros_message->max_v;
  }

  // Field name: max_a
  {
    cdr >> ros_message->max_a;
  }

  return true;
}  // NOLINT(readability/fn_size)


ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_zit6_interfaces
size_t get_serialized_size_zit6_interfaces__msg__ZitPid(
  const void * untyped_ros_message,
  size_t current_alignment)
{
  const _ZitPid__ros_msg_type * ros_message = static_cast<const _ZitPid__ros_msg_type *>(untyped_ros_message);
  (void)ros_message;
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;

  // Field name: axis
  {
    size_t item_size = sizeof(ros_message->axis);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  // Field name: is_pos_ring
  {
    size_t item_size = sizeof(ros_message->is_pos_ring);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  // Field name: kp
  {
    size_t item_size = sizeof(ros_message->kp);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  // Field name: ki
  {
    size_t item_size = sizeof(ros_message->ki);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  // Field name: kd
  {
    size_t item_size = sizeof(ros_message->kd);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  // Field name: i_limit
  {
    size_t item_size = sizeof(ros_message->i_limit);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  // Field name: out_limit
  {
    size_t item_size = sizeof(ros_message->out_limit);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  // Field name: max_v
  {
    size_t item_size = sizeof(ros_message->max_v);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  // Field name: max_a
  {
    size_t item_size = sizeof(ros_message->max_a);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  return current_alignment - initial_alignment;
}


ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_zit6_interfaces
size_t max_serialized_size_zit6_interfaces__msg__ZitPid(
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

  // Field name: axis
  {
    size_t array_size = 1;
    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }

  // Field name: is_pos_ring
  {
    size_t array_size = 1;
    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }

  // Field name: kp
  {
    size_t array_size = 1;
    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  // Field name: ki
  {
    size_t array_size = 1;
    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  // Field name: kd
  {
    size_t array_size = 1;
    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  // Field name: i_limit
  {
    size_t array_size = 1;
    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  // Field name: out_limit
  {
    size_t array_size = 1;
    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  // Field name: max_v
  {
    size_t array_size = 1;
    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  // Field name: max_a
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
    using DataType = zit6_interfaces__msg__ZitPid;
    is_plain =
      (
      offsetof(DataType, max_a) +
      last_member_size
      ) == ret_val;
  }
  return ret_val;
}

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_zit6_interfaces
bool cdr_serialize_key_zit6_interfaces__msg__ZitPid(
  const zit6_interfaces__msg__ZitPid * ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  // Field name: axis
  {
    cdr << ros_message->axis;
  }

  // Field name: is_pos_ring
  {
    cdr << (ros_message->is_pos_ring ? true : false);
  }

  // Field name: kp
  {
    cdr << ros_message->kp;
  }

  // Field name: ki
  {
    cdr << ros_message->ki;
  }

  // Field name: kd
  {
    cdr << ros_message->kd;
  }

  // Field name: i_limit
  {
    cdr << ros_message->i_limit;
  }

  // Field name: out_limit
  {
    cdr << ros_message->out_limit;
  }

  // Field name: max_v
  {
    cdr << ros_message->max_v;
  }

  // Field name: max_a
  {
    cdr << ros_message->max_a;
  }

  return true;
}

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_zit6_interfaces
size_t get_serialized_size_key_zit6_interfaces__msg__ZitPid(
  const void * untyped_ros_message,
  size_t current_alignment)
{
  const _ZitPid__ros_msg_type * ros_message = static_cast<const _ZitPid__ros_msg_type *>(untyped_ros_message);
  (void)ros_message;

  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;

  // Field name: axis
  {
    size_t item_size = sizeof(ros_message->axis);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  // Field name: is_pos_ring
  {
    size_t item_size = sizeof(ros_message->is_pos_ring);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  // Field name: kp
  {
    size_t item_size = sizeof(ros_message->kp);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  // Field name: ki
  {
    size_t item_size = sizeof(ros_message->ki);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  // Field name: kd
  {
    size_t item_size = sizeof(ros_message->kd);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  // Field name: i_limit
  {
    size_t item_size = sizeof(ros_message->i_limit);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  // Field name: out_limit
  {
    size_t item_size = sizeof(ros_message->out_limit);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  // Field name: max_v
  {
    size_t item_size = sizeof(ros_message->max_v);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  // Field name: max_a
  {
    size_t item_size = sizeof(ros_message->max_a);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  return current_alignment - initial_alignment;
}

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_zit6_interfaces
size_t max_serialized_size_key_zit6_interfaces__msg__ZitPid(
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
  // Field name: axis
  {
    size_t array_size = 1;
    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }

  // Field name: is_pos_ring
  {
    size_t array_size = 1;
    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }

  // Field name: kp
  {
    size_t array_size = 1;
    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  // Field name: ki
  {
    size_t array_size = 1;
    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  // Field name: kd
  {
    size_t array_size = 1;
    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  // Field name: i_limit
  {
    size_t array_size = 1;
    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  // Field name: out_limit
  {
    size_t array_size = 1;
    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  // Field name: max_v
  {
    size_t array_size = 1;
    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  // Field name: max_a
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
    using DataType = zit6_interfaces__msg__ZitPid;
    is_plain =
      (
      offsetof(DataType, max_a) +
      last_member_size
      ) == ret_val;
  }
  return ret_val;
}


static bool _ZitPid__cdr_serialize(
  const void * untyped_ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  if (!untyped_ros_message) {
    fprintf(stderr, "ros message handle is null\n");
    return false;
  }
  const zit6_interfaces__msg__ZitPid * ros_message = static_cast<const zit6_interfaces__msg__ZitPid *>(untyped_ros_message);
  (void)ros_message;
  return cdr_serialize_zit6_interfaces__msg__ZitPid(ros_message, cdr);
}

static bool _ZitPid__cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  void * untyped_ros_message)
{
  if (!untyped_ros_message) {
    fprintf(stderr, "ros message handle is null\n");
    return false;
  }
  zit6_interfaces__msg__ZitPid * ros_message = static_cast<zit6_interfaces__msg__ZitPid *>(untyped_ros_message);
  (void)ros_message;
  return cdr_deserialize_zit6_interfaces__msg__ZitPid(cdr, ros_message);
}

static uint32_t _ZitPid__get_serialized_size(const void * untyped_ros_message)
{
  return static_cast<uint32_t>(
    get_serialized_size_zit6_interfaces__msg__ZitPid(
      untyped_ros_message, 0));
}

static size_t _ZitPid__max_serialized_size(char & bounds_info)
{
  bool full_bounded;
  bool is_plain;
  size_t ret_val;

  ret_val = max_serialized_size_zit6_interfaces__msg__ZitPid(
    full_bounded, is_plain, 0);

  bounds_info =
    is_plain ? ROSIDL_TYPESUPPORT_FASTRTPS_PLAIN_TYPE :
    full_bounded ? ROSIDL_TYPESUPPORT_FASTRTPS_BOUNDED_TYPE : ROSIDL_TYPESUPPORT_FASTRTPS_UNBOUNDED_TYPE;
  return ret_val;
}


static message_type_support_callbacks_t __callbacks_ZitPid = {
  "zit6_interfaces::msg",
  "ZitPid",
  _ZitPid__cdr_serialize,
  _ZitPid__cdr_deserialize,
  _ZitPid__get_serialized_size,
  _ZitPid__max_serialized_size,
  nullptr
};

static rosidl_message_type_support_t _ZitPid__type_support = {
  rosidl_typesupport_fastrtps_c__identifier,
  &__callbacks_ZitPid,
  get_message_typesupport_handle_function,
  &zit6_interfaces__msg__ZitPid__get_type_hash,
  &zit6_interfaces__msg__ZitPid__get_type_description,
  &zit6_interfaces__msg__ZitPid__get_type_description_sources,
};

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, zit6_interfaces, msg, ZitPid)() {
  return &_ZitPid__type_support;
}

#if defined(__cplusplus)
}
#endif
