// generated from rosidl_typesupport_fastrtps_c/resource/idl__type_support_c.cpp.em
// with input from zit6_interfaces:msg/ZitPidStatus.idl
// generated code does not contain a copyright notice
#include "zit6_interfaces/msg/detail/zit_pid_status__rosidl_typesupport_fastrtps_c.h"


#include <cassert>
#include <cstddef>
#include <limits>
#include <string>
#include "rosidl_typesupport_fastrtps_c/identifier.h"
#include "rosidl_typesupport_fastrtps_c/serialization_helpers.hpp"
#include "rosidl_typesupport_fastrtps_c/wstring_conversion.hpp"
#include "rosidl_typesupport_fastrtps_cpp/message_type_support.h"
#include "zit6_interfaces/msg/rosidl_typesupport_fastrtps_c__visibility_control.h"
#include "zit6_interfaces/msg/detail/zit_pid_status__struct.h"
#include "zit6_interfaces/msg/detail/zit_pid_status__functions.h"
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


using _ZitPidStatus__ros_msg_type = zit6_interfaces__msg__ZitPidStatus;


ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_zit6_interfaces
bool cdr_serialize_zit6_interfaces__msg__ZitPidStatus(
  const zit6_interfaces__msg__ZitPidStatus * ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  // Field name: pos_kp
  {
    size_t size = 4;
    auto array_ptr = ros_message->pos_kp;
    cdr.serialize_array(array_ptr, size);
  }

  // Field name: pos_max_v
  {
    size_t size = 4;
    auto array_ptr = ros_message->pos_max_v;
    cdr.serialize_array(array_ptr, size);
  }

  // Field name: pos_max_a
  {
    size_t size = 4;
    auto array_ptr = ros_message->pos_max_a;
    cdr.serialize_array(array_ptr, size);
  }

  // Field name: pos_out_limit
  {
    size_t size = 4;
    auto array_ptr = ros_message->pos_out_limit;
    cdr.serialize_array(array_ptr, size);
  }

  // Field name: vel_kp
  {
    size_t size = 4;
    auto array_ptr = ros_message->vel_kp;
    cdr.serialize_array(array_ptr, size);
  }

  // Field name: vel_ki
  {
    size_t size = 4;
    auto array_ptr = ros_message->vel_ki;
    cdr.serialize_array(array_ptr, size);
  }

  // Field name: vel_kd
  {
    size_t size = 4;
    auto array_ptr = ros_message->vel_kd;
    cdr.serialize_array(array_ptr, size);
  }

  // Field name: vel_i_limit
  {
    size_t size = 4;
    auto array_ptr = ros_message->vel_i_limit;
    cdr.serialize_array(array_ptr, size);
  }

  // Field name: vel_out_limit
  {
    size_t size = 4;
    auto array_ptr = ros_message->vel_out_limit;
    cdr.serialize_array(array_ptr, size);
  }

  return true;
}

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_zit6_interfaces
bool cdr_deserialize_zit6_interfaces__msg__ZitPidStatus(
  eprosima::fastcdr::Cdr & cdr,
  zit6_interfaces__msg__ZitPidStatus * ros_message)
{
  // Field name: pos_kp
  {
    size_t size = 4;
    auto array_ptr = ros_message->pos_kp;
    cdr.deserialize_array(array_ptr, size);
  }

  // Field name: pos_max_v
  {
    size_t size = 4;
    auto array_ptr = ros_message->pos_max_v;
    cdr.deserialize_array(array_ptr, size);
  }

  // Field name: pos_max_a
  {
    size_t size = 4;
    auto array_ptr = ros_message->pos_max_a;
    cdr.deserialize_array(array_ptr, size);
  }

  // Field name: pos_out_limit
  {
    size_t size = 4;
    auto array_ptr = ros_message->pos_out_limit;
    cdr.deserialize_array(array_ptr, size);
  }

  // Field name: vel_kp
  {
    size_t size = 4;
    auto array_ptr = ros_message->vel_kp;
    cdr.deserialize_array(array_ptr, size);
  }

  // Field name: vel_ki
  {
    size_t size = 4;
    auto array_ptr = ros_message->vel_ki;
    cdr.deserialize_array(array_ptr, size);
  }

  // Field name: vel_kd
  {
    size_t size = 4;
    auto array_ptr = ros_message->vel_kd;
    cdr.deserialize_array(array_ptr, size);
  }

  // Field name: vel_i_limit
  {
    size_t size = 4;
    auto array_ptr = ros_message->vel_i_limit;
    cdr.deserialize_array(array_ptr, size);
  }

  // Field name: vel_out_limit
  {
    size_t size = 4;
    auto array_ptr = ros_message->vel_out_limit;
    cdr.deserialize_array(array_ptr, size);
  }

  return true;
}  // NOLINT(readability/fn_size)


ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_zit6_interfaces
size_t get_serialized_size_zit6_interfaces__msg__ZitPidStatus(
  const void * untyped_ros_message,
  size_t current_alignment)
{
  const _ZitPidStatus__ros_msg_type * ros_message = static_cast<const _ZitPidStatus__ros_msg_type *>(untyped_ros_message);
  (void)ros_message;
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;

  // Field name: pos_kp
  {
    size_t array_size = 4;
    auto array_ptr = ros_message->pos_kp;
    (void)array_ptr;
    size_t item_size = sizeof(array_ptr[0]);
    current_alignment += array_size * item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  // Field name: pos_max_v
  {
    size_t array_size = 4;
    auto array_ptr = ros_message->pos_max_v;
    (void)array_ptr;
    size_t item_size = sizeof(array_ptr[0]);
    current_alignment += array_size * item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  // Field name: pos_max_a
  {
    size_t array_size = 4;
    auto array_ptr = ros_message->pos_max_a;
    (void)array_ptr;
    size_t item_size = sizeof(array_ptr[0]);
    current_alignment += array_size * item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  // Field name: pos_out_limit
  {
    size_t array_size = 4;
    auto array_ptr = ros_message->pos_out_limit;
    (void)array_ptr;
    size_t item_size = sizeof(array_ptr[0]);
    current_alignment += array_size * item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  // Field name: vel_kp
  {
    size_t array_size = 4;
    auto array_ptr = ros_message->vel_kp;
    (void)array_ptr;
    size_t item_size = sizeof(array_ptr[0]);
    current_alignment += array_size * item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  // Field name: vel_ki
  {
    size_t array_size = 4;
    auto array_ptr = ros_message->vel_ki;
    (void)array_ptr;
    size_t item_size = sizeof(array_ptr[0]);
    current_alignment += array_size * item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  // Field name: vel_kd
  {
    size_t array_size = 4;
    auto array_ptr = ros_message->vel_kd;
    (void)array_ptr;
    size_t item_size = sizeof(array_ptr[0]);
    current_alignment += array_size * item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  // Field name: vel_i_limit
  {
    size_t array_size = 4;
    auto array_ptr = ros_message->vel_i_limit;
    (void)array_ptr;
    size_t item_size = sizeof(array_ptr[0]);
    current_alignment += array_size * item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  // Field name: vel_out_limit
  {
    size_t array_size = 4;
    auto array_ptr = ros_message->vel_out_limit;
    (void)array_ptr;
    size_t item_size = sizeof(array_ptr[0]);
    current_alignment += array_size * item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  return current_alignment - initial_alignment;
}


ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_zit6_interfaces
size_t max_serialized_size_zit6_interfaces__msg__ZitPidStatus(
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

  // Field name: pos_kp
  {
    size_t array_size = 4;
    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  // Field name: pos_max_v
  {
    size_t array_size = 4;
    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  // Field name: pos_max_a
  {
    size_t array_size = 4;
    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  // Field name: pos_out_limit
  {
    size_t array_size = 4;
    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  // Field name: vel_kp
  {
    size_t array_size = 4;
    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  // Field name: vel_ki
  {
    size_t array_size = 4;
    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  // Field name: vel_kd
  {
    size_t array_size = 4;
    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  // Field name: vel_i_limit
  {
    size_t array_size = 4;
    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  // Field name: vel_out_limit
  {
    size_t array_size = 4;
    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }


  size_t ret_val = current_alignment - initial_alignment;
  if (is_plain) {
    // All members are plain, and type is not empty.
    // We still need to check that the in-memory alignment
    // is the same as the CDR mandated alignment.
    using DataType = zit6_interfaces__msg__ZitPidStatus;
    is_plain =
      (
      offsetof(DataType, vel_out_limit) +
      last_member_size
      ) == ret_val;
  }
  return ret_val;
}

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_zit6_interfaces
bool cdr_serialize_key_zit6_interfaces__msg__ZitPidStatus(
  const zit6_interfaces__msg__ZitPidStatus * ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  // Field name: pos_kp
  {
    size_t size = 4;
    auto array_ptr = ros_message->pos_kp;
    cdr.serialize_array(array_ptr, size);
  }

  // Field name: pos_max_v
  {
    size_t size = 4;
    auto array_ptr = ros_message->pos_max_v;
    cdr.serialize_array(array_ptr, size);
  }

  // Field name: pos_max_a
  {
    size_t size = 4;
    auto array_ptr = ros_message->pos_max_a;
    cdr.serialize_array(array_ptr, size);
  }

  // Field name: pos_out_limit
  {
    size_t size = 4;
    auto array_ptr = ros_message->pos_out_limit;
    cdr.serialize_array(array_ptr, size);
  }

  // Field name: vel_kp
  {
    size_t size = 4;
    auto array_ptr = ros_message->vel_kp;
    cdr.serialize_array(array_ptr, size);
  }

  // Field name: vel_ki
  {
    size_t size = 4;
    auto array_ptr = ros_message->vel_ki;
    cdr.serialize_array(array_ptr, size);
  }

  // Field name: vel_kd
  {
    size_t size = 4;
    auto array_ptr = ros_message->vel_kd;
    cdr.serialize_array(array_ptr, size);
  }

  // Field name: vel_i_limit
  {
    size_t size = 4;
    auto array_ptr = ros_message->vel_i_limit;
    cdr.serialize_array(array_ptr, size);
  }

  // Field name: vel_out_limit
  {
    size_t size = 4;
    auto array_ptr = ros_message->vel_out_limit;
    cdr.serialize_array(array_ptr, size);
  }

  return true;
}

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_zit6_interfaces
size_t get_serialized_size_key_zit6_interfaces__msg__ZitPidStatus(
  const void * untyped_ros_message,
  size_t current_alignment)
{
  const _ZitPidStatus__ros_msg_type * ros_message = static_cast<const _ZitPidStatus__ros_msg_type *>(untyped_ros_message);
  (void)ros_message;

  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;

  // Field name: pos_kp
  {
    size_t array_size = 4;
    auto array_ptr = ros_message->pos_kp;
    (void)array_ptr;
    size_t item_size = sizeof(array_ptr[0]);
    current_alignment += array_size * item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  // Field name: pos_max_v
  {
    size_t array_size = 4;
    auto array_ptr = ros_message->pos_max_v;
    (void)array_ptr;
    size_t item_size = sizeof(array_ptr[0]);
    current_alignment += array_size * item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  // Field name: pos_max_a
  {
    size_t array_size = 4;
    auto array_ptr = ros_message->pos_max_a;
    (void)array_ptr;
    size_t item_size = sizeof(array_ptr[0]);
    current_alignment += array_size * item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  // Field name: pos_out_limit
  {
    size_t array_size = 4;
    auto array_ptr = ros_message->pos_out_limit;
    (void)array_ptr;
    size_t item_size = sizeof(array_ptr[0]);
    current_alignment += array_size * item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  // Field name: vel_kp
  {
    size_t array_size = 4;
    auto array_ptr = ros_message->vel_kp;
    (void)array_ptr;
    size_t item_size = sizeof(array_ptr[0]);
    current_alignment += array_size * item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  // Field name: vel_ki
  {
    size_t array_size = 4;
    auto array_ptr = ros_message->vel_ki;
    (void)array_ptr;
    size_t item_size = sizeof(array_ptr[0]);
    current_alignment += array_size * item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  // Field name: vel_kd
  {
    size_t array_size = 4;
    auto array_ptr = ros_message->vel_kd;
    (void)array_ptr;
    size_t item_size = sizeof(array_ptr[0]);
    current_alignment += array_size * item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  // Field name: vel_i_limit
  {
    size_t array_size = 4;
    auto array_ptr = ros_message->vel_i_limit;
    (void)array_ptr;
    size_t item_size = sizeof(array_ptr[0]);
    current_alignment += array_size * item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  // Field name: vel_out_limit
  {
    size_t array_size = 4;
    auto array_ptr = ros_message->vel_out_limit;
    (void)array_ptr;
    size_t item_size = sizeof(array_ptr[0]);
    current_alignment += array_size * item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  return current_alignment - initial_alignment;
}

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_zit6_interfaces
size_t max_serialized_size_key_zit6_interfaces__msg__ZitPidStatus(
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
  // Field name: pos_kp
  {
    size_t array_size = 4;
    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  // Field name: pos_max_v
  {
    size_t array_size = 4;
    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  // Field name: pos_max_a
  {
    size_t array_size = 4;
    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  // Field name: pos_out_limit
  {
    size_t array_size = 4;
    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  // Field name: vel_kp
  {
    size_t array_size = 4;
    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  // Field name: vel_ki
  {
    size_t array_size = 4;
    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  // Field name: vel_kd
  {
    size_t array_size = 4;
    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  // Field name: vel_i_limit
  {
    size_t array_size = 4;
    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  // Field name: vel_out_limit
  {
    size_t array_size = 4;
    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  size_t ret_val = current_alignment - initial_alignment;
  if (is_plain) {
    // All members are plain, and type is not empty.
    // We still need to check that the in-memory alignment
    // is the same as the CDR mandated alignment.
    using DataType = zit6_interfaces__msg__ZitPidStatus;
    is_plain =
      (
      offsetof(DataType, vel_out_limit) +
      last_member_size
      ) == ret_val;
  }
  return ret_val;
}


static bool _ZitPidStatus__cdr_serialize(
  const void * untyped_ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  if (!untyped_ros_message) {
    fprintf(stderr, "ros message handle is null\n");
    return false;
  }
  const zit6_interfaces__msg__ZitPidStatus * ros_message = static_cast<const zit6_interfaces__msg__ZitPidStatus *>(untyped_ros_message);
  (void)ros_message;
  return cdr_serialize_zit6_interfaces__msg__ZitPidStatus(ros_message, cdr);
}

static bool _ZitPidStatus__cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  void * untyped_ros_message)
{
  if (!untyped_ros_message) {
    fprintf(stderr, "ros message handle is null\n");
    return false;
  }
  zit6_interfaces__msg__ZitPidStatus * ros_message = static_cast<zit6_interfaces__msg__ZitPidStatus *>(untyped_ros_message);
  (void)ros_message;
  return cdr_deserialize_zit6_interfaces__msg__ZitPidStatus(cdr, ros_message);
}

static uint32_t _ZitPidStatus__get_serialized_size(const void * untyped_ros_message)
{
  return static_cast<uint32_t>(
    get_serialized_size_zit6_interfaces__msg__ZitPidStatus(
      untyped_ros_message, 0));
}

static size_t _ZitPidStatus__max_serialized_size(char & bounds_info)
{
  bool full_bounded;
  bool is_plain;
  size_t ret_val;

  ret_val = max_serialized_size_zit6_interfaces__msg__ZitPidStatus(
    full_bounded, is_plain, 0);

  bounds_info =
    is_plain ? ROSIDL_TYPESUPPORT_FASTRTPS_PLAIN_TYPE :
    full_bounded ? ROSIDL_TYPESUPPORT_FASTRTPS_BOUNDED_TYPE : ROSIDL_TYPESUPPORT_FASTRTPS_UNBOUNDED_TYPE;
  return ret_val;
}


static message_type_support_callbacks_t __callbacks_ZitPidStatus = {
  "zit6_interfaces::msg",
  "ZitPidStatus",
  _ZitPidStatus__cdr_serialize,
  _ZitPidStatus__cdr_deserialize,
  _ZitPidStatus__get_serialized_size,
  _ZitPidStatus__max_serialized_size,
  nullptr
};

static rosidl_message_type_support_t _ZitPidStatus__type_support = {
  rosidl_typesupport_fastrtps_c__identifier,
  &__callbacks_ZitPidStatus,
  get_message_typesupport_handle_function,
  &zit6_interfaces__msg__ZitPidStatus__get_type_hash,
  &zit6_interfaces__msg__ZitPidStatus__get_type_description,
  &zit6_interfaces__msg__ZitPidStatus__get_type_description_sources,
};

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, zit6_interfaces, msg, ZitPidStatus)() {
  return &_ZitPidStatus__type_support;
}

#if defined(__cplusplus)
}
#endif
