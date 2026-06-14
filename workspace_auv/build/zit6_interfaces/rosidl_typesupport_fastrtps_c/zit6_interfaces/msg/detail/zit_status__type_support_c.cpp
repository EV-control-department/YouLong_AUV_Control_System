// generated from rosidl_typesupport_fastrtps_c/resource/idl__type_support_c.cpp.em
// with input from zit6_interfaces:msg/ZitStatus.idl
// generated code does not contain a copyright notice
#include "zit6_interfaces/msg/detail/zit_status__rosidl_typesupport_fastrtps_c.h"


#include <cassert>
#include <cstddef>
#include <limits>
#include <string>
#include "rosidl_typesupport_fastrtps_c/identifier.h"
#include "rosidl_typesupport_fastrtps_c/serialization_helpers.hpp"
#include "rosidl_typesupport_fastrtps_c/wstring_conversion.hpp"
#include "rosidl_typesupport_fastrtps_cpp/message_type_support.h"
#include "zit6_interfaces/msg/rosidl_typesupport_fastrtps_c__visibility_control.h"
#include "zit6_interfaces/msg/detail/zit_status__struct.h"
#include "zit6_interfaces/msg/detail/zit_status__functions.h"
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


using _ZitStatus__ros_msg_type = zit6_interfaces__msg__ZitStatus;


ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_zit6_interfaces
bool cdr_serialize_zit6_interfaces__msg__ZitStatus(
  const zit6_interfaces__msg__ZitStatus * ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  // Field name: is_armed
  {
    cdr << (ros_message->is_armed ? true : false);
  }

  // Field name: arm_mode
  {
    cdr << ros_message->arm_mode;
  }

  // Field name: control_level
  {
    cdr << ros_message->control_level;
  }

  // Field name: ins_state
  {
    cdr << ros_message->ins_state;
  }

  // Field name: navigation_ready
  {
    cdr << (ros_message->navigation_ready ? true : false);
  }

  // Field name: forces
  {
    size_t size = 4;
    auto array_ptr = ros_message->forces;
    cdr.serialize_array(array_ptr, size);
  }

  // Field name: cycle_time_ms
  {
    cdr << ros_message->cycle_time_ms;
  }

  // Field name: battery_voltage
  {
    cdr << ros_message->battery_voltage;
  }

  // Field name: error_flags
  {
    cdr << ros_message->error_flags;
  }

  return true;
}

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_zit6_interfaces
bool cdr_deserialize_zit6_interfaces__msg__ZitStatus(
  eprosima::fastcdr::Cdr & cdr,
  zit6_interfaces__msg__ZitStatus * ros_message)
{
  // Field name: is_armed
  {
    uint8_t tmp;
    cdr >> tmp;
    ros_message->is_armed = tmp ? true : false;
  }

  // Field name: arm_mode
  {
    cdr >> ros_message->arm_mode;
  }

  // Field name: control_level
  {
    cdr >> ros_message->control_level;
  }

  // Field name: ins_state
  {
    cdr >> ros_message->ins_state;
  }

  // Field name: navigation_ready
  {
    uint8_t tmp;
    cdr >> tmp;
    ros_message->navigation_ready = tmp ? true : false;
  }

  // Field name: forces
  {
    size_t size = 4;
    auto array_ptr = ros_message->forces;
    cdr.deserialize_array(array_ptr, size);
  }

  // Field name: cycle_time_ms
  {
    cdr >> ros_message->cycle_time_ms;
  }

  // Field name: battery_voltage
  {
    cdr >> ros_message->battery_voltage;
  }

  // Field name: error_flags
  {
    cdr >> ros_message->error_flags;
  }

  return true;
}  // NOLINT(readability/fn_size)


ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_zit6_interfaces
size_t get_serialized_size_zit6_interfaces__msg__ZitStatus(
  const void * untyped_ros_message,
  size_t current_alignment)
{
  const _ZitStatus__ros_msg_type * ros_message = static_cast<const _ZitStatus__ros_msg_type *>(untyped_ros_message);
  (void)ros_message;
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;

  // Field name: is_armed
  {
    size_t item_size = sizeof(ros_message->is_armed);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  // Field name: arm_mode
  {
    size_t item_size = sizeof(ros_message->arm_mode);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  // Field name: control_level
  {
    size_t item_size = sizeof(ros_message->control_level);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  // Field name: ins_state
  {
    size_t item_size = sizeof(ros_message->ins_state);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  // Field name: navigation_ready
  {
    size_t item_size = sizeof(ros_message->navigation_ready);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  // Field name: forces
  {
    size_t array_size = 4;
    auto array_ptr = ros_message->forces;
    (void)array_ptr;
    size_t item_size = sizeof(array_ptr[0]);
    current_alignment += array_size * item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  // Field name: cycle_time_ms
  {
    size_t item_size = sizeof(ros_message->cycle_time_ms);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  // Field name: battery_voltage
  {
    size_t item_size = sizeof(ros_message->battery_voltage);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  // Field name: error_flags
  {
    size_t item_size = sizeof(ros_message->error_flags);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  return current_alignment - initial_alignment;
}


ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_zit6_interfaces
size_t max_serialized_size_zit6_interfaces__msg__ZitStatus(
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

  // Field name: is_armed
  {
    size_t array_size = 1;
    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }

  // Field name: arm_mode
  {
    size_t array_size = 1;
    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }

  // Field name: control_level
  {
    size_t array_size = 1;
    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }

  // Field name: ins_state
  {
    size_t array_size = 1;
    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }

  // Field name: navigation_ready
  {
    size_t array_size = 1;
    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }

  // Field name: forces
  {
    size_t array_size = 4;
    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  // Field name: cycle_time_ms
  {
    size_t array_size = 1;
    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  // Field name: battery_voltage
  {
    size_t array_size = 1;
    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  // Field name: error_flags
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
    using DataType = zit6_interfaces__msg__ZitStatus;
    is_plain =
      (
      offsetof(DataType, error_flags) +
      last_member_size
      ) == ret_val;
  }
  return ret_val;
}

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_zit6_interfaces
bool cdr_serialize_key_zit6_interfaces__msg__ZitStatus(
  const zit6_interfaces__msg__ZitStatus * ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  // Field name: is_armed
  {
    cdr << (ros_message->is_armed ? true : false);
  }

  // Field name: arm_mode
  {
    cdr << ros_message->arm_mode;
  }

  // Field name: control_level
  {
    cdr << ros_message->control_level;
  }

  // Field name: ins_state
  {
    cdr << ros_message->ins_state;
  }

  // Field name: navigation_ready
  {
    cdr << (ros_message->navigation_ready ? true : false);
  }

  // Field name: forces
  {
    size_t size = 4;
    auto array_ptr = ros_message->forces;
    cdr.serialize_array(array_ptr, size);
  }

  // Field name: cycle_time_ms
  {
    cdr << ros_message->cycle_time_ms;
  }

  // Field name: battery_voltage
  {
    cdr << ros_message->battery_voltage;
  }

  // Field name: error_flags
  {
    cdr << ros_message->error_flags;
  }

  return true;
}

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_zit6_interfaces
size_t get_serialized_size_key_zit6_interfaces__msg__ZitStatus(
  const void * untyped_ros_message,
  size_t current_alignment)
{
  const _ZitStatus__ros_msg_type * ros_message = static_cast<const _ZitStatus__ros_msg_type *>(untyped_ros_message);
  (void)ros_message;

  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;

  // Field name: is_armed
  {
    size_t item_size = sizeof(ros_message->is_armed);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  // Field name: arm_mode
  {
    size_t item_size = sizeof(ros_message->arm_mode);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  // Field name: control_level
  {
    size_t item_size = sizeof(ros_message->control_level);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  // Field name: ins_state
  {
    size_t item_size = sizeof(ros_message->ins_state);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  // Field name: navigation_ready
  {
    size_t item_size = sizeof(ros_message->navigation_ready);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  // Field name: forces
  {
    size_t array_size = 4;
    auto array_ptr = ros_message->forces;
    (void)array_ptr;
    size_t item_size = sizeof(array_ptr[0]);
    current_alignment += array_size * item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  // Field name: cycle_time_ms
  {
    size_t item_size = sizeof(ros_message->cycle_time_ms);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  // Field name: battery_voltage
  {
    size_t item_size = sizeof(ros_message->battery_voltage);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  // Field name: error_flags
  {
    size_t item_size = sizeof(ros_message->error_flags);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  return current_alignment - initial_alignment;
}

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_zit6_interfaces
size_t max_serialized_size_key_zit6_interfaces__msg__ZitStatus(
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
  // Field name: is_armed
  {
    size_t array_size = 1;
    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }

  // Field name: arm_mode
  {
    size_t array_size = 1;
    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }

  // Field name: control_level
  {
    size_t array_size = 1;
    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }

  // Field name: ins_state
  {
    size_t array_size = 1;
    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }

  // Field name: navigation_ready
  {
    size_t array_size = 1;
    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }

  // Field name: forces
  {
    size_t array_size = 4;
    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  // Field name: cycle_time_ms
  {
    size_t array_size = 1;
    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  // Field name: battery_voltage
  {
    size_t array_size = 1;
    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  // Field name: error_flags
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
    using DataType = zit6_interfaces__msg__ZitStatus;
    is_plain =
      (
      offsetof(DataType, error_flags) +
      last_member_size
      ) == ret_val;
  }
  return ret_val;
}


static bool _ZitStatus__cdr_serialize(
  const void * untyped_ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  if (!untyped_ros_message) {
    fprintf(stderr, "ros message handle is null\n");
    return false;
  }
  const zit6_interfaces__msg__ZitStatus * ros_message = static_cast<const zit6_interfaces__msg__ZitStatus *>(untyped_ros_message);
  (void)ros_message;
  return cdr_serialize_zit6_interfaces__msg__ZitStatus(ros_message, cdr);
}

static bool _ZitStatus__cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  void * untyped_ros_message)
{
  if (!untyped_ros_message) {
    fprintf(stderr, "ros message handle is null\n");
    return false;
  }
  zit6_interfaces__msg__ZitStatus * ros_message = static_cast<zit6_interfaces__msg__ZitStatus *>(untyped_ros_message);
  (void)ros_message;
  return cdr_deserialize_zit6_interfaces__msg__ZitStatus(cdr, ros_message);
}

static uint32_t _ZitStatus__get_serialized_size(const void * untyped_ros_message)
{
  return static_cast<uint32_t>(
    get_serialized_size_zit6_interfaces__msg__ZitStatus(
      untyped_ros_message, 0));
}

static size_t _ZitStatus__max_serialized_size(char & bounds_info)
{
  bool full_bounded;
  bool is_plain;
  size_t ret_val;

  ret_val = max_serialized_size_zit6_interfaces__msg__ZitStatus(
    full_bounded, is_plain, 0);

  bounds_info =
    is_plain ? ROSIDL_TYPESUPPORT_FASTRTPS_PLAIN_TYPE :
    full_bounded ? ROSIDL_TYPESUPPORT_FASTRTPS_BOUNDED_TYPE : ROSIDL_TYPESUPPORT_FASTRTPS_UNBOUNDED_TYPE;
  return ret_val;
}


static message_type_support_callbacks_t __callbacks_ZitStatus = {
  "zit6_interfaces::msg",
  "ZitStatus",
  _ZitStatus__cdr_serialize,
  _ZitStatus__cdr_deserialize,
  _ZitStatus__get_serialized_size,
  _ZitStatus__max_serialized_size,
  nullptr
};

static rosidl_message_type_support_t _ZitStatus__type_support = {
  rosidl_typesupport_fastrtps_c__identifier,
  &__callbacks_ZitStatus,
  get_message_typesupport_handle_function,
  &zit6_interfaces__msg__ZitStatus__get_type_hash,
  &zit6_interfaces__msg__ZitStatus__get_type_description,
  &zit6_interfaces__msg__ZitStatus__get_type_description_sources,
};

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, zit6_interfaces, msg, ZitStatus)() {
  return &_ZitStatus__type_support;
}

#if defined(__cplusplus)
}
#endif
