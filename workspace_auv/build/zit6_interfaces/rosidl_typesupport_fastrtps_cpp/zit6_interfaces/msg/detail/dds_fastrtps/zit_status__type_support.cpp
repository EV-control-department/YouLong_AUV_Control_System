// generated from rosidl_typesupport_fastrtps_cpp/resource/idl__type_support.cpp.em
// with input from zit6_interfaces:msg/ZitStatus.idl
// generated code does not contain a copyright notice
#include "zit6_interfaces/msg/detail/zit_status__rosidl_typesupport_fastrtps_cpp.hpp"
#include "zit6_interfaces/msg/detail/zit_status__functions.h"
#include "zit6_interfaces/msg/detail/zit_status__struct.hpp"

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

namespace zit6_interfaces
{

namespace msg
{

namespace typesupport_fastrtps_cpp
{


bool
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_zit6_interfaces
cdr_serialize(
  const zit6_interfaces::msg::ZitStatus & ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  // Member: is_armed
  cdr << (ros_message.is_armed ? true : false);

  // Member: arm_mode
  cdr << ros_message.arm_mode;

  // Member: control_level
  cdr << ros_message.control_level;

  // Member: ins_state
  cdr << ros_message.ins_state;

  // Member: navigation_ready
  cdr << (ros_message.navigation_ready ? true : false);

  // Member: forces
  {
    cdr << ros_message.forces;
  }

  // Member: cycle_time_ms
  cdr << ros_message.cycle_time_ms;

  // Member: battery_voltage
  cdr << ros_message.battery_voltage;

  // Member: error_flags
  cdr << ros_message.error_flags;

  return true;
}

bool
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_zit6_interfaces
cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  zit6_interfaces::msg::ZitStatus & ros_message)
{
  // Member: is_armed
  {
    uint8_t tmp;
    cdr >> tmp;
    ros_message.is_armed = tmp ? true : false;
  }

  // Member: arm_mode
  cdr >> ros_message.arm_mode;

  // Member: control_level
  cdr >> ros_message.control_level;

  // Member: ins_state
  cdr >> ros_message.ins_state;

  // Member: navigation_ready
  {
    uint8_t tmp;
    cdr >> tmp;
    ros_message.navigation_ready = tmp ? true : false;
  }

  // Member: forces
  {
    cdr >> ros_message.forces;
  }

  // Member: cycle_time_ms
  cdr >> ros_message.cycle_time_ms;

  // Member: battery_voltage
  cdr >> ros_message.battery_voltage;

  // Member: error_flags
  cdr >> ros_message.error_flags;

  return true;
}  // NOLINT(readability/fn_size)


size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_zit6_interfaces
get_serialized_size(
  const zit6_interfaces::msg::ZitStatus & ros_message,
  size_t current_alignment)
{
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;

  // Member: is_armed
  {
    size_t item_size = sizeof(ros_message.is_armed);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  // Member: arm_mode
  {
    size_t item_size = sizeof(ros_message.arm_mode);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  // Member: control_level
  {
    size_t item_size = sizeof(ros_message.control_level);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  // Member: ins_state
  {
    size_t item_size = sizeof(ros_message.ins_state);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  // Member: navigation_ready
  {
    size_t item_size = sizeof(ros_message.navigation_ready);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  // Member: forces
  {
    size_t array_size = 4;
    size_t item_size = sizeof(ros_message.forces[0]);
    current_alignment += array_size * item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  // Member: cycle_time_ms
  {
    size_t item_size = sizeof(ros_message.cycle_time_ms);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  // Member: battery_voltage
  {
    size_t item_size = sizeof(ros_message.battery_voltage);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  // Member: error_flags
  {
    size_t item_size = sizeof(ros_message.error_flags);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  return current_alignment - initial_alignment;
}


size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_zit6_interfaces
max_serialized_size_ZitStatus(
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

  // Member: is_armed
  {
    size_t array_size = 1;
    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }
  // Member: arm_mode
  {
    size_t array_size = 1;
    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }
  // Member: control_level
  {
    size_t array_size = 1;
    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }
  // Member: ins_state
  {
    size_t array_size = 1;
    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }
  // Member: navigation_ready
  {
    size_t array_size = 1;
    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }
  // Member: forces
  {
    size_t array_size = 4;
    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }
  // Member: cycle_time_ms
  {
    size_t array_size = 1;
    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }
  // Member: battery_voltage
  {
    size_t array_size = 1;
    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }
  // Member: error_flags
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
    using DataType = zit6_interfaces::msg::ZitStatus;
    is_plain =
      (
      offsetof(DataType, error_flags) +
      last_member_size
      ) == ret_val;
  }

  return ret_val;
}

bool
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_zit6_interfaces
cdr_serialize_key(
  const zit6_interfaces::msg::ZitStatus & ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  // Member: is_armed
  cdr << (ros_message.is_armed ? true : false);

  // Member: arm_mode
  cdr << ros_message.arm_mode;

  // Member: control_level
  cdr << ros_message.control_level;

  // Member: ins_state
  cdr << ros_message.ins_state;

  // Member: navigation_ready
  cdr << (ros_message.navigation_ready ? true : false);

  // Member: forces
  {
    cdr << ros_message.forces;
  }

  // Member: cycle_time_ms
  cdr << ros_message.cycle_time_ms;

  // Member: battery_voltage
  cdr << ros_message.battery_voltage;

  // Member: error_flags
  cdr << ros_message.error_flags;

  return true;
}

size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_zit6_interfaces
get_serialized_size_key(
  const zit6_interfaces::msg::ZitStatus & ros_message,
  size_t current_alignment)
{
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;

  // Member: is_armed
  {
    size_t item_size = sizeof(ros_message.is_armed);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  // Member: arm_mode
  {
    size_t item_size = sizeof(ros_message.arm_mode);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  // Member: control_level
  {
    size_t item_size = sizeof(ros_message.control_level);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  // Member: ins_state
  {
    size_t item_size = sizeof(ros_message.ins_state);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  // Member: navigation_ready
  {
    size_t item_size = sizeof(ros_message.navigation_ready);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  // Member: forces
  {
    size_t array_size = 4;
    size_t item_size = sizeof(ros_message.forces[0]);
    current_alignment += array_size * item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  // Member: cycle_time_ms
  {
    size_t item_size = sizeof(ros_message.cycle_time_ms);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  // Member: battery_voltage
  {
    size_t item_size = sizeof(ros_message.battery_voltage);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  // Member: error_flags
  {
    size_t item_size = sizeof(ros_message.error_flags);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  return current_alignment - initial_alignment;
}

size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_zit6_interfaces
max_serialized_size_key_ZitStatus(
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

  // Member: is_armed
  {
    size_t array_size = 1;
    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }

  // Member: arm_mode
  {
    size_t array_size = 1;
    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }

  // Member: control_level
  {
    size_t array_size = 1;
    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }

  // Member: ins_state
  {
    size_t array_size = 1;
    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }

  // Member: navigation_ready
  {
    size_t array_size = 1;
    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }

  // Member: forces
  {
    size_t array_size = 4;
    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  // Member: cycle_time_ms
  {
    size_t array_size = 1;
    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  // Member: battery_voltage
  {
    size_t array_size = 1;
    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  // Member: error_flags
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
    using DataType = zit6_interfaces::msg::ZitStatus;
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
  auto typed_message =
    static_cast<const zit6_interfaces::msg::ZitStatus *>(
    untyped_ros_message);
  return cdr_serialize(*typed_message, cdr);
}

static bool _ZitStatus__cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  void * untyped_ros_message)
{
  auto typed_message =
    static_cast<zit6_interfaces::msg::ZitStatus *>(
    untyped_ros_message);
  return cdr_deserialize(cdr, *typed_message);
}

static uint32_t _ZitStatus__get_serialized_size(
  const void * untyped_ros_message)
{
  auto typed_message =
    static_cast<const zit6_interfaces::msg::ZitStatus *>(
    untyped_ros_message);
  return static_cast<uint32_t>(get_serialized_size(*typed_message, 0));
}

static size_t _ZitStatus__max_serialized_size(char & bounds_info)
{
  bool full_bounded;
  bool is_plain;
  size_t ret_val;

  ret_val = max_serialized_size_ZitStatus(full_bounded, is_plain, 0);

  bounds_info =
    is_plain ? ROSIDL_TYPESUPPORT_FASTRTPS_PLAIN_TYPE :
    full_bounded ? ROSIDL_TYPESUPPORT_FASTRTPS_BOUNDED_TYPE : ROSIDL_TYPESUPPORT_FASTRTPS_UNBOUNDED_TYPE;
  return ret_val;
}

static message_type_support_callbacks_t _ZitStatus__callbacks = {
  "zit6_interfaces::msg",
  "ZitStatus",
  _ZitStatus__cdr_serialize,
  _ZitStatus__cdr_deserialize,
  _ZitStatus__get_serialized_size,
  _ZitStatus__max_serialized_size,
  nullptr
};

static rosidl_message_type_support_t _ZitStatus__handle = {
  rosidl_typesupport_fastrtps_cpp::typesupport_identifier,
  &_ZitStatus__callbacks,
  get_message_typesupport_handle_function,
  &zit6_interfaces__msg__ZitStatus__get_type_hash,
  &zit6_interfaces__msg__ZitStatus__get_type_description,
  &zit6_interfaces__msg__ZitStatus__get_type_description_sources,
};

}  // namespace typesupport_fastrtps_cpp

}  // namespace msg

}  // namespace zit6_interfaces

namespace rosidl_typesupport_fastrtps_cpp
{

template<>
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_EXPORT_zit6_interfaces
const rosidl_message_type_support_t *
get_message_type_support_handle<zit6_interfaces::msg::ZitStatus>()
{
  return &zit6_interfaces::msg::typesupport_fastrtps_cpp::_ZitStatus__handle;
}

}  // namespace rosidl_typesupport_fastrtps_cpp

#ifdef __cplusplus
extern "C"
{
#endif

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_cpp, zit6_interfaces, msg, ZitStatus)() {
  return &zit6_interfaces::msg::typesupport_fastrtps_cpp::_ZitStatus__handle;
}

#ifdef __cplusplus
}
#endif
