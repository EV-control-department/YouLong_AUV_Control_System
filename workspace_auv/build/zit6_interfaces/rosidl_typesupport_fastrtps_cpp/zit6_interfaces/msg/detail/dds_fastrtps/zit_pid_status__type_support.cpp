// generated from rosidl_typesupport_fastrtps_cpp/resource/idl__type_support.cpp.em
// with input from zit6_interfaces:msg/ZitPidStatus.idl
// generated code does not contain a copyright notice
#include "zit6_interfaces/msg/detail/zit_pid_status__rosidl_typesupport_fastrtps_cpp.hpp"
#include "zit6_interfaces/msg/detail/zit_pid_status__functions.h"
#include "zit6_interfaces/msg/detail/zit_pid_status__struct.hpp"

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
  const zit6_interfaces::msg::ZitPidStatus & ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  // Member: pos_kp
  {
    cdr << ros_message.pos_kp;
  }

  // Member: pos_max_v
  {
    cdr << ros_message.pos_max_v;
  }

  // Member: pos_max_a
  {
    cdr << ros_message.pos_max_a;
  }

  // Member: pos_out_limit
  {
    cdr << ros_message.pos_out_limit;
  }

  // Member: vel_kp
  {
    cdr << ros_message.vel_kp;
  }

  // Member: vel_ki
  {
    cdr << ros_message.vel_ki;
  }

  // Member: vel_kd
  {
    cdr << ros_message.vel_kd;
  }

  // Member: vel_i_limit
  {
    cdr << ros_message.vel_i_limit;
  }

  // Member: vel_out_limit
  {
    cdr << ros_message.vel_out_limit;
  }

  return true;
}

bool
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_zit6_interfaces
cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  zit6_interfaces::msg::ZitPidStatus & ros_message)
{
  // Member: pos_kp
  {
    cdr >> ros_message.pos_kp;
  }

  // Member: pos_max_v
  {
    cdr >> ros_message.pos_max_v;
  }

  // Member: pos_max_a
  {
    cdr >> ros_message.pos_max_a;
  }

  // Member: pos_out_limit
  {
    cdr >> ros_message.pos_out_limit;
  }

  // Member: vel_kp
  {
    cdr >> ros_message.vel_kp;
  }

  // Member: vel_ki
  {
    cdr >> ros_message.vel_ki;
  }

  // Member: vel_kd
  {
    cdr >> ros_message.vel_kd;
  }

  // Member: vel_i_limit
  {
    cdr >> ros_message.vel_i_limit;
  }

  // Member: vel_out_limit
  {
    cdr >> ros_message.vel_out_limit;
  }

  return true;
}  // NOLINT(readability/fn_size)


size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_zit6_interfaces
get_serialized_size(
  const zit6_interfaces::msg::ZitPidStatus & ros_message,
  size_t current_alignment)
{
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;

  // Member: pos_kp
  {
    size_t array_size = 4;
    size_t item_size = sizeof(ros_message.pos_kp[0]);
    current_alignment += array_size * item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  // Member: pos_max_v
  {
    size_t array_size = 4;
    size_t item_size = sizeof(ros_message.pos_max_v[0]);
    current_alignment += array_size * item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  // Member: pos_max_a
  {
    size_t array_size = 4;
    size_t item_size = sizeof(ros_message.pos_max_a[0]);
    current_alignment += array_size * item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  // Member: pos_out_limit
  {
    size_t array_size = 4;
    size_t item_size = sizeof(ros_message.pos_out_limit[0]);
    current_alignment += array_size * item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  // Member: vel_kp
  {
    size_t array_size = 4;
    size_t item_size = sizeof(ros_message.vel_kp[0]);
    current_alignment += array_size * item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  // Member: vel_ki
  {
    size_t array_size = 4;
    size_t item_size = sizeof(ros_message.vel_ki[0]);
    current_alignment += array_size * item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  // Member: vel_kd
  {
    size_t array_size = 4;
    size_t item_size = sizeof(ros_message.vel_kd[0]);
    current_alignment += array_size * item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  // Member: vel_i_limit
  {
    size_t array_size = 4;
    size_t item_size = sizeof(ros_message.vel_i_limit[0]);
    current_alignment += array_size * item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  // Member: vel_out_limit
  {
    size_t array_size = 4;
    size_t item_size = sizeof(ros_message.vel_out_limit[0]);
    current_alignment += array_size * item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  return current_alignment - initial_alignment;
}


size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_zit6_interfaces
max_serialized_size_ZitPidStatus(
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

  // Member: pos_kp
  {
    size_t array_size = 4;
    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }
  // Member: pos_max_v
  {
    size_t array_size = 4;
    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }
  // Member: pos_max_a
  {
    size_t array_size = 4;
    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }
  // Member: pos_out_limit
  {
    size_t array_size = 4;
    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }
  // Member: vel_kp
  {
    size_t array_size = 4;
    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }
  // Member: vel_ki
  {
    size_t array_size = 4;
    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }
  // Member: vel_kd
  {
    size_t array_size = 4;
    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }
  // Member: vel_i_limit
  {
    size_t array_size = 4;
    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }
  // Member: vel_out_limit
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
    using DataType = zit6_interfaces::msg::ZitPidStatus;
    is_plain =
      (
      offsetof(DataType, vel_out_limit) +
      last_member_size
      ) == ret_val;
  }

  return ret_val;
}

bool
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_zit6_interfaces
cdr_serialize_key(
  const zit6_interfaces::msg::ZitPidStatus & ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  // Member: pos_kp
  {
    cdr << ros_message.pos_kp;
  }

  // Member: pos_max_v
  {
    cdr << ros_message.pos_max_v;
  }

  // Member: pos_max_a
  {
    cdr << ros_message.pos_max_a;
  }

  // Member: pos_out_limit
  {
    cdr << ros_message.pos_out_limit;
  }

  // Member: vel_kp
  {
    cdr << ros_message.vel_kp;
  }

  // Member: vel_ki
  {
    cdr << ros_message.vel_ki;
  }

  // Member: vel_kd
  {
    cdr << ros_message.vel_kd;
  }

  // Member: vel_i_limit
  {
    cdr << ros_message.vel_i_limit;
  }

  // Member: vel_out_limit
  {
    cdr << ros_message.vel_out_limit;
  }

  return true;
}

size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_zit6_interfaces
get_serialized_size_key(
  const zit6_interfaces::msg::ZitPidStatus & ros_message,
  size_t current_alignment)
{
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;

  // Member: pos_kp
  {
    size_t array_size = 4;
    size_t item_size = sizeof(ros_message.pos_kp[0]);
    current_alignment += array_size * item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  // Member: pos_max_v
  {
    size_t array_size = 4;
    size_t item_size = sizeof(ros_message.pos_max_v[0]);
    current_alignment += array_size * item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  // Member: pos_max_a
  {
    size_t array_size = 4;
    size_t item_size = sizeof(ros_message.pos_max_a[0]);
    current_alignment += array_size * item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  // Member: pos_out_limit
  {
    size_t array_size = 4;
    size_t item_size = sizeof(ros_message.pos_out_limit[0]);
    current_alignment += array_size * item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  // Member: vel_kp
  {
    size_t array_size = 4;
    size_t item_size = sizeof(ros_message.vel_kp[0]);
    current_alignment += array_size * item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  // Member: vel_ki
  {
    size_t array_size = 4;
    size_t item_size = sizeof(ros_message.vel_ki[0]);
    current_alignment += array_size * item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  // Member: vel_kd
  {
    size_t array_size = 4;
    size_t item_size = sizeof(ros_message.vel_kd[0]);
    current_alignment += array_size * item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  // Member: vel_i_limit
  {
    size_t array_size = 4;
    size_t item_size = sizeof(ros_message.vel_i_limit[0]);
    current_alignment += array_size * item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  // Member: vel_out_limit
  {
    size_t array_size = 4;
    size_t item_size = sizeof(ros_message.vel_out_limit[0]);
    current_alignment += array_size * item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  return current_alignment - initial_alignment;
}

size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_zit6_interfaces
max_serialized_size_key_ZitPidStatus(
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

  // Member: pos_kp
  {
    size_t array_size = 4;
    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  // Member: pos_max_v
  {
    size_t array_size = 4;
    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  // Member: pos_max_a
  {
    size_t array_size = 4;
    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  // Member: pos_out_limit
  {
    size_t array_size = 4;
    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  // Member: vel_kp
  {
    size_t array_size = 4;
    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  // Member: vel_ki
  {
    size_t array_size = 4;
    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  // Member: vel_kd
  {
    size_t array_size = 4;
    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  // Member: vel_i_limit
  {
    size_t array_size = 4;
    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  // Member: vel_out_limit
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
    using DataType = zit6_interfaces::msg::ZitPidStatus;
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
  auto typed_message =
    static_cast<const zit6_interfaces::msg::ZitPidStatus *>(
    untyped_ros_message);
  return cdr_serialize(*typed_message, cdr);
}

static bool _ZitPidStatus__cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  void * untyped_ros_message)
{
  auto typed_message =
    static_cast<zit6_interfaces::msg::ZitPidStatus *>(
    untyped_ros_message);
  return cdr_deserialize(cdr, *typed_message);
}

static uint32_t _ZitPidStatus__get_serialized_size(
  const void * untyped_ros_message)
{
  auto typed_message =
    static_cast<const zit6_interfaces::msg::ZitPidStatus *>(
    untyped_ros_message);
  return static_cast<uint32_t>(get_serialized_size(*typed_message, 0));
}

static size_t _ZitPidStatus__max_serialized_size(char & bounds_info)
{
  bool full_bounded;
  bool is_plain;
  size_t ret_val;

  ret_val = max_serialized_size_ZitPidStatus(full_bounded, is_plain, 0);

  bounds_info =
    is_plain ? ROSIDL_TYPESUPPORT_FASTRTPS_PLAIN_TYPE :
    full_bounded ? ROSIDL_TYPESUPPORT_FASTRTPS_BOUNDED_TYPE : ROSIDL_TYPESUPPORT_FASTRTPS_UNBOUNDED_TYPE;
  return ret_val;
}

static message_type_support_callbacks_t _ZitPidStatus__callbacks = {
  "zit6_interfaces::msg",
  "ZitPidStatus",
  _ZitPidStatus__cdr_serialize,
  _ZitPidStatus__cdr_deserialize,
  _ZitPidStatus__get_serialized_size,
  _ZitPidStatus__max_serialized_size,
  nullptr
};

static rosidl_message_type_support_t _ZitPidStatus__handle = {
  rosidl_typesupport_fastrtps_cpp::typesupport_identifier,
  &_ZitPidStatus__callbacks,
  get_message_typesupport_handle_function,
  &zit6_interfaces__msg__ZitPidStatus__get_type_hash,
  &zit6_interfaces__msg__ZitPidStatus__get_type_description,
  &zit6_interfaces__msg__ZitPidStatus__get_type_description_sources,
};

}  // namespace typesupport_fastrtps_cpp

}  // namespace msg

}  // namespace zit6_interfaces

namespace rosidl_typesupport_fastrtps_cpp
{

template<>
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_EXPORT_zit6_interfaces
const rosidl_message_type_support_t *
get_message_type_support_handle<zit6_interfaces::msg::ZitPidStatus>()
{
  return &zit6_interfaces::msg::typesupport_fastrtps_cpp::_ZitPidStatus__handle;
}

}  // namespace rosidl_typesupport_fastrtps_cpp

#ifdef __cplusplus
extern "C"
{
#endif

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_cpp, zit6_interfaces, msg, ZitPidStatus)() {
  return &zit6_interfaces::msg::typesupport_fastrtps_cpp::_ZitPidStatus__handle;
}

#ifdef __cplusplus
}
#endif
