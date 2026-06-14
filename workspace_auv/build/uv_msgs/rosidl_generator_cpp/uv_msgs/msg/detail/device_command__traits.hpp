// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from uv_msgs:msg/DeviceCommand.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "uv_msgs/msg/device_command.hpp"


#ifndef UV_MSGS__MSG__DETAIL__DEVICE_COMMAND__TRAITS_HPP_
#define UV_MSGS__MSG__DETAIL__DEVICE_COMMAND__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "uv_msgs/msg/detail/device_command__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace uv_msgs
{

namespace msg
{

inline void to_flow_style_yaml(
  const DeviceCommand & msg,
  std::ostream & out)
{
  out << "{";
  // member: device_type
  {
    out << "device_type: ";
    rosidl_generator_traits::value_to_yaml(msg.device_type, out);
    out << ", ";
  }

  // member: command
  {
    out << "command: ";
    rosidl_generator_traits::value_to_yaml(msg.command, out);
    out << ", ";
  }

  // member: value
  {
    out << "value: ";
    rosidl_generator_traits::value_to_yaml(msg.value, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const DeviceCommand & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: device_type
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "device_type: ";
    rosidl_generator_traits::value_to_yaml(msg.device_type, out);
    out << "\n";
  }

  // member: command
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "command: ";
    rosidl_generator_traits::value_to_yaml(msg.command, out);
    out << "\n";
  }

  // member: value
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "value: ";
    rosidl_generator_traits::value_to_yaml(msg.value, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const DeviceCommand & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace msg

}  // namespace uv_msgs

namespace rosidl_generator_traits
{

[[deprecated("use uv_msgs::msg::to_block_style_yaml() instead")]]
inline void to_yaml(
  const uv_msgs::msg::DeviceCommand & msg,
  std::ostream & out, size_t indentation = 0)
{
  uv_msgs::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use uv_msgs::msg::to_yaml() instead")]]
inline std::string to_yaml(const uv_msgs::msg::DeviceCommand & msg)
{
  return uv_msgs::msg::to_yaml(msg);
}

template<>
inline const char * data_type<uv_msgs::msg::DeviceCommand>()
{
  return "uv_msgs::msg::DeviceCommand";
}

template<>
inline const char * name<uv_msgs::msg::DeviceCommand>()
{
  return "uv_msgs/msg/DeviceCommand";
}

template<>
struct has_fixed_size<uv_msgs::msg::DeviceCommand>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<uv_msgs::msg::DeviceCommand>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<uv_msgs::msg::DeviceCommand>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // UV_MSGS__MSG__DETAIL__DEVICE_COMMAND__TRAITS_HPP_
