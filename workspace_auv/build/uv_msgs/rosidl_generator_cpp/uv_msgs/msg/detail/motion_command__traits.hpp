// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from uv_msgs:msg/MotionCommand.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "uv_msgs/msg/motion_command.hpp"


#ifndef UV_MSGS__MSG__DETAIL__MOTION_COMMAND__TRAITS_HPP_
#define UV_MSGS__MSG__DETAIL__MOTION_COMMAND__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "uv_msgs/msg/detail/motion_command__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace uv_msgs
{

namespace msg
{

inline void to_flow_style_yaml(
  const MotionCommand & msg,
  std::ostream & out)
{
  out << "{";
  // member: mode
  {
    out << "mode: ";
    rosidl_generator_traits::value_to_yaml(msg.mode, out);
    out << ", ";
  }

  // member: type_mask
  {
    out << "type_mask: ";
    rosidl_generator_traits::value_to_yaml(msg.type_mask, out);
    out << ", ";
  }

  // member: x
  {
    out << "x: ";
    rosidl_generator_traits::value_to_yaml(msg.x, out);
    out << ", ";
  }

  // member: y
  {
    out << "y: ";
    rosidl_generator_traits::value_to_yaml(msg.y, out);
    out << ", ";
  }

  // member: z
  {
    out << "z: ";
    rosidl_generator_traits::value_to_yaml(msg.z, out);
    out << ", ";
  }

  // member: yaw
  {
    out << "yaw: ";
    rosidl_generator_traits::value_to_yaml(msg.yaw, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const MotionCommand & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: mode
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "mode: ";
    rosidl_generator_traits::value_to_yaml(msg.mode, out);
    out << "\n";
  }

  // member: type_mask
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "type_mask: ";
    rosidl_generator_traits::value_to_yaml(msg.type_mask, out);
    out << "\n";
  }

  // member: x
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "x: ";
    rosidl_generator_traits::value_to_yaml(msg.x, out);
    out << "\n";
  }

  // member: y
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "y: ";
    rosidl_generator_traits::value_to_yaml(msg.y, out);
    out << "\n";
  }

  // member: z
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "z: ";
    rosidl_generator_traits::value_to_yaml(msg.z, out);
    out << "\n";
  }

  // member: yaw
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "yaw: ";
    rosidl_generator_traits::value_to_yaml(msg.yaw, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const MotionCommand & msg, bool use_flow_style = false)
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
  const uv_msgs::msg::MotionCommand & msg,
  std::ostream & out, size_t indentation = 0)
{
  uv_msgs::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use uv_msgs::msg::to_yaml() instead")]]
inline std::string to_yaml(const uv_msgs::msg::MotionCommand & msg)
{
  return uv_msgs::msg::to_yaml(msg);
}

template<>
inline const char * data_type<uv_msgs::msg::MotionCommand>()
{
  return "uv_msgs::msg::MotionCommand";
}

template<>
inline const char * name<uv_msgs::msg::MotionCommand>()
{
  return "uv_msgs/msg/MotionCommand";
}

template<>
struct has_fixed_size<uv_msgs::msg::MotionCommand>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<uv_msgs::msg::MotionCommand>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<uv_msgs::msg::MotionCommand>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // UV_MSGS__MSG__DETAIL__MOTION_COMMAND__TRAITS_HPP_
