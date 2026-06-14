// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from uv_msgs:msg/PidState.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "uv_msgs/msg/pid_state.hpp"


#ifndef UV_MSGS__MSG__DETAIL__PID_STATE__TRAITS_HPP_
#define UV_MSGS__MSG__DETAIL__PID_STATE__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "uv_msgs/msg/detail/pid_state__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

// Include directives for member types
// Member 'speed_mag'
// Member 'vel_x'
// Member 'vel_y'
// Member 'pos_z'
// Member 'yaw'
// Member 'yaw_rate'
#include "uv_msgs/msg/detail/pid_gains__traits.hpp"

namespace uv_msgs
{

namespace msg
{

inline void to_flow_style_yaml(
  const PidState & msg,
  std::ostream & out)
{
  out << "{";
  // member: speed_mag
  {
    out << "speed_mag: ";
    to_flow_style_yaml(msg.speed_mag, out);
    out << ", ";
  }

  // member: vel_x
  {
    out << "vel_x: ";
    to_flow_style_yaml(msg.vel_x, out);
    out << ", ";
  }

  // member: vel_y
  {
    out << "vel_y: ";
    to_flow_style_yaml(msg.vel_y, out);
    out << ", ";
  }

  // member: pos_z
  {
    out << "pos_z: ";
    to_flow_style_yaml(msg.pos_z, out);
    out << ", ";
  }

  // member: yaw
  {
    out << "yaw: ";
    to_flow_style_yaml(msg.yaw, out);
    out << ", ";
  }

  // member: yaw_rate
  {
    out << "yaw_rate: ";
    to_flow_style_yaml(msg.yaw_rate, out);
    out << ", ";
  }

  // member: error_length
  {
    out << "error_length: ";
    rosidl_generator_traits::value_to_yaml(msg.error_length, out);
    out << ", ";
  }

  // member: error_angle_deg
  {
    out << "error_angle_deg: ";
    rosidl_generator_traits::value_to_yaml(msg.error_angle_deg, out);
    out << ", ";
  }

  // member: speed_cmd
  {
    out << "speed_cmd: ";
    rosidl_generator_traits::value_to_yaml(msg.speed_cmd, out);
    out << ", ";
  }

  // member: vx_body_cmd
  {
    out << "vx_body_cmd: ";
    rosidl_generator_traits::value_to_yaml(msg.vx_body_cmd, out);
    out << ", ";
  }

  // member: vy_body_cmd
  {
    out << "vy_body_cmd: ";
    rosidl_generator_traits::value_to_yaml(msg.vy_body_cmd, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const PidState & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: speed_mag
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "speed_mag:\n";
    to_block_style_yaml(msg.speed_mag, out, indentation + 2);
  }

  // member: vel_x
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "vel_x:\n";
    to_block_style_yaml(msg.vel_x, out, indentation + 2);
  }

  // member: vel_y
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "vel_y:\n";
    to_block_style_yaml(msg.vel_y, out, indentation + 2);
  }

  // member: pos_z
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "pos_z:\n";
    to_block_style_yaml(msg.pos_z, out, indentation + 2);
  }

  // member: yaw
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "yaw:\n";
    to_block_style_yaml(msg.yaw, out, indentation + 2);
  }

  // member: yaw_rate
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "yaw_rate:\n";
    to_block_style_yaml(msg.yaw_rate, out, indentation + 2);
  }

  // member: error_length
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "error_length: ";
    rosidl_generator_traits::value_to_yaml(msg.error_length, out);
    out << "\n";
  }

  // member: error_angle_deg
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "error_angle_deg: ";
    rosidl_generator_traits::value_to_yaml(msg.error_angle_deg, out);
    out << "\n";
  }

  // member: speed_cmd
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "speed_cmd: ";
    rosidl_generator_traits::value_to_yaml(msg.speed_cmd, out);
    out << "\n";
  }

  // member: vx_body_cmd
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "vx_body_cmd: ";
    rosidl_generator_traits::value_to_yaml(msg.vx_body_cmd, out);
    out << "\n";
  }

  // member: vy_body_cmd
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "vy_body_cmd: ";
    rosidl_generator_traits::value_to_yaml(msg.vy_body_cmd, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const PidState & msg, bool use_flow_style = false)
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
  const uv_msgs::msg::PidState & msg,
  std::ostream & out, size_t indentation = 0)
{
  uv_msgs::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use uv_msgs::msg::to_yaml() instead")]]
inline std::string to_yaml(const uv_msgs::msg::PidState & msg)
{
  return uv_msgs::msg::to_yaml(msg);
}

template<>
inline const char * data_type<uv_msgs::msg::PidState>()
{
  return "uv_msgs::msg::PidState";
}

template<>
inline const char * name<uv_msgs::msg::PidState>()
{
  return "uv_msgs/msg/PidState";
}

template<>
struct has_fixed_size<uv_msgs::msg::PidState>
  : std::integral_constant<bool, has_fixed_size<uv_msgs::msg::PidGains>::value> {};

template<>
struct has_bounded_size<uv_msgs::msg::PidState>
  : std::integral_constant<bool, has_bounded_size<uv_msgs::msg::PidGains>::value> {};

template<>
struct is_message<uv_msgs::msg::PidState>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // UV_MSGS__MSG__DETAIL__PID_STATE__TRAITS_HPP_
