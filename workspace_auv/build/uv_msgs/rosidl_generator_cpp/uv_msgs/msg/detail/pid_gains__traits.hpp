// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from uv_msgs:msg/PidGains.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "uv_msgs/msg/pid_gains.hpp"


#ifndef UV_MSGS__MSG__DETAIL__PID_GAINS__TRAITS_HPP_
#define UV_MSGS__MSG__DETAIL__PID_GAINS__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "uv_msgs/msg/detail/pid_gains__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace uv_msgs
{

namespace msg
{

inline void to_flow_style_yaml(
  const PidGains & msg,
  std::ostream & out)
{
  out << "{";
  // member: name
  {
    out << "name: ";
    rosidl_generator_traits::value_to_yaml(msg.name, out);
    out << ", ";
  }

  // member: kp
  {
    out << "kp: ";
    rosidl_generator_traits::value_to_yaml(msg.kp, out);
    out << ", ";
  }

  // member: ki
  {
    out << "ki: ";
    rosidl_generator_traits::value_to_yaml(msg.ki, out);
    out << ", ";
  }

  // member: kd
  {
    out << "kd: ";
    rosidl_generator_traits::value_to_yaml(msg.kd, out);
    out << ", ";
  }

  // member: i_limit
  {
    out << "i_limit: ";
    rosidl_generator_traits::value_to_yaml(msg.i_limit, out);
    out << ", ";
  }

  // member: output_limit
  {
    out << "output_limit: ";
    rosidl_generator_traits::value_to_yaml(msg.output_limit, out);
    out << ", ";
  }

  // member: feedforward
  {
    out << "feedforward: ";
    rosidl_generator_traits::value_to_yaml(msg.feedforward, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const PidGains & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: name
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "name: ";
    rosidl_generator_traits::value_to_yaml(msg.name, out);
    out << "\n";
  }

  // member: kp
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "kp: ";
    rosidl_generator_traits::value_to_yaml(msg.kp, out);
    out << "\n";
  }

  // member: ki
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "ki: ";
    rosidl_generator_traits::value_to_yaml(msg.ki, out);
    out << "\n";
  }

  // member: kd
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "kd: ";
    rosidl_generator_traits::value_to_yaml(msg.kd, out);
    out << "\n";
  }

  // member: i_limit
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "i_limit: ";
    rosidl_generator_traits::value_to_yaml(msg.i_limit, out);
    out << "\n";
  }

  // member: output_limit
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "output_limit: ";
    rosidl_generator_traits::value_to_yaml(msg.output_limit, out);
    out << "\n";
  }

  // member: feedforward
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "feedforward: ";
    rosidl_generator_traits::value_to_yaml(msg.feedforward, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const PidGains & msg, bool use_flow_style = false)
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
  const uv_msgs::msg::PidGains & msg,
  std::ostream & out, size_t indentation = 0)
{
  uv_msgs::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use uv_msgs::msg::to_yaml() instead")]]
inline std::string to_yaml(const uv_msgs::msg::PidGains & msg)
{
  return uv_msgs::msg::to_yaml(msg);
}

template<>
inline const char * data_type<uv_msgs::msg::PidGains>()
{
  return "uv_msgs::msg::PidGains";
}

template<>
inline const char * name<uv_msgs::msg::PidGains>()
{
  return "uv_msgs/msg/PidGains";
}

template<>
struct has_fixed_size<uv_msgs::msg::PidGains>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<uv_msgs::msg::PidGains>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<uv_msgs::msg::PidGains>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // UV_MSGS__MSG__DETAIL__PID_GAINS__TRAITS_HPP_
