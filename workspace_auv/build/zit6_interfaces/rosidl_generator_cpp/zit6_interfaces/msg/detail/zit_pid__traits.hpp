// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from zit6_interfaces:msg/ZitPid.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "zit6_interfaces/msg/zit_pid.hpp"


#ifndef ZIT6_INTERFACES__MSG__DETAIL__ZIT_PID__TRAITS_HPP_
#define ZIT6_INTERFACES__MSG__DETAIL__ZIT_PID__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "zit6_interfaces/msg/detail/zit_pid__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace zit6_interfaces
{

namespace msg
{

inline void to_flow_style_yaml(
  const ZitPid & msg,
  std::ostream & out)
{
  out << "{";
  // member: axis
  {
    out << "axis: ";
    rosidl_generator_traits::value_to_yaml(msg.axis, out);
    out << ", ";
  }

  // member: is_pos_ring
  {
    out << "is_pos_ring: ";
    rosidl_generator_traits::value_to_yaml(msg.is_pos_ring, out);
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

  // member: out_limit
  {
    out << "out_limit: ";
    rosidl_generator_traits::value_to_yaml(msg.out_limit, out);
    out << ", ";
  }

  // member: max_v
  {
    out << "max_v: ";
    rosidl_generator_traits::value_to_yaml(msg.max_v, out);
    out << ", ";
  }

  // member: max_a
  {
    out << "max_a: ";
    rosidl_generator_traits::value_to_yaml(msg.max_a, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const ZitPid & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: axis
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "axis: ";
    rosidl_generator_traits::value_to_yaml(msg.axis, out);
    out << "\n";
  }

  // member: is_pos_ring
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "is_pos_ring: ";
    rosidl_generator_traits::value_to_yaml(msg.is_pos_ring, out);
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

  // member: out_limit
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "out_limit: ";
    rosidl_generator_traits::value_to_yaml(msg.out_limit, out);
    out << "\n";
  }

  // member: max_v
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "max_v: ";
    rosidl_generator_traits::value_to_yaml(msg.max_v, out);
    out << "\n";
  }

  // member: max_a
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "max_a: ";
    rosidl_generator_traits::value_to_yaml(msg.max_a, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const ZitPid & msg, bool use_flow_style = false)
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

}  // namespace zit6_interfaces

namespace rosidl_generator_traits
{

[[deprecated("use zit6_interfaces::msg::to_block_style_yaml() instead")]]
inline void to_yaml(
  const zit6_interfaces::msg::ZitPid & msg,
  std::ostream & out, size_t indentation = 0)
{
  zit6_interfaces::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use zit6_interfaces::msg::to_yaml() instead")]]
inline std::string to_yaml(const zit6_interfaces::msg::ZitPid & msg)
{
  return zit6_interfaces::msg::to_yaml(msg);
}

template<>
inline const char * data_type<zit6_interfaces::msg::ZitPid>()
{
  return "zit6_interfaces::msg::ZitPid";
}

template<>
inline const char * name<zit6_interfaces::msg::ZitPid>()
{
  return "zit6_interfaces/msg/ZitPid";
}

template<>
struct has_fixed_size<zit6_interfaces::msg::ZitPid>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<zit6_interfaces::msg::ZitPid>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<zit6_interfaces::msg::ZitPid>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // ZIT6_INTERFACES__MSG__DETAIL__ZIT_PID__TRAITS_HPP_
