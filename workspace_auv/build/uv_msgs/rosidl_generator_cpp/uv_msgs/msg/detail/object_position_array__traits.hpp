// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from uv_msgs:msg/ObjectPositionArray.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "uv_msgs/msg/object_position_array.hpp"


#ifndef UV_MSGS__MSG__DETAIL__OBJECT_POSITION_ARRAY__TRAITS_HPP_
#define UV_MSGS__MSG__DETAIL__OBJECT_POSITION_ARRAY__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "uv_msgs/msg/detail/object_position_array__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__traits.hpp"
// Member 'objects'
#include "uv_msgs/msg/detail/object_position__traits.hpp"

namespace uv_msgs
{

namespace msg
{

inline void to_flow_style_yaml(
  const ObjectPositionArray & msg,
  std::ostream & out)
{
  out << "{";
  // member: header
  {
    out << "header: ";
    to_flow_style_yaml(msg.header, out);
    out << ", ";
  }

  // member: objects
  {
    if (msg.objects.size() == 0) {
      out << "objects: []";
    } else {
      out << "objects: [";
      size_t pending_items = msg.objects.size();
      for (auto item : msg.objects) {
        to_flow_style_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const ObjectPositionArray & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: header
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "header:\n";
    to_block_style_yaml(msg.header, out, indentation + 2);
  }

  // member: objects
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.objects.size() == 0) {
      out << "objects: []\n";
    } else {
      out << "objects:\n";
      for (auto item : msg.objects) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "-\n";
        to_block_style_yaml(item, out, indentation + 2);
      }
    }
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const ObjectPositionArray & msg, bool use_flow_style = false)
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
  const uv_msgs::msg::ObjectPositionArray & msg,
  std::ostream & out, size_t indentation = 0)
{
  uv_msgs::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use uv_msgs::msg::to_yaml() instead")]]
inline std::string to_yaml(const uv_msgs::msg::ObjectPositionArray & msg)
{
  return uv_msgs::msg::to_yaml(msg);
}

template<>
inline const char * data_type<uv_msgs::msg::ObjectPositionArray>()
{
  return "uv_msgs::msg::ObjectPositionArray";
}

template<>
inline const char * name<uv_msgs::msg::ObjectPositionArray>()
{
  return "uv_msgs/msg/ObjectPositionArray";
}

template<>
struct has_fixed_size<uv_msgs::msg::ObjectPositionArray>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<uv_msgs::msg::ObjectPositionArray>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<uv_msgs::msg::ObjectPositionArray>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // UV_MSGS__MSG__DETAIL__OBJECT_POSITION_ARRAY__TRAITS_HPP_
