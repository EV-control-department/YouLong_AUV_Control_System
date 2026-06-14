// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from uv_msgs:msg/Detection.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "uv_msgs/msg/detection.hpp"


#ifndef UV_MSGS__MSG__DETAIL__DETECTION__TRAITS_HPP_
#define UV_MSGS__MSG__DETAIL__DETECTION__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "uv_msgs/msg/detail/detection__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace uv_msgs
{

namespace msg
{

inline void to_flow_style_yaml(
  const Detection & msg,
  std::ostream & out)
{
  out << "{";
  // member: class_id
  {
    out << "class_id: ";
    rosidl_generator_traits::value_to_yaml(msg.class_id, out);
    out << ", ";
  }

  // member: confidence
  {
    out << "confidence: ";
    rosidl_generator_traits::value_to_yaml(msg.confidence, out);
    out << ", ";
  }

  // member: pixel_x
  {
    out << "pixel_x: ";
    rosidl_generator_traits::value_to_yaml(msg.pixel_x, out);
    out << ", ";
  }

  // member: pixel_y
  {
    out << "pixel_y: ";
    rosidl_generator_traits::value_to_yaml(msg.pixel_y, out);
    out << ", ";
  }

  // member: bbox_x1
  {
    out << "bbox_x1: ";
    rosidl_generator_traits::value_to_yaml(msg.bbox_x1, out);
    out << ", ";
  }

  // member: bbox_y1
  {
    out << "bbox_y1: ";
    rosidl_generator_traits::value_to_yaml(msg.bbox_y1, out);
    out << ", ";
  }

  // member: bbox_x2
  {
    out << "bbox_x2: ";
    rosidl_generator_traits::value_to_yaml(msg.bbox_x2, out);
    out << ", ";
  }

  // member: bbox_y2
  {
    out << "bbox_y2: ";
    rosidl_generator_traits::value_to_yaml(msg.bbox_y2, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const Detection & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: class_id
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "class_id: ";
    rosidl_generator_traits::value_to_yaml(msg.class_id, out);
    out << "\n";
  }

  // member: confidence
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "confidence: ";
    rosidl_generator_traits::value_to_yaml(msg.confidence, out);
    out << "\n";
  }

  // member: pixel_x
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "pixel_x: ";
    rosidl_generator_traits::value_to_yaml(msg.pixel_x, out);
    out << "\n";
  }

  // member: pixel_y
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "pixel_y: ";
    rosidl_generator_traits::value_to_yaml(msg.pixel_y, out);
    out << "\n";
  }

  // member: bbox_x1
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "bbox_x1: ";
    rosidl_generator_traits::value_to_yaml(msg.bbox_x1, out);
    out << "\n";
  }

  // member: bbox_y1
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "bbox_y1: ";
    rosidl_generator_traits::value_to_yaml(msg.bbox_y1, out);
    out << "\n";
  }

  // member: bbox_x2
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "bbox_x2: ";
    rosidl_generator_traits::value_to_yaml(msg.bbox_x2, out);
    out << "\n";
  }

  // member: bbox_y2
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "bbox_y2: ";
    rosidl_generator_traits::value_to_yaml(msg.bbox_y2, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const Detection & msg, bool use_flow_style = false)
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
  const uv_msgs::msg::Detection & msg,
  std::ostream & out, size_t indentation = 0)
{
  uv_msgs::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use uv_msgs::msg::to_yaml() instead")]]
inline std::string to_yaml(const uv_msgs::msg::Detection & msg)
{
  return uv_msgs::msg::to_yaml(msg);
}

template<>
inline const char * data_type<uv_msgs::msg::Detection>()
{
  return "uv_msgs::msg::Detection";
}

template<>
inline const char * name<uv_msgs::msg::Detection>()
{
  return "uv_msgs/msg/Detection";
}

template<>
struct has_fixed_size<uv_msgs::msg::Detection>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<uv_msgs::msg::Detection>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<uv_msgs::msg::Detection>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // UV_MSGS__MSG__DETAIL__DETECTION__TRAITS_HPP_
