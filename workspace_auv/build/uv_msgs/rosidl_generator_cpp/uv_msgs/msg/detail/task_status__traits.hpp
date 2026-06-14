// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from uv_msgs:msg/TaskStatus.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "uv_msgs/msg/task_status.hpp"


#ifndef UV_MSGS__MSG__DETAIL__TASK_STATUS__TRAITS_HPP_
#define UV_MSGS__MSG__DETAIL__TASK_STATUS__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "uv_msgs/msg/detail/task_status__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace uv_msgs
{

namespace msg
{

inline void to_flow_style_yaml(
  const TaskStatus & msg,
  std::ostream & out)
{
  out << "{";
  // member: status
  {
    out << "status: ";
    rosidl_generator_traits::value_to_yaml(msg.status, out);
    out << ", ";
  }

  // member: current_task_index
  {
    out << "current_task_index: ";
    rosidl_generator_traits::value_to_yaml(msg.current_task_index, out);
    out << ", ";
  }

  // member: total_tasks
  {
    out << "total_tasks: ";
    rosidl_generator_traits::value_to_yaml(msg.total_tasks, out);
    out << ", ";
  }

  // member: current_task_name
  {
    out << "current_task_name: ";
    rosidl_generator_traits::value_to_yaml(msg.current_task_name, out);
    out << ", ";
  }

  // member: error_message
  {
    out << "error_message: ";
    rosidl_generator_traits::value_to_yaml(msg.error_message, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const TaskStatus & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: status
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "status: ";
    rosidl_generator_traits::value_to_yaml(msg.status, out);
    out << "\n";
  }

  // member: current_task_index
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "current_task_index: ";
    rosidl_generator_traits::value_to_yaml(msg.current_task_index, out);
    out << "\n";
  }

  // member: total_tasks
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "total_tasks: ";
    rosidl_generator_traits::value_to_yaml(msg.total_tasks, out);
    out << "\n";
  }

  // member: current_task_name
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "current_task_name: ";
    rosidl_generator_traits::value_to_yaml(msg.current_task_name, out);
    out << "\n";
  }

  // member: error_message
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "error_message: ";
    rosidl_generator_traits::value_to_yaml(msg.error_message, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const TaskStatus & msg, bool use_flow_style = false)
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
  const uv_msgs::msg::TaskStatus & msg,
  std::ostream & out, size_t indentation = 0)
{
  uv_msgs::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use uv_msgs::msg::to_yaml() instead")]]
inline std::string to_yaml(const uv_msgs::msg::TaskStatus & msg)
{
  return uv_msgs::msg::to_yaml(msg);
}

template<>
inline const char * data_type<uv_msgs::msg::TaskStatus>()
{
  return "uv_msgs::msg::TaskStatus";
}

template<>
inline const char * name<uv_msgs::msg::TaskStatus>()
{
  return "uv_msgs/msg/TaskStatus";
}

template<>
struct has_fixed_size<uv_msgs::msg::TaskStatus>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<uv_msgs::msg::TaskStatus>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<uv_msgs::msg::TaskStatus>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // UV_MSGS__MSG__DETAIL__TASK_STATUS__TRAITS_HPP_
