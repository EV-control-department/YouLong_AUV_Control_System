// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from uv_msgs:action/BasicMotion.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "uv_msgs/action/basic_motion.hpp"


#ifndef UV_MSGS__ACTION__DETAIL__BASIC_MOTION__TRAITS_HPP_
#define UV_MSGS__ACTION__DETAIL__BASIC_MOTION__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "uv_msgs/action/detail/basic_motion__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace uv_msgs
{

namespace action
{

inline void to_flow_style_yaml(
  const BasicMotion_Goal & msg,
  std::ostream & out)
{
  out << "{";
  // member: cmd_type
  {
    out << "cmd_type: ";
    rosidl_generator_traits::value_to_yaml(msg.cmd_type, out);
    out << ", ";
  }

  // member: axes
  {
    out << "axes: ";
    rosidl_generator_traits::value_to_yaml(msg.axes, out);
    out << ", ";
  }

  // member: target
  {
    if (msg.target.size() == 0) {
      out << "target: []";
    } else {
      out << "target: [";
      size_t pending_items = msg.target.size();
      for (auto item : msg.target) {
        rosidl_generator_traits::value_to_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
    out << ", ";
  }

  // member: timeout
  {
    out << "timeout: ";
    rosidl_generator_traits::value_to_yaml(msg.timeout, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const BasicMotion_Goal & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: cmd_type
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "cmd_type: ";
    rosidl_generator_traits::value_to_yaml(msg.cmd_type, out);
    out << "\n";
  }

  // member: axes
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "axes: ";
    rosidl_generator_traits::value_to_yaml(msg.axes, out);
    out << "\n";
  }

  // member: target
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.target.size() == 0) {
      out << "target: []\n";
    } else {
      out << "target:\n";
      for (auto item : msg.target) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "- ";
        rosidl_generator_traits::value_to_yaml(item, out);
        out << "\n";
      }
    }
  }

  // member: timeout
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "timeout: ";
    rosidl_generator_traits::value_to_yaml(msg.timeout, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const BasicMotion_Goal & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace action

}  // namespace uv_msgs

namespace rosidl_generator_traits
{

[[deprecated("use uv_msgs::action::to_block_style_yaml() instead")]]
inline void to_yaml(
  const uv_msgs::action::BasicMotion_Goal & msg,
  std::ostream & out, size_t indentation = 0)
{
  uv_msgs::action::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use uv_msgs::action::to_yaml() instead")]]
inline std::string to_yaml(const uv_msgs::action::BasicMotion_Goal & msg)
{
  return uv_msgs::action::to_yaml(msg);
}

template<>
inline const char * data_type<uv_msgs::action::BasicMotion_Goal>()
{
  return "uv_msgs::action::BasicMotion_Goal";
}

template<>
inline const char * name<uv_msgs::action::BasicMotion_Goal>()
{
  return "uv_msgs/action/BasicMotion_Goal";
}

template<>
struct has_fixed_size<uv_msgs::action::BasicMotion_Goal>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<uv_msgs::action::BasicMotion_Goal>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<uv_msgs::action::BasicMotion_Goal>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace uv_msgs
{

namespace action
{

inline void to_flow_style_yaml(
  const BasicMotion_Result & msg,
  std::ostream & out)
{
  out << "{";
  // member: success
  {
    out << "success: ";
    rosidl_generator_traits::value_to_yaml(msg.success, out);
    out << ", ";
  }

  // member: message
  {
    out << "message: ";
    rosidl_generator_traits::value_to_yaml(msg.message, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const BasicMotion_Result & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: success
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "success: ";
    rosidl_generator_traits::value_to_yaml(msg.success, out);
    out << "\n";
  }

  // member: message
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "message: ";
    rosidl_generator_traits::value_to_yaml(msg.message, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const BasicMotion_Result & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace action

}  // namespace uv_msgs

namespace rosidl_generator_traits
{

[[deprecated("use uv_msgs::action::to_block_style_yaml() instead")]]
inline void to_yaml(
  const uv_msgs::action::BasicMotion_Result & msg,
  std::ostream & out, size_t indentation = 0)
{
  uv_msgs::action::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use uv_msgs::action::to_yaml() instead")]]
inline std::string to_yaml(const uv_msgs::action::BasicMotion_Result & msg)
{
  return uv_msgs::action::to_yaml(msg);
}

template<>
inline const char * data_type<uv_msgs::action::BasicMotion_Result>()
{
  return "uv_msgs::action::BasicMotion_Result";
}

template<>
inline const char * name<uv_msgs::action::BasicMotion_Result>()
{
  return "uv_msgs/action/BasicMotion_Result";
}

template<>
struct has_fixed_size<uv_msgs::action::BasicMotion_Result>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<uv_msgs::action::BasicMotion_Result>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<uv_msgs::action::BasicMotion_Result>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace uv_msgs
{

namespace action
{

inline void to_flow_style_yaml(
  const BasicMotion_Feedback & msg,
  std::ostream & out)
{
  out << "{";
  // member: distance_remaining
  {
    out << "distance_remaining: ";
    rosidl_generator_traits::value_to_yaml(msg.distance_remaining, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const BasicMotion_Feedback & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: distance_remaining
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "distance_remaining: ";
    rosidl_generator_traits::value_to_yaml(msg.distance_remaining, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const BasicMotion_Feedback & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace action

}  // namespace uv_msgs

namespace rosidl_generator_traits
{

[[deprecated("use uv_msgs::action::to_block_style_yaml() instead")]]
inline void to_yaml(
  const uv_msgs::action::BasicMotion_Feedback & msg,
  std::ostream & out, size_t indentation = 0)
{
  uv_msgs::action::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use uv_msgs::action::to_yaml() instead")]]
inline std::string to_yaml(const uv_msgs::action::BasicMotion_Feedback & msg)
{
  return uv_msgs::action::to_yaml(msg);
}

template<>
inline const char * data_type<uv_msgs::action::BasicMotion_Feedback>()
{
  return "uv_msgs::action::BasicMotion_Feedback";
}

template<>
inline const char * name<uv_msgs::action::BasicMotion_Feedback>()
{
  return "uv_msgs/action/BasicMotion_Feedback";
}

template<>
struct has_fixed_size<uv_msgs::action::BasicMotion_Feedback>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<uv_msgs::action::BasicMotion_Feedback>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<uv_msgs::action::BasicMotion_Feedback>
  : std::true_type {};

}  // namespace rosidl_generator_traits

// Include directives for member types
// Member 'goal_id'
#include "unique_identifier_msgs/msg/detail/uuid__traits.hpp"
// Member 'goal'
#include "uv_msgs/action/detail/basic_motion__traits.hpp"

namespace uv_msgs
{

namespace action
{

inline void to_flow_style_yaml(
  const BasicMotion_SendGoal_Request & msg,
  std::ostream & out)
{
  out << "{";
  // member: goal_id
  {
    out << "goal_id: ";
    to_flow_style_yaml(msg.goal_id, out);
    out << ", ";
  }

  // member: goal
  {
    out << "goal: ";
    to_flow_style_yaml(msg.goal, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const BasicMotion_SendGoal_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: goal_id
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "goal_id:\n";
    to_block_style_yaml(msg.goal_id, out, indentation + 2);
  }

  // member: goal
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "goal:\n";
    to_block_style_yaml(msg.goal, out, indentation + 2);
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const BasicMotion_SendGoal_Request & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace action

}  // namespace uv_msgs

namespace rosidl_generator_traits
{

[[deprecated("use uv_msgs::action::to_block_style_yaml() instead")]]
inline void to_yaml(
  const uv_msgs::action::BasicMotion_SendGoal_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  uv_msgs::action::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use uv_msgs::action::to_yaml() instead")]]
inline std::string to_yaml(const uv_msgs::action::BasicMotion_SendGoal_Request & msg)
{
  return uv_msgs::action::to_yaml(msg);
}

template<>
inline const char * data_type<uv_msgs::action::BasicMotion_SendGoal_Request>()
{
  return "uv_msgs::action::BasicMotion_SendGoal_Request";
}

template<>
inline const char * name<uv_msgs::action::BasicMotion_SendGoal_Request>()
{
  return "uv_msgs/action/BasicMotion_SendGoal_Request";
}

template<>
struct has_fixed_size<uv_msgs::action::BasicMotion_SendGoal_Request>
  : std::integral_constant<bool, has_fixed_size<unique_identifier_msgs::msg::UUID>::value && has_fixed_size<uv_msgs::action::BasicMotion_Goal>::value> {};

template<>
struct has_bounded_size<uv_msgs::action::BasicMotion_SendGoal_Request>
  : std::integral_constant<bool, has_bounded_size<unique_identifier_msgs::msg::UUID>::value && has_bounded_size<uv_msgs::action::BasicMotion_Goal>::value> {};

template<>
struct is_message<uv_msgs::action::BasicMotion_SendGoal_Request>
  : std::true_type {};

}  // namespace rosidl_generator_traits

// Include directives for member types
// Member 'stamp'
#include "builtin_interfaces/msg/detail/time__traits.hpp"

namespace uv_msgs
{

namespace action
{

inline void to_flow_style_yaml(
  const BasicMotion_SendGoal_Response & msg,
  std::ostream & out)
{
  out << "{";
  // member: accepted
  {
    out << "accepted: ";
    rosidl_generator_traits::value_to_yaml(msg.accepted, out);
    out << ", ";
  }

  // member: stamp
  {
    out << "stamp: ";
    to_flow_style_yaml(msg.stamp, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const BasicMotion_SendGoal_Response & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: accepted
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "accepted: ";
    rosidl_generator_traits::value_to_yaml(msg.accepted, out);
    out << "\n";
  }

  // member: stamp
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "stamp:\n";
    to_block_style_yaml(msg.stamp, out, indentation + 2);
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const BasicMotion_SendGoal_Response & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace action

}  // namespace uv_msgs

namespace rosidl_generator_traits
{

[[deprecated("use uv_msgs::action::to_block_style_yaml() instead")]]
inline void to_yaml(
  const uv_msgs::action::BasicMotion_SendGoal_Response & msg,
  std::ostream & out, size_t indentation = 0)
{
  uv_msgs::action::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use uv_msgs::action::to_yaml() instead")]]
inline std::string to_yaml(const uv_msgs::action::BasicMotion_SendGoal_Response & msg)
{
  return uv_msgs::action::to_yaml(msg);
}

template<>
inline const char * data_type<uv_msgs::action::BasicMotion_SendGoal_Response>()
{
  return "uv_msgs::action::BasicMotion_SendGoal_Response";
}

template<>
inline const char * name<uv_msgs::action::BasicMotion_SendGoal_Response>()
{
  return "uv_msgs/action/BasicMotion_SendGoal_Response";
}

template<>
struct has_fixed_size<uv_msgs::action::BasicMotion_SendGoal_Response>
  : std::integral_constant<bool, has_fixed_size<builtin_interfaces::msg::Time>::value> {};

template<>
struct has_bounded_size<uv_msgs::action::BasicMotion_SendGoal_Response>
  : std::integral_constant<bool, has_bounded_size<builtin_interfaces::msg::Time>::value> {};

template<>
struct is_message<uv_msgs::action::BasicMotion_SendGoal_Response>
  : std::true_type {};

}  // namespace rosidl_generator_traits

// Include directives for member types
// Member 'info'
#include "service_msgs/msg/detail/service_event_info__traits.hpp"

namespace uv_msgs
{

namespace action
{

inline void to_flow_style_yaml(
  const BasicMotion_SendGoal_Event & msg,
  std::ostream & out)
{
  out << "{";
  // member: info
  {
    out << "info: ";
    to_flow_style_yaml(msg.info, out);
    out << ", ";
  }

  // member: request
  {
    if (msg.request.size() == 0) {
      out << "request: []";
    } else {
      out << "request: [";
      size_t pending_items = msg.request.size();
      for (auto item : msg.request) {
        to_flow_style_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
    out << ", ";
  }

  // member: response
  {
    if (msg.response.size() == 0) {
      out << "response: []";
    } else {
      out << "response: [";
      size_t pending_items = msg.response.size();
      for (auto item : msg.response) {
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
  const BasicMotion_SendGoal_Event & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: info
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "info:\n";
    to_block_style_yaml(msg.info, out, indentation + 2);
  }

  // member: request
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.request.size() == 0) {
      out << "request: []\n";
    } else {
      out << "request:\n";
      for (auto item : msg.request) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "-\n";
        to_block_style_yaml(item, out, indentation + 2);
      }
    }
  }

  // member: response
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.response.size() == 0) {
      out << "response: []\n";
    } else {
      out << "response:\n";
      for (auto item : msg.response) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "-\n";
        to_block_style_yaml(item, out, indentation + 2);
      }
    }
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const BasicMotion_SendGoal_Event & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace action

}  // namespace uv_msgs

namespace rosidl_generator_traits
{

[[deprecated("use uv_msgs::action::to_block_style_yaml() instead")]]
inline void to_yaml(
  const uv_msgs::action::BasicMotion_SendGoal_Event & msg,
  std::ostream & out, size_t indentation = 0)
{
  uv_msgs::action::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use uv_msgs::action::to_yaml() instead")]]
inline std::string to_yaml(const uv_msgs::action::BasicMotion_SendGoal_Event & msg)
{
  return uv_msgs::action::to_yaml(msg);
}

template<>
inline const char * data_type<uv_msgs::action::BasicMotion_SendGoal_Event>()
{
  return "uv_msgs::action::BasicMotion_SendGoal_Event";
}

template<>
inline const char * name<uv_msgs::action::BasicMotion_SendGoal_Event>()
{
  return "uv_msgs/action/BasicMotion_SendGoal_Event";
}

template<>
struct has_fixed_size<uv_msgs::action::BasicMotion_SendGoal_Event>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<uv_msgs::action::BasicMotion_SendGoal_Event>
  : std::integral_constant<bool, has_bounded_size<service_msgs::msg::ServiceEventInfo>::value && has_bounded_size<uv_msgs::action::BasicMotion_SendGoal_Request>::value && has_bounded_size<uv_msgs::action::BasicMotion_SendGoal_Response>::value> {};

template<>
struct is_message<uv_msgs::action::BasicMotion_SendGoal_Event>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<uv_msgs::action::BasicMotion_SendGoal>()
{
  return "uv_msgs::action::BasicMotion_SendGoal";
}

template<>
inline const char * name<uv_msgs::action::BasicMotion_SendGoal>()
{
  return "uv_msgs/action/BasicMotion_SendGoal";
}

template<>
struct has_fixed_size<uv_msgs::action::BasicMotion_SendGoal>
  : std::integral_constant<
    bool,
    has_fixed_size<uv_msgs::action::BasicMotion_SendGoal_Request>::value &&
    has_fixed_size<uv_msgs::action::BasicMotion_SendGoal_Response>::value
  >
{
};

template<>
struct has_bounded_size<uv_msgs::action::BasicMotion_SendGoal>
  : std::integral_constant<
    bool,
    has_bounded_size<uv_msgs::action::BasicMotion_SendGoal_Request>::value &&
    has_bounded_size<uv_msgs::action::BasicMotion_SendGoal_Response>::value
  >
{
};

template<>
struct is_service<uv_msgs::action::BasicMotion_SendGoal>
  : std::true_type
{
};

template<>
struct is_service_request<uv_msgs::action::BasicMotion_SendGoal_Request>
  : std::true_type
{
};

template<>
struct is_service_response<uv_msgs::action::BasicMotion_SendGoal_Response>
  : std::true_type
{
};

}  // namespace rosidl_generator_traits

// Include directives for member types
// Member 'goal_id'
// already included above
// #include "unique_identifier_msgs/msg/detail/uuid__traits.hpp"

namespace uv_msgs
{

namespace action
{

inline void to_flow_style_yaml(
  const BasicMotion_GetResult_Request & msg,
  std::ostream & out)
{
  out << "{";
  // member: goal_id
  {
    out << "goal_id: ";
    to_flow_style_yaml(msg.goal_id, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const BasicMotion_GetResult_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: goal_id
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "goal_id:\n";
    to_block_style_yaml(msg.goal_id, out, indentation + 2);
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const BasicMotion_GetResult_Request & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace action

}  // namespace uv_msgs

namespace rosidl_generator_traits
{

[[deprecated("use uv_msgs::action::to_block_style_yaml() instead")]]
inline void to_yaml(
  const uv_msgs::action::BasicMotion_GetResult_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  uv_msgs::action::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use uv_msgs::action::to_yaml() instead")]]
inline std::string to_yaml(const uv_msgs::action::BasicMotion_GetResult_Request & msg)
{
  return uv_msgs::action::to_yaml(msg);
}

template<>
inline const char * data_type<uv_msgs::action::BasicMotion_GetResult_Request>()
{
  return "uv_msgs::action::BasicMotion_GetResult_Request";
}

template<>
inline const char * name<uv_msgs::action::BasicMotion_GetResult_Request>()
{
  return "uv_msgs/action/BasicMotion_GetResult_Request";
}

template<>
struct has_fixed_size<uv_msgs::action::BasicMotion_GetResult_Request>
  : std::integral_constant<bool, has_fixed_size<unique_identifier_msgs::msg::UUID>::value> {};

template<>
struct has_bounded_size<uv_msgs::action::BasicMotion_GetResult_Request>
  : std::integral_constant<bool, has_bounded_size<unique_identifier_msgs::msg::UUID>::value> {};

template<>
struct is_message<uv_msgs::action::BasicMotion_GetResult_Request>
  : std::true_type {};

}  // namespace rosidl_generator_traits

// Include directives for member types
// Member 'result'
// already included above
// #include "uv_msgs/action/detail/basic_motion__traits.hpp"

namespace uv_msgs
{

namespace action
{

inline void to_flow_style_yaml(
  const BasicMotion_GetResult_Response & msg,
  std::ostream & out)
{
  out << "{";
  // member: status
  {
    out << "status: ";
    rosidl_generator_traits::value_to_yaml(msg.status, out);
    out << ", ";
  }

  // member: result
  {
    out << "result: ";
    to_flow_style_yaml(msg.result, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const BasicMotion_GetResult_Response & msg,
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

  // member: result
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "result:\n";
    to_block_style_yaml(msg.result, out, indentation + 2);
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const BasicMotion_GetResult_Response & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace action

}  // namespace uv_msgs

namespace rosidl_generator_traits
{

[[deprecated("use uv_msgs::action::to_block_style_yaml() instead")]]
inline void to_yaml(
  const uv_msgs::action::BasicMotion_GetResult_Response & msg,
  std::ostream & out, size_t indentation = 0)
{
  uv_msgs::action::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use uv_msgs::action::to_yaml() instead")]]
inline std::string to_yaml(const uv_msgs::action::BasicMotion_GetResult_Response & msg)
{
  return uv_msgs::action::to_yaml(msg);
}

template<>
inline const char * data_type<uv_msgs::action::BasicMotion_GetResult_Response>()
{
  return "uv_msgs::action::BasicMotion_GetResult_Response";
}

template<>
inline const char * name<uv_msgs::action::BasicMotion_GetResult_Response>()
{
  return "uv_msgs/action/BasicMotion_GetResult_Response";
}

template<>
struct has_fixed_size<uv_msgs::action::BasicMotion_GetResult_Response>
  : std::integral_constant<bool, has_fixed_size<uv_msgs::action::BasicMotion_Result>::value> {};

template<>
struct has_bounded_size<uv_msgs::action::BasicMotion_GetResult_Response>
  : std::integral_constant<bool, has_bounded_size<uv_msgs::action::BasicMotion_Result>::value> {};

template<>
struct is_message<uv_msgs::action::BasicMotion_GetResult_Response>
  : std::true_type {};

}  // namespace rosidl_generator_traits

// Include directives for member types
// Member 'info'
// already included above
// #include "service_msgs/msg/detail/service_event_info__traits.hpp"

namespace uv_msgs
{

namespace action
{

inline void to_flow_style_yaml(
  const BasicMotion_GetResult_Event & msg,
  std::ostream & out)
{
  out << "{";
  // member: info
  {
    out << "info: ";
    to_flow_style_yaml(msg.info, out);
    out << ", ";
  }

  // member: request
  {
    if (msg.request.size() == 0) {
      out << "request: []";
    } else {
      out << "request: [";
      size_t pending_items = msg.request.size();
      for (auto item : msg.request) {
        to_flow_style_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
    out << ", ";
  }

  // member: response
  {
    if (msg.response.size() == 0) {
      out << "response: []";
    } else {
      out << "response: [";
      size_t pending_items = msg.response.size();
      for (auto item : msg.response) {
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
  const BasicMotion_GetResult_Event & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: info
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "info:\n";
    to_block_style_yaml(msg.info, out, indentation + 2);
  }

  // member: request
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.request.size() == 0) {
      out << "request: []\n";
    } else {
      out << "request:\n";
      for (auto item : msg.request) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "-\n";
        to_block_style_yaml(item, out, indentation + 2);
      }
    }
  }

  // member: response
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.response.size() == 0) {
      out << "response: []\n";
    } else {
      out << "response:\n";
      for (auto item : msg.response) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "-\n";
        to_block_style_yaml(item, out, indentation + 2);
      }
    }
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const BasicMotion_GetResult_Event & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace action

}  // namespace uv_msgs

namespace rosidl_generator_traits
{

[[deprecated("use uv_msgs::action::to_block_style_yaml() instead")]]
inline void to_yaml(
  const uv_msgs::action::BasicMotion_GetResult_Event & msg,
  std::ostream & out, size_t indentation = 0)
{
  uv_msgs::action::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use uv_msgs::action::to_yaml() instead")]]
inline std::string to_yaml(const uv_msgs::action::BasicMotion_GetResult_Event & msg)
{
  return uv_msgs::action::to_yaml(msg);
}

template<>
inline const char * data_type<uv_msgs::action::BasicMotion_GetResult_Event>()
{
  return "uv_msgs::action::BasicMotion_GetResult_Event";
}

template<>
inline const char * name<uv_msgs::action::BasicMotion_GetResult_Event>()
{
  return "uv_msgs/action/BasicMotion_GetResult_Event";
}

template<>
struct has_fixed_size<uv_msgs::action::BasicMotion_GetResult_Event>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<uv_msgs::action::BasicMotion_GetResult_Event>
  : std::integral_constant<bool, has_bounded_size<service_msgs::msg::ServiceEventInfo>::value && has_bounded_size<uv_msgs::action::BasicMotion_GetResult_Request>::value && has_bounded_size<uv_msgs::action::BasicMotion_GetResult_Response>::value> {};

template<>
struct is_message<uv_msgs::action::BasicMotion_GetResult_Event>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<uv_msgs::action::BasicMotion_GetResult>()
{
  return "uv_msgs::action::BasicMotion_GetResult";
}

template<>
inline const char * name<uv_msgs::action::BasicMotion_GetResult>()
{
  return "uv_msgs/action/BasicMotion_GetResult";
}

template<>
struct has_fixed_size<uv_msgs::action::BasicMotion_GetResult>
  : std::integral_constant<
    bool,
    has_fixed_size<uv_msgs::action::BasicMotion_GetResult_Request>::value &&
    has_fixed_size<uv_msgs::action::BasicMotion_GetResult_Response>::value
  >
{
};

template<>
struct has_bounded_size<uv_msgs::action::BasicMotion_GetResult>
  : std::integral_constant<
    bool,
    has_bounded_size<uv_msgs::action::BasicMotion_GetResult_Request>::value &&
    has_bounded_size<uv_msgs::action::BasicMotion_GetResult_Response>::value
  >
{
};

template<>
struct is_service<uv_msgs::action::BasicMotion_GetResult>
  : std::true_type
{
};

template<>
struct is_service_request<uv_msgs::action::BasicMotion_GetResult_Request>
  : std::true_type
{
};

template<>
struct is_service_response<uv_msgs::action::BasicMotion_GetResult_Response>
  : std::true_type
{
};

}  // namespace rosidl_generator_traits

// Include directives for member types
// Member 'goal_id'
// already included above
// #include "unique_identifier_msgs/msg/detail/uuid__traits.hpp"
// Member 'feedback'
// already included above
// #include "uv_msgs/action/detail/basic_motion__traits.hpp"

namespace uv_msgs
{

namespace action
{

inline void to_flow_style_yaml(
  const BasicMotion_FeedbackMessage & msg,
  std::ostream & out)
{
  out << "{";
  // member: goal_id
  {
    out << "goal_id: ";
    to_flow_style_yaml(msg.goal_id, out);
    out << ", ";
  }

  // member: feedback
  {
    out << "feedback: ";
    to_flow_style_yaml(msg.feedback, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const BasicMotion_FeedbackMessage & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: goal_id
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "goal_id:\n";
    to_block_style_yaml(msg.goal_id, out, indentation + 2);
  }

  // member: feedback
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "feedback:\n";
    to_block_style_yaml(msg.feedback, out, indentation + 2);
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const BasicMotion_FeedbackMessage & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace action

}  // namespace uv_msgs

namespace rosidl_generator_traits
{

[[deprecated("use uv_msgs::action::to_block_style_yaml() instead")]]
inline void to_yaml(
  const uv_msgs::action::BasicMotion_FeedbackMessage & msg,
  std::ostream & out, size_t indentation = 0)
{
  uv_msgs::action::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use uv_msgs::action::to_yaml() instead")]]
inline std::string to_yaml(const uv_msgs::action::BasicMotion_FeedbackMessage & msg)
{
  return uv_msgs::action::to_yaml(msg);
}

template<>
inline const char * data_type<uv_msgs::action::BasicMotion_FeedbackMessage>()
{
  return "uv_msgs::action::BasicMotion_FeedbackMessage";
}

template<>
inline const char * name<uv_msgs::action::BasicMotion_FeedbackMessage>()
{
  return "uv_msgs/action/BasicMotion_FeedbackMessage";
}

template<>
struct has_fixed_size<uv_msgs::action::BasicMotion_FeedbackMessage>
  : std::integral_constant<bool, has_fixed_size<unique_identifier_msgs::msg::UUID>::value && has_fixed_size<uv_msgs::action::BasicMotion_Feedback>::value> {};

template<>
struct has_bounded_size<uv_msgs::action::BasicMotion_FeedbackMessage>
  : std::integral_constant<bool, has_bounded_size<unique_identifier_msgs::msg::UUID>::value && has_bounded_size<uv_msgs::action::BasicMotion_Feedback>::value> {};

template<>
struct is_message<uv_msgs::action::BasicMotion_FeedbackMessage>
  : std::true_type {};

}  // namespace rosidl_generator_traits


namespace rosidl_generator_traits
{

template<>
struct is_action<uv_msgs::action::BasicMotion>
  : std::true_type
{
};

template<>
struct is_action_goal<uv_msgs::action::BasicMotion_Goal>
  : std::true_type
{
};

template<>
struct is_action_result<uv_msgs::action::BasicMotion_Result>
  : std::true_type
{
};

template<>
struct is_action_feedback<uv_msgs::action::BasicMotion_Feedback>
  : std::true_type
{
};

}  // namespace rosidl_generator_traits


#endif  // UV_MSGS__ACTION__DETAIL__BASIC_MOTION__TRAITS_HPP_
