// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from zit6_interfaces:srv/UpdateParams.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "zit6_interfaces/srv/update_params.hpp"


#ifndef ZIT6_INTERFACES__SRV__DETAIL__UPDATE_PARAMS__TRAITS_HPP_
#define ZIT6_INTERFACES__SRV__DETAIL__UPDATE_PARAMS__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "zit6_interfaces/srv/detail/update_params__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace zit6_interfaces
{

namespace srv
{

inline void to_flow_style_yaml(
  const UpdateParams_Request & msg,
  std::ostream & out)
{
  out << "{";
  // member: json
  {
    out << "json: ";
    rosidl_generator_traits::value_to_yaml(msg.json, out);
    out << ", ";
  }

  // member: paths
  {
    if (msg.paths.size() == 0) {
      out << "paths: []";
    } else {
      out << "paths: [";
      size_t pending_items = msg.paths.size();
      for (auto item : msg.paths) {
        rosidl_generator_traits::value_to_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
    out << ", ";
  }

  // member: values
  {
    if (msg.values.size() == 0) {
      out << "values: []";
    } else {
      out << "values: [";
      size_t pending_items = msg.values.size();
      for (auto item : msg.values) {
        rosidl_generator_traits::value_to_yaml(item, out);
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
  const UpdateParams_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: json
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "json: ";
    rosidl_generator_traits::value_to_yaml(msg.json, out);
    out << "\n";
  }

  // member: paths
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.paths.size() == 0) {
      out << "paths: []\n";
    } else {
      out << "paths:\n";
      for (auto item : msg.paths) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "- ";
        rosidl_generator_traits::value_to_yaml(item, out);
        out << "\n";
      }
    }
  }

  // member: values
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.values.size() == 0) {
      out << "values: []\n";
    } else {
      out << "values:\n";
      for (auto item : msg.values) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "- ";
        rosidl_generator_traits::value_to_yaml(item, out);
        out << "\n";
      }
    }
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const UpdateParams_Request & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace srv

}  // namespace zit6_interfaces

namespace rosidl_generator_traits
{

[[deprecated("use zit6_interfaces::srv::to_block_style_yaml() instead")]]
inline void to_yaml(
  const zit6_interfaces::srv::UpdateParams_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  zit6_interfaces::srv::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use zit6_interfaces::srv::to_yaml() instead")]]
inline std::string to_yaml(const zit6_interfaces::srv::UpdateParams_Request & msg)
{
  return zit6_interfaces::srv::to_yaml(msg);
}

template<>
inline const char * data_type<zit6_interfaces::srv::UpdateParams_Request>()
{
  return "zit6_interfaces::srv::UpdateParams_Request";
}

template<>
inline const char * name<zit6_interfaces::srv::UpdateParams_Request>()
{
  return "zit6_interfaces/srv/UpdateParams_Request";
}

template<>
struct has_fixed_size<zit6_interfaces::srv::UpdateParams_Request>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<zit6_interfaces::srv::UpdateParams_Request>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<zit6_interfaces::srv::UpdateParams_Request>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace zit6_interfaces
{

namespace srv
{

inline void to_flow_style_yaml(
  const UpdateParams_Response & msg,
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
  const UpdateParams_Response & msg,
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

inline std::string to_yaml(const UpdateParams_Response & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace srv

}  // namespace zit6_interfaces

namespace rosidl_generator_traits
{

[[deprecated("use zit6_interfaces::srv::to_block_style_yaml() instead")]]
inline void to_yaml(
  const zit6_interfaces::srv::UpdateParams_Response & msg,
  std::ostream & out, size_t indentation = 0)
{
  zit6_interfaces::srv::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use zit6_interfaces::srv::to_yaml() instead")]]
inline std::string to_yaml(const zit6_interfaces::srv::UpdateParams_Response & msg)
{
  return zit6_interfaces::srv::to_yaml(msg);
}

template<>
inline const char * data_type<zit6_interfaces::srv::UpdateParams_Response>()
{
  return "zit6_interfaces::srv::UpdateParams_Response";
}

template<>
inline const char * name<zit6_interfaces::srv::UpdateParams_Response>()
{
  return "zit6_interfaces/srv/UpdateParams_Response";
}

template<>
struct has_fixed_size<zit6_interfaces::srv::UpdateParams_Response>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<zit6_interfaces::srv::UpdateParams_Response>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<zit6_interfaces::srv::UpdateParams_Response>
  : std::true_type {};

}  // namespace rosidl_generator_traits

// Include directives for member types
// Member 'info'
#include "service_msgs/msg/detail/service_event_info__traits.hpp"

namespace zit6_interfaces
{

namespace srv
{

inline void to_flow_style_yaml(
  const UpdateParams_Event & msg,
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
  const UpdateParams_Event & msg,
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

inline std::string to_yaml(const UpdateParams_Event & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace srv

}  // namespace zit6_interfaces

namespace rosidl_generator_traits
{

[[deprecated("use zit6_interfaces::srv::to_block_style_yaml() instead")]]
inline void to_yaml(
  const zit6_interfaces::srv::UpdateParams_Event & msg,
  std::ostream & out, size_t indentation = 0)
{
  zit6_interfaces::srv::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use zit6_interfaces::srv::to_yaml() instead")]]
inline std::string to_yaml(const zit6_interfaces::srv::UpdateParams_Event & msg)
{
  return zit6_interfaces::srv::to_yaml(msg);
}

template<>
inline const char * data_type<zit6_interfaces::srv::UpdateParams_Event>()
{
  return "zit6_interfaces::srv::UpdateParams_Event";
}

template<>
inline const char * name<zit6_interfaces::srv::UpdateParams_Event>()
{
  return "zit6_interfaces/srv/UpdateParams_Event";
}

template<>
struct has_fixed_size<zit6_interfaces::srv::UpdateParams_Event>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<zit6_interfaces::srv::UpdateParams_Event>
  : std::integral_constant<bool, has_bounded_size<service_msgs::msg::ServiceEventInfo>::value && has_bounded_size<zit6_interfaces::srv::UpdateParams_Request>::value && has_bounded_size<zit6_interfaces::srv::UpdateParams_Response>::value> {};

template<>
struct is_message<zit6_interfaces::srv::UpdateParams_Event>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<zit6_interfaces::srv::UpdateParams>()
{
  return "zit6_interfaces::srv::UpdateParams";
}

template<>
inline const char * name<zit6_interfaces::srv::UpdateParams>()
{
  return "zit6_interfaces/srv/UpdateParams";
}

template<>
struct has_fixed_size<zit6_interfaces::srv::UpdateParams>
  : std::integral_constant<
    bool,
    has_fixed_size<zit6_interfaces::srv::UpdateParams_Request>::value &&
    has_fixed_size<zit6_interfaces::srv::UpdateParams_Response>::value
  >
{
};

template<>
struct has_bounded_size<zit6_interfaces::srv::UpdateParams>
  : std::integral_constant<
    bool,
    has_bounded_size<zit6_interfaces::srv::UpdateParams_Request>::value &&
    has_bounded_size<zit6_interfaces::srv::UpdateParams_Response>::value
  >
{
};

template<>
struct is_service<zit6_interfaces::srv::UpdateParams>
  : std::true_type
{
};

template<>
struct is_service_request<zit6_interfaces::srv::UpdateParams_Request>
  : std::true_type
{
};

template<>
struct is_service_response<zit6_interfaces::srv::UpdateParams_Response>
  : std::true_type
{
};

}  // namespace rosidl_generator_traits

#endif  // ZIT6_INTERFACES__SRV__DETAIL__UPDATE_PARAMS__TRAITS_HPP_
