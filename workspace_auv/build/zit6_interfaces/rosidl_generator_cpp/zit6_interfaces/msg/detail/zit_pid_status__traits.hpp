// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from zit6_interfaces:msg/ZitPidStatus.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "zit6_interfaces/msg/zit_pid_status.hpp"


#ifndef ZIT6_INTERFACES__MSG__DETAIL__ZIT_PID_STATUS__TRAITS_HPP_
#define ZIT6_INTERFACES__MSG__DETAIL__ZIT_PID_STATUS__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "zit6_interfaces/msg/detail/zit_pid_status__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace zit6_interfaces
{

namespace msg
{

inline void to_flow_style_yaml(
  const ZitPidStatus & msg,
  std::ostream & out)
{
  out << "{";
  // member: pos_kp
  {
    if (msg.pos_kp.size() == 0) {
      out << "pos_kp: []";
    } else {
      out << "pos_kp: [";
      size_t pending_items = msg.pos_kp.size();
      for (auto item : msg.pos_kp) {
        rosidl_generator_traits::value_to_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
    out << ", ";
  }

  // member: pos_max_v
  {
    if (msg.pos_max_v.size() == 0) {
      out << "pos_max_v: []";
    } else {
      out << "pos_max_v: [";
      size_t pending_items = msg.pos_max_v.size();
      for (auto item : msg.pos_max_v) {
        rosidl_generator_traits::value_to_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
    out << ", ";
  }

  // member: pos_max_a
  {
    if (msg.pos_max_a.size() == 0) {
      out << "pos_max_a: []";
    } else {
      out << "pos_max_a: [";
      size_t pending_items = msg.pos_max_a.size();
      for (auto item : msg.pos_max_a) {
        rosidl_generator_traits::value_to_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
    out << ", ";
  }

  // member: pos_out_limit
  {
    if (msg.pos_out_limit.size() == 0) {
      out << "pos_out_limit: []";
    } else {
      out << "pos_out_limit: [";
      size_t pending_items = msg.pos_out_limit.size();
      for (auto item : msg.pos_out_limit) {
        rosidl_generator_traits::value_to_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
    out << ", ";
  }

  // member: vel_kp
  {
    if (msg.vel_kp.size() == 0) {
      out << "vel_kp: []";
    } else {
      out << "vel_kp: [";
      size_t pending_items = msg.vel_kp.size();
      for (auto item : msg.vel_kp) {
        rosidl_generator_traits::value_to_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
    out << ", ";
  }

  // member: vel_ki
  {
    if (msg.vel_ki.size() == 0) {
      out << "vel_ki: []";
    } else {
      out << "vel_ki: [";
      size_t pending_items = msg.vel_ki.size();
      for (auto item : msg.vel_ki) {
        rosidl_generator_traits::value_to_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
    out << ", ";
  }

  // member: vel_kd
  {
    if (msg.vel_kd.size() == 0) {
      out << "vel_kd: []";
    } else {
      out << "vel_kd: [";
      size_t pending_items = msg.vel_kd.size();
      for (auto item : msg.vel_kd) {
        rosidl_generator_traits::value_to_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
    out << ", ";
  }

  // member: vel_i_limit
  {
    if (msg.vel_i_limit.size() == 0) {
      out << "vel_i_limit: []";
    } else {
      out << "vel_i_limit: [";
      size_t pending_items = msg.vel_i_limit.size();
      for (auto item : msg.vel_i_limit) {
        rosidl_generator_traits::value_to_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
    out << ", ";
  }

  // member: vel_out_limit
  {
    if (msg.vel_out_limit.size() == 0) {
      out << "vel_out_limit: []";
    } else {
      out << "vel_out_limit: [";
      size_t pending_items = msg.vel_out_limit.size();
      for (auto item : msg.vel_out_limit) {
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
  const ZitPidStatus & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: pos_kp
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.pos_kp.size() == 0) {
      out << "pos_kp: []\n";
    } else {
      out << "pos_kp:\n";
      for (auto item : msg.pos_kp) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "- ";
        rosidl_generator_traits::value_to_yaml(item, out);
        out << "\n";
      }
    }
  }

  // member: pos_max_v
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.pos_max_v.size() == 0) {
      out << "pos_max_v: []\n";
    } else {
      out << "pos_max_v:\n";
      for (auto item : msg.pos_max_v) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "- ";
        rosidl_generator_traits::value_to_yaml(item, out);
        out << "\n";
      }
    }
  }

  // member: pos_max_a
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.pos_max_a.size() == 0) {
      out << "pos_max_a: []\n";
    } else {
      out << "pos_max_a:\n";
      for (auto item : msg.pos_max_a) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "- ";
        rosidl_generator_traits::value_to_yaml(item, out);
        out << "\n";
      }
    }
  }

  // member: pos_out_limit
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.pos_out_limit.size() == 0) {
      out << "pos_out_limit: []\n";
    } else {
      out << "pos_out_limit:\n";
      for (auto item : msg.pos_out_limit) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "- ";
        rosidl_generator_traits::value_to_yaml(item, out);
        out << "\n";
      }
    }
  }

  // member: vel_kp
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.vel_kp.size() == 0) {
      out << "vel_kp: []\n";
    } else {
      out << "vel_kp:\n";
      for (auto item : msg.vel_kp) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "- ";
        rosidl_generator_traits::value_to_yaml(item, out);
        out << "\n";
      }
    }
  }

  // member: vel_ki
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.vel_ki.size() == 0) {
      out << "vel_ki: []\n";
    } else {
      out << "vel_ki:\n";
      for (auto item : msg.vel_ki) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "- ";
        rosidl_generator_traits::value_to_yaml(item, out);
        out << "\n";
      }
    }
  }

  // member: vel_kd
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.vel_kd.size() == 0) {
      out << "vel_kd: []\n";
    } else {
      out << "vel_kd:\n";
      for (auto item : msg.vel_kd) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "- ";
        rosidl_generator_traits::value_to_yaml(item, out);
        out << "\n";
      }
    }
  }

  // member: vel_i_limit
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.vel_i_limit.size() == 0) {
      out << "vel_i_limit: []\n";
    } else {
      out << "vel_i_limit:\n";
      for (auto item : msg.vel_i_limit) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "- ";
        rosidl_generator_traits::value_to_yaml(item, out);
        out << "\n";
      }
    }
  }

  // member: vel_out_limit
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.vel_out_limit.size() == 0) {
      out << "vel_out_limit: []\n";
    } else {
      out << "vel_out_limit:\n";
      for (auto item : msg.vel_out_limit) {
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

inline std::string to_yaml(const ZitPidStatus & msg, bool use_flow_style = false)
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
  const zit6_interfaces::msg::ZitPidStatus & msg,
  std::ostream & out, size_t indentation = 0)
{
  zit6_interfaces::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use zit6_interfaces::msg::to_yaml() instead")]]
inline std::string to_yaml(const zit6_interfaces::msg::ZitPidStatus & msg)
{
  return zit6_interfaces::msg::to_yaml(msg);
}

template<>
inline const char * data_type<zit6_interfaces::msg::ZitPidStatus>()
{
  return "zit6_interfaces::msg::ZitPidStatus";
}

template<>
inline const char * name<zit6_interfaces::msg::ZitPidStatus>()
{
  return "zit6_interfaces/msg/ZitPidStatus";
}

template<>
struct has_fixed_size<zit6_interfaces::msg::ZitPidStatus>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<zit6_interfaces::msg::ZitPidStatus>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<zit6_interfaces::msg::ZitPidStatus>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // ZIT6_INTERFACES__MSG__DETAIL__ZIT_PID_STATUS__TRAITS_HPP_
