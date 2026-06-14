// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from uv_msgs:msg/PidState.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "uv_msgs/msg/pid_state.hpp"


#ifndef UV_MSGS__MSG__DETAIL__PID_STATE__STRUCT_HPP_
#define UV_MSGS__MSG__DETAIL__PID_STATE__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


// Include directives for member types
// Member 'speed_mag'
// Member 'vel_x'
// Member 'vel_y'
// Member 'pos_z'
// Member 'yaw'
// Member 'yaw_rate'
#include "uv_msgs/msg/detail/pid_gains__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__uv_msgs__msg__PidState __attribute__((deprecated))
#else
# define DEPRECATED__uv_msgs__msg__PidState __declspec(deprecated)
#endif

namespace uv_msgs
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct PidState_
{
  using Type = PidState_<ContainerAllocator>;

  explicit PidState_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : speed_mag(_init),
    vel_x(_init),
    vel_y(_init),
    pos_z(_init),
    yaw(_init),
    yaw_rate(_init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->error_length = 0.0f;
      this->error_angle_deg = 0.0f;
      this->speed_cmd = 0.0f;
      this->vx_body_cmd = 0.0f;
      this->vy_body_cmd = 0.0f;
    }
  }

  explicit PidState_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : speed_mag(_alloc, _init),
    vel_x(_alloc, _init),
    vel_y(_alloc, _init),
    pos_z(_alloc, _init),
    yaw(_alloc, _init),
    yaw_rate(_alloc, _init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->error_length = 0.0f;
      this->error_angle_deg = 0.0f;
      this->speed_cmd = 0.0f;
      this->vx_body_cmd = 0.0f;
      this->vy_body_cmd = 0.0f;
    }
  }

  // field types and members
  using _speed_mag_type =
    uv_msgs::msg::PidGains_<ContainerAllocator>;
  _speed_mag_type speed_mag;
  using _vel_x_type =
    uv_msgs::msg::PidGains_<ContainerAllocator>;
  _vel_x_type vel_x;
  using _vel_y_type =
    uv_msgs::msg::PidGains_<ContainerAllocator>;
  _vel_y_type vel_y;
  using _pos_z_type =
    uv_msgs::msg::PidGains_<ContainerAllocator>;
  _pos_z_type pos_z;
  using _yaw_type =
    uv_msgs::msg::PidGains_<ContainerAllocator>;
  _yaw_type yaw;
  using _yaw_rate_type =
    uv_msgs::msg::PidGains_<ContainerAllocator>;
  _yaw_rate_type yaw_rate;
  using _error_length_type =
    float;
  _error_length_type error_length;
  using _error_angle_deg_type =
    float;
  _error_angle_deg_type error_angle_deg;
  using _speed_cmd_type =
    float;
  _speed_cmd_type speed_cmd;
  using _vx_body_cmd_type =
    float;
  _vx_body_cmd_type vx_body_cmd;
  using _vy_body_cmd_type =
    float;
  _vy_body_cmd_type vy_body_cmd;

  // setters for named parameter idiom
  Type & set__speed_mag(
    const uv_msgs::msg::PidGains_<ContainerAllocator> & _arg)
  {
    this->speed_mag = _arg;
    return *this;
  }
  Type & set__vel_x(
    const uv_msgs::msg::PidGains_<ContainerAllocator> & _arg)
  {
    this->vel_x = _arg;
    return *this;
  }
  Type & set__vel_y(
    const uv_msgs::msg::PidGains_<ContainerAllocator> & _arg)
  {
    this->vel_y = _arg;
    return *this;
  }
  Type & set__pos_z(
    const uv_msgs::msg::PidGains_<ContainerAllocator> & _arg)
  {
    this->pos_z = _arg;
    return *this;
  }
  Type & set__yaw(
    const uv_msgs::msg::PidGains_<ContainerAllocator> & _arg)
  {
    this->yaw = _arg;
    return *this;
  }
  Type & set__yaw_rate(
    const uv_msgs::msg::PidGains_<ContainerAllocator> & _arg)
  {
    this->yaw_rate = _arg;
    return *this;
  }
  Type & set__error_length(
    const float & _arg)
  {
    this->error_length = _arg;
    return *this;
  }
  Type & set__error_angle_deg(
    const float & _arg)
  {
    this->error_angle_deg = _arg;
    return *this;
  }
  Type & set__speed_cmd(
    const float & _arg)
  {
    this->speed_cmd = _arg;
    return *this;
  }
  Type & set__vx_body_cmd(
    const float & _arg)
  {
    this->vx_body_cmd = _arg;
    return *this;
  }
  Type & set__vy_body_cmd(
    const float & _arg)
  {
    this->vy_body_cmd = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    uv_msgs::msg::PidState_<ContainerAllocator> *;
  using ConstRawPtr =
    const uv_msgs::msg::PidState_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<uv_msgs::msg::PidState_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<uv_msgs::msg::PidState_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      uv_msgs::msg::PidState_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<uv_msgs::msg::PidState_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      uv_msgs::msg::PidState_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<uv_msgs::msg::PidState_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<uv_msgs::msg::PidState_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<uv_msgs::msg::PidState_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__uv_msgs__msg__PidState
    std::shared_ptr<uv_msgs::msg::PidState_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__uv_msgs__msg__PidState
    std::shared_ptr<uv_msgs::msg::PidState_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const PidState_ & other) const
  {
    if (this->speed_mag != other.speed_mag) {
      return false;
    }
    if (this->vel_x != other.vel_x) {
      return false;
    }
    if (this->vel_y != other.vel_y) {
      return false;
    }
    if (this->pos_z != other.pos_z) {
      return false;
    }
    if (this->yaw != other.yaw) {
      return false;
    }
    if (this->yaw_rate != other.yaw_rate) {
      return false;
    }
    if (this->error_length != other.error_length) {
      return false;
    }
    if (this->error_angle_deg != other.error_angle_deg) {
      return false;
    }
    if (this->speed_cmd != other.speed_cmd) {
      return false;
    }
    if (this->vx_body_cmd != other.vx_body_cmd) {
      return false;
    }
    if (this->vy_body_cmd != other.vy_body_cmd) {
      return false;
    }
    return true;
  }
  bool operator!=(const PidState_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct PidState_

// alias to use template instance with default allocator
using PidState =
  uv_msgs::msg::PidState_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace uv_msgs

#endif  // UV_MSGS__MSG__DETAIL__PID_STATE__STRUCT_HPP_
