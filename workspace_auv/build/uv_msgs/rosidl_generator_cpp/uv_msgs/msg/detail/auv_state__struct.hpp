// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from uv_msgs:msg/AuvState.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "uv_msgs/msg/auv_state.hpp"


#ifndef UV_MSGS__MSG__DETAIL__AUV_STATE__STRUCT_HPP_
#define UV_MSGS__MSG__DETAIL__AUV_STATE__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__uv_msgs__msg__AuvState __attribute__((deprecated))
#else
# define DEPRECATED__uv_msgs__msg__AuvState __declspec(deprecated)
#endif

namespace uv_msgs
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct AuvState_
{
  using Type = AuvState_<ContainerAllocator>;

  explicit AuvState_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->pos_x = 0.0f;
      this->pos_y = 0.0f;
      this->pos_z = 0.0f;
      this->yaw = 0.0f;
      this->vel_x = 0.0f;
      this->vel_y = 0.0f;
      this->vel_z = 0.0f;
      this->yaw_rate = 0.0f;
      this->vel_world_x = 0.0f;
      this->vel_world_y = 0.0f;
      this->armed = false;
      this->control_mode = 0;
      this->battery_voltage = 0.0f;
      this->error_flags = 0ul;
      this->cycle_time_ms = 0.0f;
      this->target_x = 0.0f;
      this->target_y = 0.0f;
      this->target_z = 0.0f;
      this->target_yaw = 0.0f;
    }
  }

  explicit AuvState_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_alloc;
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->pos_x = 0.0f;
      this->pos_y = 0.0f;
      this->pos_z = 0.0f;
      this->yaw = 0.0f;
      this->vel_x = 0.0f;
      this->vel_y = 0.0f;
      this->vel_z = 0.0f;
      this->yaw_rate = 0.0f;
      this->vel_world_x = 0.0f;
      this->vel_world_y = 0.0f;
      this->armed = false;
      this->control_mode = 0;
      this->battery_voltage = 0.0f;
      this->error_flags = 0ul;
      this->cycle_time_ms = 0.0f;
      this->target_x = 0.0f;
      this->target_y = 0.0f;
      this->target_z = 0.0f;
      this->target_yaw = 0.0f;
    }
  }

  // field types and members
  using _pos_x_type =
    float;
  _pos_x_type pos_x;
  using _pos_y_type =
    float;
  _pos_y_type pos_y;
  using _pos_z_type =
    float;
  _pos_z_type pos_z;
  using _yaw_type =
    float;
  _yaw_type yaw;
  using _vel_x_type =
    float;
  _vel_x_type vel_x;
  using _vel_y_type =
    float;
  _vel_y_type vel_y;
  using _vel_z_type =
    float;
  _vel_z_type vel_z;
  using _yaw_rate_type =
    float;
  _yaw_rate_type yaw_rate;
  using _vel_world_x_type =
    float;
  _vel_world_x_type vel_world_x;
  using _vel_world_y_type =
    float;
  _vel_world_y_type vel_world_y;
  using _armed_type =
    bool;
  _armed_type armed;
  using _control_mode_type =
    uint8_t;
  _control_mode_type control_mode;
  using _battery_voltage_type =
    float;
  _battery_voltage_type battery_voltage;
  using _error_flags_type =
    uint32_t;
  _error_flags_type error_flags;
  using _cycle_time_ms_type =
    float;
  _cycle_time_ms_type cycle_time_ms;
  using _target_x_type =
    float;
  _target_x_type target_x;
  using _target_y_type =
    float;
  _target_y_type target_y;
  using _target_z_type =
    float;
  _target_z_type target_z;
  using _target_yaw_type =
    float;
  _target_yaw_type target_yaw;

  // setters for named parameter idiom
  Type & set__pos_x(
    const float & _arg)
  {
    this->pos_x = _arg;
    return *this;
  }
  Type & set__pos_y(
    const float & _arg)
  {
    this->pos_y = _arg;
    return *this;
  }
  Type & set__pos_z(
    const float & _arg)
  {
    this->pos_z = _arg;
    return *this;
  }
  Type & set__yaw(
    const float & _arg)
  {
    this->yaw = _arg;
    return *this;
  }
  Type & set__vel_x(
    const float & _arg)
  {
    this->vel_x = _arg;
    return *this;
  }
  Type & set__vel_y(
    const float & _arg)
  {
    this->vel_y = _arg;
    return *this;
  }
  Type & set__vel_z(
    const float & _arg)
  {
    this->vel_z = _arg;
    return *this;
  }
  Type & set__yaw_rate(
    const float & _arg)
  {
    this->yaw_rate = _arg;
    return *this;
  }
  Type & set__vel_world_x(
    const float & _arg)
  {
    this->vel_world_x = _arg;
    return *this;
  }
  Type & set__vel_world_y(
    const float & _arg)
  {
    this->vel_world_y = _arg;
    return *this;
  }
  Type & set__armed(
    const bool & _arg)
  {
    this->armed = _arg;
    return *this;
  }
  Type & set__control_mode(
    const uint8_t & _arg)
  {
    this->control_mode = _arg;
    return *this;
  }
  Type & set__battery_voltage(
    const float & _arg)
  {
    this->battery_voltage = _arg;
    return *this;
  }
  Type & set__error_flags(
    const uint32_t & _arg)
  {
    this->error_flags = _arg;
    return *this;
  }
  Type & set__cycle_time_ms(
    const float & _arg)
  {
    this->cycle_time_ms = _arg;
    return *this;
  }
  Type & set__target_x(
    const float & _arg)
  {
    this->target_x = _arg;
    return *this;
  }
  Type & set__target_y(
    const float & _arg)
  {
    this->target_y = _arg;
    return *this;
  }
  Type & set__target_z(
    const float & _arg)
  {
    this->target_z = _arg;
    return *this;
  }
  Type & set__target_yaw(
    const float & _arg)
  {
    this->target_yaw = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    uv_msgs::msg::AuvState_<ContainerAllocator> *;
  using ConstRawPtr =
    const uv_msgs::msg::AuvState_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<uv_msgs::msg::AuvState_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<uv_msgs::msg::AuvState_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      uv_msgs::msg::AuvState_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<uv_msgs::msg::AuvState_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      uv_msgs::msg::AuvState_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<uv_msgs::msg::AuvState_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<uv_msgs::msg::AuvState_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<uv_msgs::msg::AuvState_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__uv_msgs__msg__AuvState
    std::shared_ptr<uv_msgs::msg::AuvState_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__uv_msgs__msg__AuvState
    std::shared_ptr<uv_msgs::msg::AuvState_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const AuvState_ & other) const
  {
    if (this->pos_x != other.pos_x) {
      return false;
    }
    if (this->pos_y != other.pos_y) {
      return false;
    }
    if (this->pos_z != other.pos_z) {
      return false;
    }
    if (this->yaw != other.yaw) {
      return false;
    }
    if (this->vel_x != other.vel_x) {
      return false;
    }
    if (this->vel_y != other.vel_y) {
      return false;
    }
    if (this->vel_z != other.vel_z) {
      return false;
    }
    if (this->yaw_rate != other.yaw_rate) {
      return false;
    }
    if (this->vel_world_x != other.vel_world_x) {
      return false;
    }
    if (this->vel_world_y != other.vel_world_y) {
      return false;
    }
    if (this->armed != other.armed) {
      return false;
    }
    if (this->control_mode != other.control_mode) {
      return false;
    }
    if (this->battery_voltage != other.battery_voltage) {
      return false;
    }
    if (this->error_flags != other.error_flags) {
      return false;
    }
    if (this->cycle_time_ms != other.cycle_time_ms) {
      return false;
    }
    if (this->target_x != other.target_x) {
      return false;
    }
    if (this->target_y != other.target_y) {
      return false;
    }
    if (this->target_z != other.target_z) {
      return false;
    }
    if (this->target_yaw != other.target_yaw) {
      return false;
    }
    return true;
  }
  bool operator!=(const AuvState_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct AuvState_

// alias to use template instance with default allocator
using AuvState =
  uv_msgs::msg::AuvState_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace uv_msgs

#endif  // UV_MSGS__MSG__DETAIL__AUV_STATE__STRUCT_HPP_
