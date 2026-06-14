// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from uv_msgs:msg/PidGains.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "uv_msgs/msg/pid_gains.hpp"


#ifndef UV_MSGS__MSG__DETAIL__PID_GAINS__STRUCT_HPP_
#define UV_MSGS__MSG__DETAIL__PID_GAINS__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__uv_msgs__msg__PidGains __attribute__((deprecated))
#else
# define DEPRECATED__uv_msgs__msg__PidGains __declspec(deprecated)
#endif

namespace uv_msgs
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct PidGains_
{
  using Type = PidGains_<ContainerAllocator>;

  explicit PidGains_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->name = "";
      this->kp = 0.0f;
      this->ki = 0.0f;
      this->kd = 0.0f;
      this->i_limit = 0.0f;
      this->output_limit = 0.0f;
      this->feedforward = 0.0f;
    }
  }

  explicit PidGains_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : name(_alloc)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->name = "";
      this->kp = 0.0f;
      this->ki = 0.0f;
      this->kd = 0.0f;
      this->i_limit = 0.0f;
      this->output_limit = 0.0f;
      this->feedforward = 0.0f;
    }
  }

  // field types and members
  using _name_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _name_type name;
  using _kp_type =
    float;
  _kp_type kp;
  using _ki_type =
    float;
  _ki_type ki;
  using _kd_type =
    float;
  _kd_type kd;
  using _i_limit_type =
    float;
  _i_limit_type i_limit;
  using _output_limit_type =
    float;
  _output_limit_type output_limit;
  using _feedforward_type =
    float;
  _feedforward_type feedforward;

  // setters for named parameter idiom
  Type & set__name(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->name = _arg;
    return *this;
  }
  Type & set__kp(
    const float & _arg)
  {
    this->kp = _arg;
    return *this;
  }
  Type & set__ki(
    const float & _arg)
  {
    this->ki = _arg;
    return *this;
  }
  Type & set__kd(
    const float & _arg)
  {
    this->kd = _arg;
    return *this;
  }
  Type & set__i_limit(
    const float & _arg)
  {
    this->i_limit = _arg;
    return *this;
  }
  Type & set__output_limit(
    const float & _arg)
  {
    this->output_limit = _arg;
    return *this;
  }
  Type & set__feedforward(
    const float & _arg)
  {
    this->feedforward = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    uv_msgs::msg::PidGains_<ContainerAllocator> *;
  using ConstRawPtr =
    const uv_msgs::msg::PidGains_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<uv_msgs::msg::PidGains_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<uv_msgs::msg::PidGains_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      uv_msgs::msg::PidGains_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<uv_msgs::msg::PidGains_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      uv_msgs::msg::PidGains_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<uv_msgs::msg::PidGains_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<uv_msgs::msg::PidGains_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<uv_msgs::msg::PidGains_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__uv_msgs__msg__PidGains
    std::shared_ptr<uv_msgs::msg::PidGains_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__uv_msgs__msg__PidGains
    std::shared_ptr<uv_msgs::msg::PidGains_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const PidGains_ & other) const
  {
    if (this->name != other.name) {
      return false;
    }
    if (this->kp != other.kp) {
      return false;
    }
    if (this->ki != other.ki) {
      return false;
    }
    if (this->kd != other.kd) {
      return false;
    }
    if (this->i_limit != other.i_limit) {
      return false;
    }
    if (this->output_limit != other.output_limit) {
      return false;
    }
    if (this->feedforward != other.feedforward) {
      return false;
    }
    return true;
  }
  bool operator!=(const PidGains_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct PidGains_

// alias to use template instance with default allocator
using PidGains =
  uv_msgs::msg::PidGains_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace uv_msgs

#endif  // UV_MSGS__MSG__DETAIL__PID_GAINS__STRUCT_HPP_
