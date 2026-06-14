// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from zit6_interfaces:msg/ZitPid.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "zit6_interfaces/msg/zit_pid.hpp"


#ifndef ZIT6_INTERFACES__MSG__DETAIL__ZIT_PID__STRUCT_HPP_
#define ZIT6_INTERFACES__MSG__DETAIL__ZIT_PID__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__zit6_interfaces__msg__ZitPid __attribute__((deprecated))
#else
# define DEPRECATED__zit6_interfaces__msg__ZitPid __declspec(deprecated)
#endif

namespace zit6_interfaces
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct ZitPid_
{
  using Type = ZitPid_<ContainerAllocator>;

  explicit ZitPid_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->axis = 0;
      this->is_pos_ring = false;
      this->kp = 0.0f;
      this->ki = 0.0f;
      this->kd = 0.0f;
      this->i_limit = 0.0f;
      this->out_limit = 0.0f;
      this->max_v = 0.0f;
      this->max_a = 0.0f;
    }
  }

  explicit ZitPid_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_alloc;
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->axis = 0;
      this->is_pos_ring = false;
      this->kp = 0.0f;
      this->ki = 0.0f;
      this->kd = 0.0f;
      this->i_limit = 0.0f;
      this->out_limit = 0.0f;
      this->max_v = 0.0f;
      this->max_a = 0.0f;
    }
  }

  // field types and members
  using _axis_type =
    uint8_t;
  _axis_type axis;
  using _is_pos_ring_type =
    bool;
  _is_pos_ring_type is_pos_ring;
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
  using _out_limit_type =
    float;
  _out_limit_type out_limit;
  using _max_v_type =
    float;
  _max_v_type max_v;
  using _max_a_type =
    float;
  _max_a_type max_a;

  // setters for named parameter idiom
  Type & set__axis(
    const uint8_t & _arg)
  {
    this->axis = _arg;
    return *this;
  }
  Type & set__is_pos_ring(
    const bool & _arg)
  {
    this->is_pos_ring = _arg;
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
  Type & set__out_limit(
    const float & _arg)
  {
    this->out_limit = _arg;
    return *this;
  }
  Type & set__max_v(
    const float & _arg)
  {
    this->max_v = _arg;
    return *this;
  }
  Type & set__max_a(
    const float & _arg)
  {
    this->max_a = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    zit6_interfaces::msg::ZitPid_<ContainerAllocator> *;
  using ConstRawPtr =
    const zit6_interfaces::msg::ZitPid_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<zit6_interfaces::msg::ZitPid_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<zit6_interfaces::msg::ZitPid_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      zit6_interfaces::msg::ZitPid_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<zit6_interfaces::msg::ZitPid_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      zit6_interfaces::msg::ZitPid_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<zit6_interfaces::msg::ZitPid_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<zit6_interfaces::msg::ZitPid_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<zit6_interfaces::msg::ZitPid_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__zit6_interfaces__msg__ZitPid
    std::shared_ptr<zit6_interfaces::msg::ZitPid_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__zit6_interfaces__msg__ZitPid
    std::shared_ptr<zit6_interfaces::msg::ZitPid_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const ZitPid_ & other) const
  {
    if (this->axis != other.axis) {
      return false;
    }
    if (this->is_pos_ring != other.is_pos_ring) {
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
    if (this->out_limit != other.out_limit) {
      return false;
    }
    if (this->max_v != other.max_v) {
      return false;
    }
    if (this->max_a != other.max_a) {
      return false;
    }
    return true;
  }
  bool operator!=(const ZitPid_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct ZitPid_

// alias to use template instance with default allocator
using ZitPid =
  zit6_interfaces::msg::ZitPid_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace zit6_interfaces

#endif  // ZIT6_INTERFACES__MSG__DETAIL__ZIT_PID__STRUCT_HPP_
