// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from zit6_interfaces:msg/ZitPidStatus.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "zit6_interfaces/msg/zit_pid_status.hpp"


#ifndef ZIT6_INTERFACES__MSG__DETAIL__ZIT_PID_STATUS__STRUCT_HPP_
#define ZIT6_INTERFACES__MSG__DETAIL__ZIT_PID_STATUS__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__zit6_interfaces__msg__ZitPidStatus __attribute__((deprecated))
#else
# define DEPRECATED__zit6_interfaces__msg__ZitPidStatus __declspec(deprecated)
#endif

namespace zit6_interfaces
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct ZitPidStatus_
{
  using Type = ZitPidStatus_<ContainerAllocator>;

  explicit ZitPidStatus_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      std::fill<typename std::array<float, 4>::iterator, float>(this->pos_kp.begin(), this->pos_kp.end(), 0.0f);
      std::fill<typename std::array<float, 4>::iterator, float>(this->pos_max_v.begin(), this->pos_max_v.end(), 0.0f);
      std::fill<typename std::array<float, 4>::iterator, float>(this->pos_max_a.begin(), this->pos_max_a.end(), 0.0f);
      std::fill<typename std::array<float, 4>::iterator, float>(this->pos_out_limit.begin(), this->pos_out_limit.end(), 0.0f);
      std::fill<typename std::array<float, 4>::iterator, float>(this->vel_kp.begin(), this->vel_kp.end(), 0.0f);
      std::fill<typename std::array<float, 4>::iterator, float>(this->vel_ki.begin(), this->vel_ki.end(), 0.0f);
      std::fill<typename std::array<float, 4>::iterator, float>(this->vel_kd.begin(), this->vel_kd.end(), 0.0f);
      std::fill<typename std::array<float, 4>::iterator, float>(this->vel_i_limit.begin(), this->vel_i_limit.end(), 0.0f);
      std::fill<typename std::array<float, 4>::iterator, float>(this->vel_out_limit.begin(), this->vel_out_limit.end(), 0.0f);
    }
  }

  explicit ZitPidStatus_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : pos_kp(_alloc),
    pos_max_v(_alloc),
    pos_max_a(_alloc),
    pos_out_limit(_alloc),
    vel_kp(_alloc),
    vel_ki(_alloc),
    vel_kd(_alloc),
    vel_i_limit(_alloc),
    vel_out_limit(_alloc)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      std::fill<typename std::array<float, 4>::iterator, float>(this->pos_kp.begin(), this->pos_kp.end(), 0.0f);
      std::fill<typename std::array<float, 4>::iterator, float>(this->pos_max_v.begin(), this->pos_max_v.end(), 0.0f);
      std::fill<typename std::array<float, 4>::iterator, float>(this->pos_max_a.begin(), this->pos_max_a.end(), 0.0f);
      std::fill<typename std::array<float, 4>::iterator, float>(this->pos_out_limit.begin(), this->pos_out_limit.end(), 0.0f);
      std::fill<typename std::array<float, 4>::iterator, float>(this->vel_kp.begin(), this->vel_kp.end(), 0.0f);
      std::fill<typename std::array<float, 4>::iterator, float>(this->vel_ki.begin(), this->vel_ki.end(), 0.0f);
      std::fill<typename std::array<float, 4>::iterator, float>(this->vel_kd.begin(), this->vel_kd.end(), 0.0f);
      std::fill<typename std::array<float, 4>::iterator, float>(this->vel_i_limit.begin(), this->vel_i_limit.end(), 0.0f);
      std::fill<typename std::array<float, 4>::iterator, float>(this->vel_out_limit.begin(), this->vel_out_limit.end(), 0.0f);
    }
  }

  // field types and members
  using _pos_kp_type =
    std::array<float, 4>;
  _pos_kp_type pos_kp;
  using _pos_max_v_type =
    std::array<float, 4>;
  _pos_max_v_type pos_max_v;
  using _pos_max_a_type =
    std::array<float, 4>;
  _pos_max_a_type pos_max_a;
  using _pos_out_limit_type =
    std::array<float, 4>;
  _pos_out_limit_type pos_out_limit;
  using _vel_kp_type =
    std::array<float, 4>;
  _vel_kp_type vel_kp;
  using _vel_ki_type =
    std::array<float, 4>;
  _vel_ki_type vel_ki;
  using _vel_kd_type =
    std::array<float, 4>;
  _vel_kd_type vel_kd;
  using _vel_i_limit_type =
    std::array<float, 4>;
  _vel_i_limit_type vel_i_limit;
  using _vel_out_limit_type =
    std::array<float, 4>;
  _vel_out_limit_type vel_out_limit;

  // setters for named parameter idiom
  Type & set__pos_kp(
    const std::array<float, 4> & _arg)
  {
    this->pos_kp = _arg;
    return *this;
  }
  Type & set__pos_max_v(
    const std::array<float, 4> & _arg)
  {
    this->pos_max_v = _arg;
    return *this;
  }
  Type & set__pos_max_a(
    const std::array<float, 4> & _arg)
  {
    this->pos_max_a = _arg;
    return *this;
  }
  Type & set__pos_out_limit(
    const std::array<float, 4> & _arg)
  {
    this->pos_out_limit = _arg;
    return *this;
  }
  Type & set__vel_kp(
    const std::array<float, 4> & _arg)
  {
    this->vel_kp = _arg;
    return *this;
  }
  Type & set__vel_ki(
    const std::array<float, 4> & _arg)
  {
    this->vel_ki = _arg;
    return *this;
  }
  Type & set__vel_kd(
    const std::array<float, 4> & _arg)
  {
    this->vel_kd = _arg;
    return *this;
  }
  Type & set__vel_i_limit(
    const std::array<float, 4> & _arg)
  {
    this->vel_i_limit = _arg;
    return *this;
  }
  Type & set__vel_out_limit(
    const std::array<float, 4> & _arg)
  {
    this->vel_out_limit = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    zit6_interfaces::msg::ZitPidStatus_<ContainerAllocator> *;
  using ConstRawPtr =
    const zit6_interfaces::msg::ZitPidStatus_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<zit6_interfaces::msg::ZitPidStatus_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<zit6_interfaces::msg::ZitPidStatus_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      zit6_interfaces::msg::ZitPidStatus_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<zit6_interfaces::msg::ZitPidStatus_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      zit6_interfaces::msg::ZitPidStatus_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<zit6_interfaces::msg::ZitPidStatus_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<zit6_interfaces::msg::ZitPidStatus_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<zit6_interfaces::msg::ZitPidStatus_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__zit6_interfaces__msg__ZitPidStatus
    std::shared_ptr<zit6_interfaces::msg::ZitPidStatus_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__zit6_interfaces__msg__ZitPidStatus
    std::shared_ptr<zit6_interfaces::msg::ZitPidStatus_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const ZitPidStatus_ & other) const
  {
    if (this->pos_kp != other.pos_kp) {
      return false;
    }
    if (this->pos_max_v != other.pos_max_v) {
      return false;
    }
    if (this->pos_max_a != other.pos_max_a) {
      return false;
    }
    if (this->pos_out_limit != other.pos_out_limit) {
      return false;
    }
    if (this->vel_kp != other.vel_kp) {
      return false;
    }
    if (this->vel_ki != other.vel_ki) {
      return false;
    }
    if (this->vel_kd != other.vel_kd) {
      return false;
    }
    if (this->vel_i_limit != other.vel_i_limit) {
      return false;
    }
    if (this->vel_out_limit != other.vel_out_limit) {
      return false;
    }
    return true;
  }
  bool operator!=(const ZitPidStatus_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct ZitPidStatus_

// alias to use template instance with default allocator
using ZitPidStatus =
  zit6_interfaces::msg::ZitPidStatus_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace zit6_interfaces

#endif  // ZIT6_INTERFACES__MSG__DETAIL__ZIT_PID_STATUS__STRUCT_HPP_
