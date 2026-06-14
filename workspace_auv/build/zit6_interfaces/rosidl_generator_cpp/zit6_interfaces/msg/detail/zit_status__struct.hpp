// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from zit6_interfaces:msg/ZitStatus.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "zit6_interfaces/msg/zit_status.hpp"


#ifndef ZIT6_INTERFACES__MSG__DETAIL__ZIT_STATUS__STRUCT_HPP_
#define ZIT6_INTERFACES__MSG__DETAIL__ZIT_STATUS__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__zit6_interfaces__msg__ZitStatus __attribute__((deprecated))
#else
# define DEPRECATED__zit6_interfaces__msg__ZitStatus __declspec(deprecated)
#endif

namespace zit6_interfaces
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct ZitStatus_
{
  using Type = ZitStatus_<ContainerAllocator>;

  explicit ZitStatus_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->is_armed = false;
      this->arm_mode = 0;
      this->control_level = 0;
      this->ins_state = 0;
      this->navigation_ready = false;
      std::fill<typename std::array<float, 4>::iterator, float>(this->forces.begin(), this->forces.end(), 0.0f);
      this->cycle_time_ms = 0.0f;
      this->battery_voltage = 0.0f;
      this->error_flags = 0ul;
    }
  }

  explicit ZitStatus_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : forces(_alloc)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->is_armed = false;
      this->arm_mode = 0;
      this->control_level = 0;
      this->ins_state = 0;
      this->navigation_ready = false;
      std::fill<typename std::array<float, 4>::iterator, float>(this->forces.begin(), this->forces.end(), 0.0f);
      this->cycle_time_ms = 0.0f;
      this->battery_voltage = 0.0f;
      this->error_flags = 0ul;
    }
  }

  // field types and members
  using _is_armed_type =
    bool;
  _is_armed_type is_armed;
  using _arm_mode_type =
    uint8_t;
  _arm_mode_type arm_mode;
  using _control_level_type =
    uint8_t;
  _control_level_type control_level;
  using _ins_state_type =
    uint8_t;
  _ins_state_type ins_state;
  using _navigation_ready_type =
    bool;
  _navigation_ready_type navigation_ready;
  using _forces_type =
    std::array<float, 4>;
  _forces_type forces;
  using _cycle_time_ms_type =
    float;
  _cycle_time_ms_type cycle_time_ms;
  using _battery_voltage_type =
    float;
  _battery_voltage_type battery_voltage;
  using _error_flags_type =
    uint32_t;
  _error_flags_type error_flags;

  // setters for named parameter idiom
  Type & set__is_armed(
    const bool & _arg)
  {
    this->is_armed = _arg;
    return *this;
  }
  Type & set__arm_mode(
    const uint8_t & _arg)
  {
    this->arm_mode = _arg;
    return *this;
  }
  Type & set__control_level(
    const uint8_t & _arg)
  {
    this->control_level = _arg;
    return *this;
  }
  Type & set__ins_state(
    const uint8_t & _arg)
  {
    this->ins_state = _arg;
    return *this;
  }
  Type & set__navigation_ready(
    const bool & _arg)
  {
    this->navigation_ready = _arg;
    return *this;
  }
  Type & set__forces(
    const std::array<float, 4> & _arg)
  {
    this->forces = _arg;
    return *this;
  }
  Type & set__cycle_time_ms(
    const float & _arg)
  {
    this->cycle_time_ms = _arg;
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

  // constant declarations
  static constexpr uint8_t LEVEL_NONE =
    0u;
  static constexpr uint8_t LEVEL_POS =
    1u;
  static constexpr uint8_t LEVEL_VEL =
    2u;
  static constexpr uint8_t LEVEL_FORCE =
    3u;
  static constexpr uint32_t ERROR_FORCE_STOP =
    1u;
  static constexpr uint32_t ERROR_SENSOR_FAIL =
    2u;
  static constexpr uint32_t ERROR_VOLTAGE_LOW =
    4u;
  static constexpr uint32_t ERROR_COMM_TIMEOUT =
    8u;

  // pointer types
  using RawPtr =
    zit6_interfaces::msg::ZitStatus_<ContainerAllocator> *;
  using ConstRawPtr =
    const zit6_interfaces::msg::ZitStatus_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<zit6_interfaces::msg::ZitStatus_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<zit6_interfaces::msg::ZitStatus_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      zit6_interfaces::msg::ZitStatus_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<zit6_interfaces::msg::ZitStatus_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      zit6_interfaces::msg::ZitStatus_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<zit6_interfaces::msg::ZitStatus_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<zit6_interfaces::msg::ZitStatus_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<zit6_interfaces::msg::ZitStatus_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__zit6_interfaces__msg__ZitStatus
    std::shared_ptr<zit6_interfaces::msg::ZitStatus_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__zit6_interfaces__msg__ZitStatus
    std::shared_ptr<zit6_interfaces::msg::ZitStatus_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const ZitStatus_ & other) const
  {
    if (this->is_armed != other.is_armed) {
      return false;
    }
    if (this->arm_mode != other.arm_mode) {
      return false;
    }
    if (this->control_level != other.control_level) {
      return false;
    }
    if (this->ins_state != other.ins_state) {
      return false;
    }
    if (this->navigation_ready != other.navigation_ready) {
      return false;
    }
    if (this->forces != other.forces) {
      return false;
    }
    if (this->cycle_time_ms != other.cycle_time_ms) {
      return false;
    }
    if (this->battery_voltage != other.battery_voltage) {
      return false;
    }
    if (this->error_flags != other.error_flags) {
      return false;
    }
    return true;
  }
  bool operator!=(const ZitStatus_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct ZitStatus_

// alias to use template instance with default allocator
using ZitStatus =
  zit6_interfaces::msg::ZitStatus_<std::allocator<void>>;

// constant definitions
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t ZitStatus_<ContainerAllocator>::LEVEL_NONE;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t ZitStatus_<ContainerAllocator>::LEVEL_POS;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t ZitStatus_<ContainerAllocator>::LEVEL_VEL;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t ZitStatus_<ContainerAllocator>::LEVEL_FORCE;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint32_t ZitStatus_<ContainerAllocator>::ERROR_FORCE_STOP;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint32_t ZitStatus_<ContainerAllocator>::ERROR_SENSOR_FAIL;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint32_t ZitStatus_<ContainerAllocator>::ERROR_VOLTAGE_LOW;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint32_t ZitStatus_<ContainerAllocator>::ERROR_COMM_TIMEOUT;
#endif  // __cplusplus < 201703L

}  // namespace msg

}  // namespace zit6_interfaces

#endif  // ZIT6_INTERFACES__MSG__DETAIL__ZIT_STATUS__STRUCT_HPP_
