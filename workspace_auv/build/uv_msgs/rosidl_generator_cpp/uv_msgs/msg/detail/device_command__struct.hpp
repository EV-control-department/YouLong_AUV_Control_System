// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from uv_msgs:msg/DeviceCommand.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "uv_msgs/msg/device_command.hpp"


#ifndef UV_MSGS__MSG__DETAIL__DEVICE_COMMAND__STRUCT_HPP_
#define UV_MSGS__MSG__DETAIL__DEVICE_COMMAND__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__uv_msgs__msg__DeviceCommand __attribute__((deprecated))
#else
# define DEPRECATED__uv_msgs__msg__DeviceCommand __declspec(deprecated)
#endif

namespace uv_msgs
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct DeviceCommand_
{
  using Type = DeviceCommand_<ContainerAllocator>;

  explicit DeviceCommand_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->device_type = 0;
      this->command = 0;
      this->value = 0.0f;
    }
  }

  explicit DeviceCommand_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_alloc;
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->device_type = 0;
      this->command = 0;
      this->value = 0.0f;
    }
  }

  // field types and members
  using _device_type_type =
    uint8_t;
  _device_type_type device_type;
  using _command_type =
    uint8_t;
  _command_type command;
  using _value_type =
    float;
  _value_type value;

  // setters for named parameter idiom
  Type & set__device_type(
    const uint8_t & _arg)
  {
    this->device_type = _arg;
    return *this;
  }
  Type & set__command(
    const uint8_t & _arg)
  {
    this->command = _arg;
    return *this;
  }
  Type & set__value(
    const float & _arg)
  {
    this->value = _arg;
    return *this;
  }

  // constant declarations
  static constexpr uint8_t DEVICE_LED =
    0u;
  static constexpr uint8_t DEVICE_ARM =
    1u;
  static constexpr uint8_t DEVICE_SERVO =
    2u;

  // pointer types
  using RawPtr =
    uv_msgs::msg::DeviceCommand_<ContainerAllocator> *;
  using ConstRawPtr =
    const uv_msgs::msg::DeviceCommand_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<uv_msgs::msg::DeviceCommand_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<uv_msgs::msg::DeviceCommand_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      uv_msgs::msg::DeviceCommand_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<uv_msgs::msg::DeviceCommand_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      uv_msgs::msg::DeviceCommand_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<uv_msgs::msg::DeviceCommand_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<uv_msgs::msg::DeviceCommand_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<uv_msgs::msg::DeviceCommand_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__uv_msgs__msg__DeviceCommand
    std::shared_ptr<uv_msgs::msg::DeviceCommand_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__uv_msgs__msg__DeviceCommand
    std::shared_ptr<uv_msgs::msg::DeviceCommand_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const DeviceCommand_ & other) const
  {
    if (this->device_type != other.device_type) {
      return false;
    }
    if (this->command != other.command) {
      return false;
    }
    if (this->value != other.value) {
      return false;
    }
    return true;
  }
  bool operator!=(const DeviceCommand_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct DeviceCommand_

// alias to use template instance with default allocator
using DeviceCommand =
  uv_msgs::msg::DeviceCommand_<std::allocator<void>>;

// constant definitions
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t DeviceCommand_<ContainerAllocator>::DEVICE_LED;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t DeviceCommand_<ContainerAllocator>::DEVICE_ARM;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t DeviceCommand_<ContainerAllocator>::DEVICE_SERVO;
#endif  // __cplusplus < 201703L

}  // namespace msg

}  // namespace uv_msgs

#endif  // UV_MSGS__MSG__DETAIL__DEVICE_COMMAND__STRUCT_HPP_
