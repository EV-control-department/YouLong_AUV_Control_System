// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from uv_msgs:msg/TaskStatus.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "uv_msgs/msg/task_status.hpp"


#ifndef UV_MSGS__MSG__DETAIL__TASK_STATUS__STRUCT_HPP_
#define UV_MSGS__MSG__DETAIL__TASK_STATUS__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__uv_msgs__msg__TaskStatus __attribute__((deprecated))
#else
# define DEPRECATED__uv_msgs__msg__TaskStatus __declspec(deprecated)
#endif

namespace uv_msgs
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct TaskStatus_
{
  using Type = TaskStatus_<ContainerAllocator>;

  explicit TaskStatus_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->status = 0;
      this->current_task_index = 0ul;
      this->total_tasks = 0ul;
      this->current_task_name = "";
      this->error_message = "";
    }
  }

  explicit TaskStatus_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : current_task_name(_alloc),
    error_message(_alloc)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->status = 0;
      this->current_task_index = 0ul;
      this->total_tasks = 0ul;
      this->current_task_name = "";
      this->error_message = "";
    }
  }

  // field types and members
  using _status_type =
    uint8_t;
  _status_type status;
  using _current_task_index_type =
    uint32_t;
  _current_task_index_type current_task_index;
  using _total_tasks_type =
    uint32_t;
  _total_tasks_type total_tasks;
  using _current_task_name_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _current_task_name_type current_task_name;
  using _error_message_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _error_message_type error_message;

  // setters for named parameter idiom
  Type & set__status(
    const uint8_t & _arg)
  {
    this->status = _arg;
    return *this;
  }
  Type & set__current_task_index(
    const uint32_t & _arg)
  {
    this->current_task_index = _arg;
    return *this;
  }
  Type & set__total_tasks(
    const uint32_t & _arg)
  {
    this->total_tasks = _arg;
    return *this;
  }
  Type & set__current_task_name(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->current_task_name = _arg;
    return *this;
  }
  Type & set__error_message(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->error_message = _arg;
    return *this;
  }

  // constant declarations
  static constexpr uint8_t STATUS_IDLE =
    0u;
  static constexpr uint8_t STATUS_RUNNING =
    1u;
  static constexpr uint8_t STATUS_PAUSED =
    2u;
  static constexpr uint8_t STATUS_DONE =
    3u;
  static constexpr uint8_t STATUS_ERROR =
    4u;

  // pointer types
  using RawPtr =
    uv_msgs::msg::TaskStatus_<ContainerAllocator> *;
  using ConstRawPtr =
    const uv_msgs::msg::TaskStatus_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<uv_msgs::msg::TaskStatus_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<uv_msgs::msg::TaskStatus_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      uv_msgs::msg::TaskStatus_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<uv_msgs::msg::TaskStatus_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      uv_msgs::msg::TaskStatus_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<uv_msgs::msg::TaskStatus_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<uv_msgs::msg::TaskStatus_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<uv_msgs::msg::TaskStatus_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__uv_msgs__msg__TaskStatus
    std::shared_ptr<uv_msgs::msg::TaskStatus_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__uv_msgs__msg__TaskStatus
    std::shared_ptr<uv_msgs::msg::TaskStatus_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const TaskStatus_ & other) const
  {
    if (this->status != other.status) {
      return false;
    }
    if (this->current_task_index != other.current_task_index) {
      return false;
    }
    if (this->total_tasks != other.total_tasks) {
      return false;
    }
    if (this->current_task_name != other.current_task_name) {
      return false;
    }
    if (this->error_message != other.error_message) {
      return false;
    }
    return true;
  }
  bool operator!=(const TaskStatus_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct TaskStatus_

// alias to use template instance with default allocator
using TaskStatus =
  uv_msgs::msg::TaskStatus_<std::allocator<void>>;

// constant definitions
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t TaskStatus_<ContainerAllocator>::STATUS_IDLE;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t TaskStatus_<ContainerAllocator>::STATUS_RUNNING;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t TaskStatus_<ContainerAllocator>::STATUS_PAUSED;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t TaskStatus_<ContainerAllocator>::STATUS_DONE;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t TaskStatus_<ContainerAllocator>::STATUS_ERROR;
#endif  // __cplusplus < 201703L

}  // namespace msg

}  // namespace uv_msgs

#endif  // UV_MSGS__MSG__DETAIL__TASK_STATUS__STRUCT_HPP_
