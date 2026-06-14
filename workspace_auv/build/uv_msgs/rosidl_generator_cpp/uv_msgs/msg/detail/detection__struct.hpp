// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from uv_msgs:msg/Detection.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "uv_msgs/msg/detection.hpp"


#ifndef UV_MSGS__MSG__DETAIL__DETECTION__STRUCT_HPP_
#define UV_MSGS__MSG__DETAIL__DETECTION__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__uv_msgs__msg__Detection __attribute__((deprecated))
#else
# define DEPRECATED__uv_msgs__msg__Detection __declspec(deprecated)
#endif

namespace uv_msgs
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct Detection_
{
  using Type = Detection_<ContainerAllocator>;

  explicit Detection_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->class_id = 0;
      this->confidence = 0.0f;
      this->pixel_x = 0.0f;
      this->pixel_y = 0.0f;
      this->bbox_x1 = 0.0f;
      this->bbox_y1 = 0.0f;
      this->bbox_x2 = 0.0f;
      this->bbox_y2 = 0.0f;
    }
  }

  explicit Detection_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_alloc;
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->class_id = 0;
      this->confidence = 0.0f;
      this->pixel_x = 0.0f;
      this->pixel_y = 0.0f;
      this->bbox_x1 = 0.0f;
      this->bbox_y1 = 0.0f;
      this->bbox_x2 = 0.0f;
      this->bbox_y2 = 0.0f;
    }
  }

  // field types and members
  using _class_id_type =
    uint16_t;
  _class_id_type class_id;
  using _confidence_type =
    float;
  _confidence_type confidence;
  using _pixel_x_type =
    float;
  _pixel_x_type pixel_x;
  using _pixel_y_type =
    float;
  _pixel_y_type pixel_y;
  using _bbox_x1_type =
    float;
  _bbox_x1_type bbox_x1;
  using _bbox_y1_type =
    float;
  _bbox_y1_type bbox_y1;
  using _bbox_x2_type =
    float;
  _bbox_x2_type bbox_x2;
  using _bbox_y2_type =
    float;
  _bbox_y2_type bbox_y2;

  // setters for named parameter idiom
  Type & set__class_id(
    const uint16_t & _arg)
  {
    this->class_id = _arg;
    return *this;
  }
  Type & set__confidence(
    const float & _arg)
  {
    this->confidence = _arg;
    return *this;
  }
  Type & set__pixel_x(
    const float & _arg)
  {
    this->pixel_x = _arg;
    return *this;
  }
  Type & set__pixel_y(
    const float & _arg)
  {
    this->pixel_y = _arg;
    return *this;
  }
  Type & set__bbox_x1(
    const float & _arg)
  {
    this->bbox_x1 = _arg;
    return *this;
  }
  Type & set__bbox_y1(
    const float & _arg)
  {
    this->bbox_y1 = _arg;
    return *this;
  }
  Type & set__bbox_x2(
    const float & _arg)
  {
    this->bbox_x2 = _arg;
    return *this;
  }
  Type & set__bbox_y2(
    const float & _arg)
  {
    this->bbox_y2 = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    uv_msgs::msg::Detection_<ContainerAllocator> *;
  using ConstRawPtr =
    const uv_msgs::msg::Detection_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<uv_msgs::msg::Detection_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<uv_msgs::msg::Detection_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      uv_msgs::msg::Detection_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<uv_msgs::msg::Detection_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      uv_msgs::msg::Detection_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<uv_msgs::msg::Detection_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<uv_msgs::msg::Detection_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<uv_msgs::msg::Detection_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__uv_msgs__msg__Detection
    std::shared_ptr<uv_msgs::msg::Detection_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__uv_msgs__msg__Detection
    std::shared_ptr<uv_msgs::msg::Detection_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const Detection_ & other) const
  {
    if (this->class_id != other.class_id) {
      return false;
    }
    if (this->confidence != other.confidence) {
      return false;
    }
    if (this->pixel_x != other.pixel_x) {
      return false;
    }
    if (this->pixel_y != other.pixel_y) {
      return false;
    }
    if (this->bbox_x1 != other.bbox_x1) {
      return false;
    }
    if (this->bbox_y1 != other.bbox_y1) {
      return false;
    }
    if (this->bbox_x2 != other.bbox_x2) {
      return false;
    }
    if (this->bbox_y2 != other.bbox_y2) {
      return false;
    }
    return true;
  }
  bool operator!=(const Detection_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct Detection_

// alias to use template instance with default allocator
using Detection =
  uv_msgs::msg::Detection_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace uv_msgs

#endif  // UV_MSGS__MSG__DETAIL__DETECTION__STRUCT_HPP_
