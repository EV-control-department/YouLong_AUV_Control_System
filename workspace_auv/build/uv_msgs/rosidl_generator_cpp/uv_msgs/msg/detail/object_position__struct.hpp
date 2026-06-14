// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from uv_msgs:msg/ObjectPosition.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "uv_msgs/msg/object_position.hpp"


#ifndef UV_MSGS__MSG__DETAIL__OBJECT_POSITION__STRUCT_HPP_
#define UV_MSGS__MSG__DETAIL__OBJECT_POSITION__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__uv_msgs__msg__ObjectPosition __attribute__((deprecated))
#else
# define DEPRECATED__uv_msgs__msg__ObjectPosition __declspec(deprecated)
#endif

namespace uv_msgs
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct ObjectPosition_
{
  using Type = ObjectPosition_<ContainerAllocator>;

  explicit ObjectPosition_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->class_id = 0;
      this->class_name = "";
      this->world_x = 0.0f;
      this->world_y = 0.0f;
      this->world_z = 0.0f;
      this->confidence = 0.0f;
      this->num_observations = 0ul;
    }
  }

  explicit ObjectPosition_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : class_name(_alloc)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->class_id = 0;
      this->class_name = "";
      this->world_x = 0.0f;
      this->world_y = 0.0f;
      this->world_z = 0.0f;
      this->confidence = 0.0f;
      this->num_observations = 0ul;
    }
  }

  // field types and members
  using _class_id_type =
    uint16_t;
  _class_id_type class_id;
  using _class_name_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _class_name_type class_name;
  using _world_x_type =
    float;
  _world_x_type world_x;
  using _world_y_type =
    float;
  _world_y_type world_y;
  using _world_z_type =
    float;
  _world_z_type world_z;
  using _confidence_type =
    float;
  _confidence_type confidence;
  using _num_observations_type =
    uint32_t;
  _num_observations_type num_observations;

  // setters for named parameter idiom
  Type & set__class_id(
    const uint16_t & _arg)
  {
    this->class_id = _arg;
    return *this;
  }
  Type & set__class_name(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->class_name = _arg;
    return *this;
  }
  Type & set__world_x(
    const float & _arg)
  {
    this->world_x = _arg;
    return *this;
  }
  Type & set__world_y(
    const float & _arg)
  {
    this->world_y = _arg;
    return *this;
  }
  Type & set__world_z(
    const float & _arg)
  {
    this->world_z = _arg;
    return *this;
  }
  Type & set__confidence(
    const float & _arg)
  {
    this->confidence = _arg;
    return *this;
  }
  Type & set__num_observations(
    const uint32_t & _arg)
  {
    this->num_observations = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    uv_msgs::msg::ObjectPosition_<ContainerAllocator> *;
  using ConstRawPtr =
    const uv_msgs::msg::ObjectPosition_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<uv_msgs::msg::ObjectPosition_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<uv_msgs::msg::ObjectPosition_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      uv_msgs::msg::ObjectPosition_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<uv_msgs::msg::ObjectPosition_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      uv_msgs::msg::ObjectPosition_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<uv_msgs::msg::ObjectPosition_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<uv_msgs::msg::ObjectPosition_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<uv_msgs::msg::ObjectPosition_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__uv_msgs__msg__ObjectPosition
    std::shared_ptr<uv_msgs::msg::ObjectPosition_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__uv_msgs__msg__ObjectPosition
    std::shared_ptr<uv_msgs::msg::ObjectPosition_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const ObjectPosition_ & other) const
  {
    if (this->class_id != other.class_id) {
      return false;
    }
    if (this->class_name != other.class_name) {
      return false;
    }
    if (this->world_x != other.world_x) {
      return false;
    }
    if (this->world_y != other.world_y) {
      return false;
    }
    if (this->world_z != other.world_z) {
      return false;
    }
    if (this->confidence != other.confidence) {
      return false;
    }
    if (this->num_observations != other.num_observations) {
      return false;
    }
    return true;
  }
  bool operator!=(const ObjectPosition_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct ObjectPosition_

// alias to use template instance with default allocator
using ObjectPosition =
  uv_msgs::msg::ObjectPosition_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace uv_msgs

#endif  // UV_MSGS__MSG__DETAIL__OBJECT_POSITION__STRUCT_HPP_
