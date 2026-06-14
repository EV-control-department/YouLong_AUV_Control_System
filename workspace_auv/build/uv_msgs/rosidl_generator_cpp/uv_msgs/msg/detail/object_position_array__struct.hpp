// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from uv_msgs:msg/ObjectPositionArray.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "uv_msgs/msg/object_position_array.hpp"


#ifndef UV_MSGS__MSG__DETAIL__OBJECT_POSITION_ARRAY__STRUCT_HPP_
#define UV_MSGS__MSG__DETAIL__OBJECT_POSITION_ARRAY__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__struct.hpp"
// Member 'objects'
#include "uv_msgs/msg/detail/object_position__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__uv_msgs__msg__ObjectPositionArray __attribute__((deprecated))
#else
# define DEPRECATED__uv_msgs__msg__ObjectPositionArray __declspec(deprecated)
#endif

namespace uv_msgs
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct ObjectPositionArray_
{
  using Type = ObjectPositionArray_<ContainerAllocator>;

  explicit ObjectPositionArray_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_init)
  {
    (void)_init;
  }

  explicit ObjectPositionArray_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_alloc, _init)
  {
    (void)_init;
  }

  // field types and members
  using _header_type =
    std_msgs::msg::Header_<ContainerAllocator>;
  _header_type header;
  using _objects_type =
    std::vector<uv_msgs::msg::ObjectPosition_<ContainerAllocator>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<uv_msgs::msg::ObjectPosition_<ContainerAllocator>>>;
  _objects_type objects;

  // setters for named parameter idiom
  Type & set__header(
    const std_msgs::msg::Header_<ContainerAllocator> & _arg)
  {
    this->header = _arg;
    return *this;
  }
  Type & set__objects(
    const std::vector<uv_msgs::msg::ObjectPosition_<ContainerAllocator>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<uv_msgs::msg::ObjectPosition_<ContainerAllocator>>> & _arg)
  {
    this->objects = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    uv_msgs::msg::ObjectPositionArray_<ContainerAllocator> *;
  using ConstRawPtr =
    const uv_msgs::msg::ObjectPositionArray_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<uv_msgs::msg::ObjectPositionArray_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<uv_msgs::msg::ObjectPositionArray_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      uv_msgs::msg::ObjectPositionArray_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<uv_msgs::msg::ObjectPositionArray_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      uv_msgs::msg::ObjectPositionArray_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<uv_msgs::msg::ObjectPositionArray_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<uv_msgs::msg::ObjectPositionArray_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<uv_msgs::msg::ObjectPositionArray_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__uv_msgs__msg__ObjectPositionArray
    std::shared_ptr<uv_msgs::msg::ObjectPositionArray_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__uv_msgs__msg__ObjectPositionArray
    std::shared_ptr<uv_msgs::msg::ObjectPositionArray_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const ObjectPositionArray_ & other) const
  {
    if (this->header != other.header) {
      return false;
    }
    if (this->objects != other.objects) {
      return false;
    }
    return true;
  }
  bool operator!=(const ObjectPositionArray_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct ObjectPositionArray_

// alias to use template instance with default allocator
using ObjectPositionArray =
  uv_msgs::msg::ObjectPositionArray_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace uv_msgs

#endif  // UV_MSGS__MSG__DETAIL__OBJECT_POSITION_ARRAY__STRUCT_HPP_
