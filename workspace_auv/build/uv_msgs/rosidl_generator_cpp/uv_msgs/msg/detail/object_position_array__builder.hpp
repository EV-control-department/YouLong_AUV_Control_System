// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from uv_msgs:msg/ObjectPositionArray.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "uv_msgs/msg/object_position_array.hpp"


#ifndef UV_MSGS__MSG__DETAIL__OBJECT_POSITION_ARRAY__BUILDER_HPP_
#define UV_MSGS__MSG__DETAIL__OBJECT_POSITION_ARRAY__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "uv_msgs/msg/detail/object_position_array__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace uv_msgs
{

namespace msg
{

namespace builder
{

class Init_ObjectPositionArray_objects
{
public:
  explicit Init_ObjectPositionArray_objects(::uv_msgs::msg::ObjectPositionArray & msg)
  : msg_(msg)
  {}
  ::uv_msgs::msg::ObjectPositionArray objects(::uv_msgs::msg::ObjectPositionArray::_objects_type arg)
  {
    msg_.objects = std::move(arg);
    return std::move(msg_);
  }

private:
  ::uv_msgs::msg::ObjectPositionArray msg_;
};

class Init_ObjectPositionArray_header
{
public:
  Init_ObjectPositionArray_header()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_ObjectPositionArray_objects header(::uv_msgs::msg::ObjectPositionArray::_header_type arg)
  {
    msg_.header = std::move(arg);
    return Init_ObjectPositionArray_objects(msg_);
  }

private:
  ::uv_msgs::msg::ObjectPositionArray msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::uv_msgs::msg::ObjectPositionArray>()
{
  return uv_msgs::msg::builder::Init_ObjectPositionArray_header();
}

}  // namespace uv_msgs

#endif  // UV_MSGS__MSG__DETAIL__OBJECT_POSITION_ARRAY__BUILDER_HPP_
