// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from uv_msgs:msg/ObjectPosition.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "uv_msgs/msg/object_position.hpp"


#ifndef UV_MSGS__MSG__DETAIL__OBJECT_POSITION__BUILDER_HPP_
#define UV_MSGS__MSG__DETAIL__OBJECT_POSITION__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "uv_msgs/msg/detail/object_position__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace uv_msgs
{

namespace msg
{

namespace builder
{

class Init_ObjectPosition_num_observations
{
public:
  explicit Init_ObjectPosition_num_observations(::uv_msgs::msg::ObjectPosition & msg)
  : msg_(msg)
  {}
  ::uv_msgs::msg::ObjectPosition num_observations(::uv_msgs::msg::ObjectPosition::_num_observations_type arg)
  {
    msg_.num_observations = std::move(arg);
    return std::move(msg_);
  }

private:
  ::uv_msgs::msg::ObjectPosition msg_;
};

class Init_ObjectPosition_confidence
{
public:
  explicit Init_ObjectPosition_confidence(::uv_msgs::msg::ObjectPosition & msg)
  : msg_(msg)
  {}
  Init_ObjectPosition_num_observations confidence(::uv_msgs::msg::ObjectPosition::_confidence_type arg)
  {
    msg_.confidence = std::move(arg);
    return Init_ObjectPosition_num_observations(msg_);
  }

private:
  ::uv_msgs::msg::ObjectPosition msg_;
};

class Init_ObjectPosition_world_z
{
public:
  explicit Init_ObjectPosition_world_z(::uv_msgs::msg::ObjectPosition & msg)
  : msg_(msg)
  {}
  Init_ObjectPosition_confidence world_z(::uv_msgs::msg::ObjectPosition::_world_z_type arg)
  {
    msg_.world_z = std::move(arg);
    return Init_ObjectPosition_confidence(msg_);
  }

private:
  ::uv_msgs::msg::ObjectPosition msg_;
};

class Init_ObjectPosition_world_y
{
public:
  explicit Init_ObjectPosition_world_y(::uv_msgs::msg::ObjectPosition & msg)
  : msg_(msg)
  {}
  Init_ObjectPosition_world_z world_y(::uv_msgs::msg::ObjectPosition::_world_y_type arg)
  {
    msg_.world_y = std::move(arg);
    return Init_ObjectPosition_world_z(msg_);
  }

private:
  ::uv_msgs::msg::ObjectPosition msg_;
};

class Init_ObjectPosition_world_x
{
public:
  explicit Init_ObjectPosition_world_x(::uv_msgs::msg::ObjectPosition & msg)
  : msg_(msg)
  {}
  Init_ObjectPosition_world_y world_x(::uv_msgs::msg::ObjectPosition::_world_x_type arg)
  {
    msg_.world_x = std::move(arg);
    return Init_ObjectPosition_world_y(msg_);
  }

private:
  ::uv_msgs::msg::ObjectPosition msg_;
};

class Init_ObjectPosition_class_name
{
public:
  explicit Init_ObjectPosition_class_name(::uv_msgs::msg::ObjectPosition & msg)
  : msg_(msg)
  {}
  Init_ObjectPosition_world_x class_name(::uv_msgs::msg::ObjectPosition::_class_name_type arg)
  {
    msg_.class_name = std::move(arg);
    return Init_ObjectPosition_world_x(msg_);
  }

private:
  ::uv_msgs::msg::ObjectPosition msg_;
};

class Init_ObjectPosition_class_id
{
public:
  Init_ObjectPosition_class_id()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_ObjectPosition_class_name class_id(::uv_msgs::msg::ObjectPosition::_class_id_type arg)
  {
    msg_.class_id = std::move(arg);
    return Init_ObjectPosition_class_name(msg_);
  }

private:
  ::uv_msgs::msg::ObjectPosition msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::uv_msgs::msg::ObjectPosition>()
{
  return uv_msgs::msg::builder::Init_ObjectPosition_class_id();
}

}  // namespace uv_msgs

#endif  // UV_MSGS__MSG__DETAIL__OBJECT_POSITION__BUILDER_HPP_
