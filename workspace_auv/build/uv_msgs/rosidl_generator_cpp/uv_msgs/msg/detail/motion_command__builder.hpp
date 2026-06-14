// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from uv_msgs:msg/MotionCommand.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "uv_msgs/msg/motion_command.hpp"


#ifndef UV_MSGS__MSG__DETAIL__MOTION_COMMAND__BUILDER_HPP_
#define UV_MSGS__MSG__DETAIL__MOTION_COMMAND__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "uv_msgs/msg/detail/motion_command__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace uv_msgs
{

namespace msg
{

namespace builder
{

class Init_MotionCommand_yaw
{
public:
  explicit Init_MotionCommand_yaw(::uv_msgs::msg::MotionCommand & msg)
  : msg_(msg)
  {}
  ::uv_msgs::msg::MotionCommand yaw(::uv_msgs::msg::MotionCommand::_yaw_type arg)
  {
    msg_.yaw = std::move(arg);
    return std::move(msg_);
  }

private:
  ::uv_msgs::msg::MotionCommand msg_;
};

class Init_MotionCommand_z
{
public:
  explicit Init_MotionCommand_z(::uv_msgs::msg::MotionCommand & msg)
  : msg_(msg)
  {}
  Init_MotionCommand_yaw z(::uv_msgs::msg::MotionCommand::_z_type arg)
  {
    msg_.z = std::move(arg);
    return Init_MotionCommand_yaw(msg_);
  }

private:
  ::uv_msgs::msg::MotionCommand msg_;
};

class Init_MotionCommand_y
{
public:
  explicit Init_MotionCommand_y(::uv_msgs::msg::MotionCommand & msg)
  : msg_(msg)
  {}
  Init_MotionCommand_z y(::uv_msgs::msg::MotionCommand::_y_type arg)
  {
    msg_.y = std::move(arg);
    return Init_MotionCommand_z(msg_);
  }

private:
  ::uv_msgs::msg::MotionCommand msg_;
};

class Init_MotionCommand_x
{
public:
  explicit Init_MotionCommand_x(::uv_msgs::msg::MotionCommand & msg)
  : msg_(msg)
  {}
  Init_MotionCommand_y x(::uv_msgs::msg::MotionCommand::_x_type arg)
  {
    msg_.x = std::move(arg);
    return Init_MotionCommand_y(msg_);
  }

private:
  ::uv_msgs::msg::MotionCommand msg_;
};

class Init_MotionCommand_type_mask
{
public:
  explicit Init_MotionCommand_type_mask(::uv_msgs::msg::MotionCommand & msg)
  : msg_(msg)
  {}
  Init_MotionCommand_x type_mask(::uv_msgs::msg::MotionCommand::_type_mask_type arg)
  {
    msg_.type_mask = std::move(arg);
    return Init_MotionCommand_x(msg_);
  }

private:
  ::uv_msgs::msg::MotionCommand msg_;
};

class Init_MotionCommand_mode
{
public:
  Init_MotionCommand_mode()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_MotionCommand_type_mask mode(::uv_msgs::msg::MotionCommand::_mode_type arg)
  {
    msg_.mode = std::move(arg);
    return Init_MotionCommand_type_mask(msg_);
  }

private:
  ::uv_msgs::msg::MotionCommand msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::uv_msgs::msg::MotionCommand>()
{
  return uv_msgs::msg::builder::Init_MotionCommand_mode();
}

}  // namespace uv_msgs

#endif  // UV_MSGS__MSG__DETAIL__MOTION_COMMAND__BUILDER_HPP_
