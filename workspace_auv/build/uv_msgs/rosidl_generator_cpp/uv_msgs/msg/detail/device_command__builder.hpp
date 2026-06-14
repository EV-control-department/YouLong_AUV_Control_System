// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from uv_msgs:msg/DeviceCommand.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "uv_msgs/msg/device_command.hpp"


#ifndef UV_MSGS__MSG__DETAIL__DEVICE_COMMAND__BUILDER_HPP_
#define UV_MSGS__MSG__DETAIL__DEVICE_COMMAND__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "uv_msgs/msg/detail/device_command__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace uv_msgs
{

namespace msg
{

namespace builder
{

class Init_DeviceCommand_value
{
public:
  explicit Init_DeviceCommand_value(::uv_msgs::msg::DeviceCommand & msg)
  : msg_(msg)
  {}
  ::uv_msgs::msg::DeviceCommand value(::uv_msgs::msg::DeviceCommand::_value_type arg)
  {
    msg_.value = std::move(arg);
    return std::move(msg_);
  }

private:
  ::uv_msgs::msg::DeviceCommand msg_;
};

class Init_DeviceCommand_command
{
public:
  explicit Init_DeviceCommand_command(::uv_msgs::msg::DeviceCommand & msg)
  : msg_(msg)
  {}
  Init_DeviceCommand_value command(::uv_msgs::msg::DeviceCommand::_command_type arg)
  {
    msg_.command = std::move(arg);
    return Init_DeviceCommand_value(msg_);
  }

private:
  ::uv_msgs::msg::DeviceCommand msg_;
};

class Init_DeviceCommand_device_type
{
public:
  Init_DeviceCommand_device_type()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_DeviceCommand_command device_type(::uv_msgs::msg::DeviceCommand::_device_type_type arg)
  {
    msg_.device_type = std::move(arg);
    return Init_DeviceCommand_command(msg_);
  }

private:
  ::uv_msgs::msg::DeviceCommand msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::uv_msgs::msg::DeviceCommand>()
{
  return uv_msgs::msg::builder::Init_DeviceCommand_device_type();
}

}  // namespace uv_msgs

#endif  // UV_MSGS__MSG__DETAIL__DEVICE_COMMAND__BUILDER_HPP_
