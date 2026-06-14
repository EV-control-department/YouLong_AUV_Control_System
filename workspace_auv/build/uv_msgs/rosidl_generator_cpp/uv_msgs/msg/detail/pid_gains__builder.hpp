// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from uv_msgs:msg/PidGains.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "uv_msgs/msg/pid_gains.hpp"


#ifndef UV_MSGS__MSG__DETAIL__PID_GAINS__BUILDER_HPP_
#define UV_MSGS__MSG__DETAIL__PID_GAINS__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "uv_msgs/msg/detail/pid_gains__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace uv_msgs
{

namespace msg
{

namespace builder
{

class Init_PidGains_feedforward
{
public:
  explicit Init_PidGains_feedforward(::uv_msgs::msg::PidGains & msg)
  : msg_(msg)
  {}
  ::uv_msgs::msg::PidGains feedforward(::uv_msgs::msg::PidGains::_feedforward_type arg)
  {
    msg_.feedforward = std::move(arg);
    return std::move(msg_);
  }

private:
  ::uv_msgs::msg::PidGains msg_;
};

class Init_PidGains_output_limit
{
public:
  explicit Init_PidGains_output_limit(::uv_msgs::msg::PidGains & msg)
  : msg_(msg)
  {}
  Init_PidGains_feedforward output_limit(::uv_msgs::msg::PidGains::_output_limit_type arg)
  {
    msg_.output_limit = std::move(arg);
    return Init_PidGains_feedforward(msg_);
  }

private:
  ::uv_msgs::msg::PidGains msg_;
};

class Init_PidGains_i_limit
{
public:
  explicit Init_PidGains_i_limit(::uv_msgs::msg::PidGains & msg)
  : msg_(msg)
  {}
  Init_PidGains_output_limit i_limit(::uv_msgs::msg::PidGains::_i_limit_type arg)
  {
    msg_.i_limit = std::move(arg);
    return Init_PidGains_output_limit(msg_);
  }

private:
  ::uv_msgs::msg::PidGains msg_;
};

class Init_PidGains_kd
{
public:
  explicit Init_PidGains_kd(::uv_msgs::msg::PidGains & msg)
  : msg_(msg)
  {}
  Init_PidGains_i_limit kd(::uv_msgs::msg::PidGains::_kd_type arg)
  {
    msg_.kd = std::move(arg);
    return Init_PidGains_i_limit(msg_);
  }

private:
  ::uv_msgs::msg::PidGains msg_;
};

class Init_PidGains_ki
{
public:
  explicit Init_PidGains_ki(::uv_msgs::msg::PidGains & msg)
  : msg_(msg)
  {}
  Init_PidGains_kd ki(::uv_msgs::msg::PidGains::_ki_type arg)
  {
    msg_.ki = std::move(arg);
    return Init_PidGains_kd(msg_);
  }

private:
  ::uv_msgs::msg::PidGains msg_;
};

class Init_PidGains_kp
{
public:
  explicit Init_PidGains_kp(::uv_msgs::msg::PidGains & msg)
  : msg_(msg)
  {}
  Init_PidGains_ki kp(::uv_msgs::msg::PidGains::_kp_type arg)
  {
    msg_.kp = std::move(arg);
    return Init_PidGains_ki(msg_);
  }

private:
  ::uv_msgs::msg::PidGains msg_;
};

class Init_PidGains_name
{
public:
  Init_PidGains_name()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_PidGains_kp name(::uv_msgs::msg::PidGains::_name_type arg)
  {
    msg_.name = std::move(arg);
    return Init_PidGains_kp(msg_);
  }

private:
  ::uv_msgs::msg::PidGains msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::uv_msgs::msg::PidGains>()
{
  return uv_msgs::msg::builder::Init_PidGains_name();
}

}  // namespace uv_msgs

#endif  // UV_MSGS__MSG__DETAIL__PID_GAINS__BUILDER_HPP_
