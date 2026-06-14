// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from uv_msgs:msg/PidState.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "uv_msgs/msg/pid_state.hpp"


#ifndef UV_MSGS__MSG__DETAIL__PID_STATE__BUILDER_HPP_
#define UV_MSGS__MSG__DETAIL__PID_STATE__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "uv_msgs/msg/detail/pid_state__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace uv_msgs
{

namespace msg
{

namespace builder
{

class Init_PidState_vy_body_cmd
{
public:
  explicit Init_PidState_vy_body_cmd(::uv_msgs::msg::PidState & msg)
  : msg_(msg)
  {}
  ::uv_msgs::msg::PidState vy_body_cmd(::uv_msgs::msg::PidState::_vy_body_cmd_type arg)
  {
    msg_.vy_body_cmd = std::move(arg);
    return std::move(msg_);
  }

private:
  ::uv_msgs::msg::PidState msg_;
};

class Init_PidState_vx_body_cmd
{
public:
  explicit Init_PidState_vx_body_cmd(::uv_msgs::msg::PidState & msg)
  : msg_(msg)
  {}
  Init_PidState_vy_body_cmd vx_body_cmd(::uv_msgs::msg::PidState::_vx_body_cmd_type arg)
  {
    msg_.vx_body_cmd = std::move(arg);
    return Init_PidState_vy_body_cmd(msg_);
  }

private:
  ::uv_msgs::msg::PidState msg_;
};

class Init_PidState_speed_cmd
{
public:
  explicit Init_PidState_speed_cmd(::uv_msgs::msg::PidState & msg)
  : msg_(msg)
  {}
  Init_PidState_vx_body_cmd speed_cmd(::uv_msgs::msg::PidState::_speed_cmd_type arg)
  {
    msg_.speed_cmd = std::move(arg);
    return Init_PidState_vx_body_cmd(msg_);
  }

private:
  ::uv_msgs::msg::PidState msg_;
};

class Init_PidState_error_angle_deg
{
public:
  explicit Init_PidState_error_angle_deg(::uv_msgs::msg::PidState & msg)
  : msg_(msg)
  {}
  Init_PidState_speed_cmd error_angle_deg(::uv_msgs::msg::PidState::_error_angle_deg_type arg)
  {
    msg_.error_angle_deg = std::move(arg);
    return Init_PidState_speed_cmd(msg_);
  }

private:
  ::uv_msgs::msg::PidState msg_;
};

class Init_PidState_error_length
{
public:
  explicit Init_PidState_error_length(::uv_msgs::msg::PidState & msg)
  : msg_(msg)
  {}
  Init_PidState_error_angle_deg error_length(::uv_msgs::msg::PidState::_error_length_type arg)
  {
    msg_.error_length = std::move(arg);
    return Init_PidState_error_angle_deg(msg_);
  }

private:
  ::uv_msgs::msg::PidState msg_;
};

class Init_PidState_yaw_rate
{
public:
  explicit Init_PidState_yaw_rate(::uv_msgs::msg::PidState & msg)
  : msg_(msg)
  {}
  Init_PidState_error_length yaw_rate(::uv_msgs::msg::PidState::_yaw_rate_type arg)
  {
    msg_.yaw_rate = std::move(arg);
    return Init_PidState_error_length(msg_);
  }

private:
  ::uv_msgs::msg::PidState msg_;
};

class Init_PidState_yaw
{
public:
  explicit Init_PidState_yaw(::uv_msgs::msg::PidState & msg)
  : msg_(msg)
  {}
  Init_PidState_yaw_rate yaw(::uv_msgs::msg::PidState::_yaw_type arg)
  {
    msg_.yaw = std::move(arg);
    return Init_PidState_yaw_rate(msg_);
  }

private:
  ::uv_msgs::msg::PidState msg_;
};

class Init_PidState_pos_z
{
public:
  explicit Init_PidState_pos_z(::uv_msgs::msg::PidState & msg)
  : msg_(msg)
  {}
  Init_PidState_yaw pos_z(::uv_msgs::msg::PidState::_pos_z_type arg)
  {
    msg_.pos_z = std::move(arg);
    return Init_PidState_yaw(msg_);
  }

private:
  ::uv_msgs::msg::PidState msg_;
};

class Init_PidState_vel_y
{
public:
  explicit Init_PidState_vel_y(::uv_msgs::msg::PidState & msg)
  : msg_(msg)
  {}
  Init_PidState_pos_z vel_y(::uv_msgs::msg::PidState::_vel_y_type arg)
  {
    msg_.vel_y = std::move(arg);
    return Init_PidState_pos_z(msg_);
  }

private:
  ::uv_msgs::msg::PidState msg_;
};

class Init_PidState_vel_x
{
public:
  explicit Init_PidState_vel_x(::uv_msgs::msg::PidState & msg)
  : msg_(msg)
  {}
  Init_PidState_vel_y vel_x(::uv_msgs::msg::PidState::_vel_x_type arg)
  {
    msg_.vel_x = std::move(arg);
    return Init_PidState_vel_y(msg_);
  }

private:
  ::uv_msgs::msg::PidState msg_;
};

class Init_PidState_speed_mag
{
public:
  Init_PidState_speed_mag()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_PidState_vel_x speed_mag(::uv_msgs::msg::PidState::_speed_mag_type arg)
  {
    msg_.speed_mag = std::move(arg);
    return Init_PidState_vel_x(msg_);
  }

private:
  ::uv_msgs::msg::PidState msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::uv_msgs::msg::PidState>()
{
  return uv_msgs::msg::builder::Init_PidState_speed_mag();
}

}  // namespace uv_msgs

#endif  // UV_MSGS__MSG__DETAIL__PID_STATE__BUILDER_HPP_
