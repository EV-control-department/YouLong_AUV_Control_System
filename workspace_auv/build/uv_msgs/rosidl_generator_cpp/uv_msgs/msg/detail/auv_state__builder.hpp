// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from uv_msgs:msg/AuvState.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "uv_msgs/msg/auv_state.hpp"


#ifndef UV_MSGS__MSG__DETAIL__AUV_STATE__BUILDER_HPP_
#define UV_MSGS__MSG__DETAIL__AUV_STATE__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "uv_msgs/msg/detail/auv_state__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace uv_msgs
{

namespace msg
{

namespace builder
{

class Init_AuvState_target_yaw
{
public:
  explicit Init_AuvState_target_yaw(::uv_msgs::msg::AuvState & msg)
  : msg_(msg)
  {}
  ::uv_msgs::msg::AuvState target_yaw(::uv_msgs::msg::AuvState::_target_yaw_type arg)
  {
    msg_.target_yaw = std::move(arg);
    return std::move(msg_);
  }

private:
  ::uv_msgs::msg::AuvState msg_;
};

class Init_AuvState_target_z
{
public:
  explicit Init_AuvState_target_z(::uv_msgs::msg::AuvState & msg)
  : msg_(msg)
  {}
  Init_AuvState_target_yaw target_z(::uv_msgs::msg::AuvState::_target_z_type arg)
  {
    msg_.target_z = std::move(arg);
    return Init_AuvState_target_yaw(msg_);
  }

private:
  ::uv_msgs::msg::AuvState msg_;
};

class Init_AuvState_target_y
{
public:
  explicit Init_AuvState_target_y(::uv_msgs::msg::AuvState & msg)
  : msg_(msg)
  {}
  Init_AuvState_target_z target_y(::uv_msgs::msg::AuvState::_target_y_type arg)
  {
    msg_.target_y = std::move(arg);
    return Init_AuvState_target_z(msg_);
  }

private:
  ::uv_msgs::msg::AuvState msg_;
};

class Init_AuvState_target_x
{
public:
  explicit Init_AuvState_target_x(::uv_msgs::msg::AuvState & msg)
  : msg_(msg)
  {}
  Init_AuvState_target_y target_x(::uv_msgs::msg::AuvState::_target_x_type arg)
  {
    msg_.target_x = std::move(arg);
    return Init_AuvState_target_y(msg_);
  }

private:
  ::uv_msgs::msg::AuvState msg_;
};

class Init_AuvState_cycle_time_ms
{
public:
  explicit Init_AuvState_cycle_time_ms(::uv_msgs::msg::AuvState & msg)
  : msg_(msg)
  {}
  Init_AuvState_target_x cycle_time_ms(::uv_msgs::msg::AuvState::_cycle_time_ms_type arg)
  {
    msg_.cycle_time_ms = std::move(arg);
    return Init_AuvState_target_x(msg_);
  }

private:
  ::uv_msgs::msg::AuvState msg_;
};

class Init_AuvState_error_flags
{
public:
  explicit Init_AuvState_error_flags(::uv_msgs::msg::AuvState & msg)
  : msg_(msg)
  {}
  Init_AuvState_cycle_time_ms error_flags(::uv_msgs::msg::AuvState::_error_flags_type arg)
  {
    msg_.error_flags = std::move(arg);
    return Init_AuvState_cycle_time_ms(msg_);
  }

private:
  ::uv_msgs::msg::AuvState msg_;
};

class Init_AuvState_battery_voltage
{
public:
  explicit Init_AuvState_battery_voltage(::uv_msgs::msg::AuvState & msg)
  : msg_(msg)
  {}
  Init_AuvState_error_flags battery_voltage(::uv_msgs::msg::AuvState::_battery_voltage_type arg)
  {
    msg_.battery_voltage = std::move(arg);
    return Init_AuvState_error_flags(msg_);
  }

private:
  ::uv_msgs::msg::AuvState msg_;
};

class Init_AuvState_control_mode
{
public:
  explicit Init_AuvState_control_mode(::uv_msgs::msg::AuvState & msg)
  : msg_(msg)
  {}
  Init_AuvState_battery_voltage control_mode(::uv_msgs::msg::AuvState::_control_mode_type arg)
  {
    msg_.control_mode = std::move(arg);
    return Init_AuvState_battery_voltage(msg_);
  }

private:
  ::uv_msgs::msg::AuvState msg_;
};

class Init_AuvState_armed
{
public:
  explicit Init_AuvState_armed(::uv_msgs::msg::AuvState & msg)
  : msg_(msg)
  {}
  Init_AuvState_control_mode armed(::uv_msgs::msg::AuvState::_armed_type arg)
  {
    msg_.armed = std::move(arg);
    return Init_AuvState_control_mode(msg_);
  }

private:
  ::uv_msgs::msg::AuvState msg_;
};

class Init_AuvState_vel_world_y
{
public:
  explicit Init_AuvState_vel_world_y(::uv_msgs::msg::AuvState & msg)
  : msg_(msg)
  {}
  Init_AuvState_armed vel_world_y(::uv_msgs::msg::AuvState::_vel_world_y_type arg)
  {
    msg_.vel_world_y = std::move(arg);
    return Init_AuvState_armed(msg_);
  }

private:
  ::uv_msgs::msg::AuvState msg_;
};

class Init_AuvState_vel_world_x
{
public:
  explicit Init_AuvState_vel_world_x(::uv_msgs::msg::AuvState & msg)
  : msg_(msg)
  {}
  Init_AuvState_vel_world_y vel_world_x(::uv_msgs::msg::AuvState::_vel_world_x_type arg)
  {
    msg_.vel_world_x = std::move(arg);
    return Init_AuvState_vel_world_y(msg_);
  }

private:
  ::uv_msgs::msg::AuvState msg_;
};

class Init_AuvState_yaw_rate
{
public:
  explicit Init_AuvState_yaw_rate(::uv_msgs::msg::AuvState & msg)
  : msg_(msg)
  {}
  Init_AuvState_vel_world_x yaw_rate(::uv_msgs::msg::AuvState::_yaw_rate_type arg)
  {
    msg_.yaw_rate = std::move(arg);
    return Init_AuvState_vel_world_x(msg_);
  }

private:
  ::uv_msgs::msg::AuvState msg_;
};

class Init_AuvState_vel_z
{
public:
  explicit Init_AuvState_vel_z(::uv_msgs::msg::AuvState & msg)
  : msg_(msg)
  {}
  Init_AuvState_yaw_rate vel_z(::uv_msgs::msg::AuvState::_vel_z_type arg)
  {
    msg_.vel_z = std::move(arg);
    return Init_AuvState_yaw_rate(msg_);
  }

private:
  ::uv_msgs::msg::AuvState msg_;
};

class Init_AuvState_vel_y
{
public:
  explicit Init_AuvState_vel_y(::uv_msgs::msg::AuvState & msg)
  : msg_(msg)
  {}
  Init_AuvState_vel_z vel_y(::uv_msgs::msg::AuvState::_vel_y_type arg)
  {
    msg_.vel_y = std::move(arg);
    return Init_AuvState_vel_z(msg_);
  }

private:
  ::uv_msgs::msg::AuvState msg_;
};

class Init_AuvState_vel_x
{
public:
  explicit Init_AuvState_vel_x(::uv_msgs::msg::AuvState & msg)
  : msg_(msg)
  {}
  Init_AuvState_vel_y vel_x(::uv_msgs::msg::AuvState::_vel_x_type arg)
  {
    msg_.vel_x = std::move(arg);
    return Init_AuvState_vel_y(msg_);
  }

private:
  ::uv_msgs::msg::AuvState msg_;
};

class Init_AuvState_yaw
{
public:
  explicit Init_AuvState_yaw(::uv_msgs::msg::AuvState & msg)
  : msg_(msg)
  {}
  Init_AuvState_vel_x yaw(::uv_msgs::msg::AuvState::_yaw_type arg)
  {
    msg_.yaw = std::move(arg);
    return Init_AuvState_vel_x(msg_);
  }

private:
  ::uv_msgs::msg::AuvState msg_;
};

class Init_AuvState_pos_z
{
public:
  explicit Init_AuvState_pos_z(::uv_msgs::msg::AuvState & msg)
  : msg_(msg)
  {}
  Init_AuvState_yaw pos_z(::uv_msgs::msg::AuvState::_pos_z_type arg)
  {
    msg_.pos_z = std::move(arg);
    return Init_AuvState_yaw(msg_);
  }

private:
  ::uv_msgs::msg::AuvState msg_;
};

class Init_AuvState_pos_y
{
public:
  explicit Init_AuvState_pos_y(::uv_msgs::msg::AuvState & msg)
  : msg_(msg)
  {}
  Init_AuvState_pos_z pos_y(::uv_msgs::msg::AuvState::_pos_y_type arg)
  {
    msg_.pos_y = std::move(arg);
    return Init_AuvState_pos_z(msg_);
  }

private:
  ::uv_msgs::msg::AuvState msg_;
};

class Init_AuvState_pos_x
{
public:
  Init_AuvState_pos_x()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_AuvState_pos_y pos_x(::uv_msgs::msg::AuvState::_pos_x_type arg)
  {
    msg_.pos_x = std::move(arg);
    return Init_AuvState_pos_y(msg_);
  }

private:
  ::uv_msgs::msg::AuvState msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::uv_msgs::msg::AuvState>()
{
  return uv_msgs::msg::builder::Init_AuvState_pos_x();
}

}  // namespace uv_msgs

#endif  // UV_MSGS__MSG__DETAIL__AUV_STATE__BUILDER_HPP_
