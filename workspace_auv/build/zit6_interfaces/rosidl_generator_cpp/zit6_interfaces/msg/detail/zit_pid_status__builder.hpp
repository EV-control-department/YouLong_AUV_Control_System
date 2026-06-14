// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from zit6_interfaces:msg/ZitPidStatus.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "zit6_interfaces/msg/zit_pid_status.hpp"


#ifndef ZIT6_INTERFACES__MSG__DETAIL__ZIT_PID_STATUS__BUILDER_HPP_
#define ZIT6_INTERFACES__MSG__DETAIL__ZIT_PID_STATUS__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "zit6_interfaces/msg/detail/zit_pid_status__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace zit6_interfaces
{

namespace msg
{

namespace builder
{

class Init_ZitPidStatus_vel_out_limit
{
public:
  explicit Init_ZitPidStatus_vel_out_limit(::zit6_interfaces::msg::ZitPidStatus & msg)
  : msg_(msg)
  {}
  ::zit6_interfaces::msg::ZitPidStatus vel_out_limit(::zit6_interfaces::msg::ZitPidStatus::_vel_out_limit_type arg)
  {
    msg_.vel_out_limit = std::move(arg);
    return std::move(msg_);
  }

private:
  ::zit6_interfaces::msg::ZitPidStatus msg_;
};

class Init_ZitPidStatus_vel_i_limit
{
public:
  explicit Init_ZitPidStatus_vel_i_limit(::zit6_interfaces::msg::ZitPidStatus & msg)
  : msg_(msg)
  {}
  Init_ZitPidStatus_vel_out_limit vel_i_limit(::zit6_interfaces::msg::ZitPidStatus::_vel_i_limit_type arg)
  {
    msg_.vel_i_limit = std::move(arg);
    return Init_ZitPidStatus_vel_out_limit(msg_);
  }

private:
  ::zit6_interfaces::msg::ZitPidStatus msg_;
};

class Init_ZitPidStatus_vel_kd
{
public:
  explicit Init_ZitPidStatus_vel_kd(::zit6_interfaces::msg::ZitPidStatus & msg)
  : msg_(msg)
  {}
  Init_ZitPidStatus_vel_i_limit vel_kd(::zit6_interfaces::msg::ZitPidStatus::_vel_kd_type arg)
  {
    msg_.vel_kd = std::move(arg);
    return Init_ZitPidStatus_vel_i_limit(msg_);
  }

private:
  ::zit6_interfaces::msg::ZitPidStatus msg_;
};

class Init_ZitPidStatus_vel_ki
{
public:
  explicit Init_ZitPidStatus_vel_ki(::zit6_interfaces::msg::ZitPidStatus & msg)
  : msg_(msg)
  {}
  Init_ZitPidStatus_vel_kd vel_ki(::zit6_interfaces::msg::ZitPidStatus::_vel_ki_type arg)
  {
    msg_.vel_ki = std::move(arg);
    return Init_ZitPidStatus_vel_kd(msg_);
  }

private:
  ::zit6_interfaces::msg::ZitPidStatus msg_;
};

class Init_ZitPidStatus_vel_kp
{
public:
  explicit Init_ZitPidStatus_vel_kp(::zit6_interfaces::msg::ZitPidStatus & msg)
  : msg_(msg)
  {}
  Init_ZitPidStatus_vel_ki vel_kp(::zit6_interfaces::msg::ZitPidStatus::_vel_kp_type arg)
  {
    msg_.vel_kp = std::move(arg);
    return Init_ZitPidStatus_vel_ki(msg_);
  }

private:
  ::zit6_interfaces::msg::ZitPidStatus msg_;
};

class Init_ZitPidStatus_pos_out_limit
{
public:
  explicit Init_ZitPidStatus_pos_out_limit(::zit6_interfaces::msg::ZitPidStatus & msg)
  : msg_(msg)
  {}
  Init_ZitPidStatus_vel_kp pos_out_limit(::zit6_interfaces::msg::ZitPidStatus::_pos_out_limit_type arg)
  {
    msg_.pos_out_limit = std::move(arg);
    return Init_ZitPidStatus_vel_kp(msg_);
  }

private:
  ::zit6_interfaces::msg::ZitPidStatus msg_;
};

class Init_ZitPidStatus_pos_max_a
{
public:
  explicit Init_ZitPidStatus_pos_max_a(::zit6_interfaces::msg::ZitPidStatus & msg)
  : msg_(msg)
  {}
  Init_ZitPidStatus_pos_out_limit pos_max_a(::zit6_interfaces::msg::ZitPidStatus::_pos_max_a_type arg)
  {
    msg_.pos_max_a = std::move(arg);
    return Init_ZitPidStatus_pos_out_limit(msg_);
  }

private:
  ::zit6_interfaces::msg::ZitPidStatus msg_;
};

class Init_ZitPidStatus_pos_max_v
{
public:
  explicit Init_ZitPidStatus_pos_max_v(::zit6_interfaces::msg::ZitPidStatus & msg)
  : msg_(msg)
  {}
  Init_ZitPidStatus_pos_max_a pos_max_v(::zit6_interfaces::msg::ZitPidStatus::_pos_max_v_type arg)
  {
    msg_.pos_max_v = std::move(arg);
    return Init_ZitPidStatus_pos_max_a(msg_);
  }

private:
  ::zit6_interfaces::msg::ZitPidStatus msg_;
};

class Init_ZitPidStatus_pos_kp
{
public:
  Init_ZitPidStatus_pos_kp()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_ZitPidStatus_pos_max_v pos_kp(::zit6_interfaces::msg::ZitPidStatus::_pos_kp_type arg)
  {
    msg_.pos_kp = std::move(arg);
    return Init_ZitPidStatus_pos_max_v(msg_);
  }

private:
  ::zit6_interfaces::msg::ZitPidStatus msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::zit6_interfaces::msg::ZitPidStatus>()
{
  return zit6_interfaces::msg::builder::Init_ZitPidStatus_pos_kp();
}

}  // namespace zit6_interfaces

#endif  // ZIT6_INTERFACES__MSG__DETAIL__ZIT_PID_STATUS__BUILDER_HPP_
