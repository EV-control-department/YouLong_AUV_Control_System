// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from zit6_interfaces:msg/ZitPid.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "zit6_interfaces/msg/zit_pid.hpp"


#ifndef ZIT6_INTERFACES__MSG__DETAIL__ZIT_PID__BUILDER_HPP_
#define ZIT6_INTERFACES__MSG__DETAIL__ZIT_PID__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "zit6_interfaces/msg/detail/zit_pid__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace zit6_interfaces
{

namespace msg
{

namespace builder
{

class Init_ZitPid_max_a
{
public:
  explicit Init_ZitPid_max_a(::zit6_interfaces::msg::ZitPid & msg)
  : msg_(msg)
  {}
  ::zit6_interfaces::msg::ZitPid max_a(::zit6_interfaces::msg::ZitPid::_max_a_type arg)
  {
    msg_.max_a = std::move(arg);
    return std::move(msg_);
  }

private:
  ::zit6_interfaces::msg::ZitPid msg_;
};

class Init_ZitPid_max_v
{
public:
  explicit Init_ZitPid_max_v(::zit6_interfaces::msg::ZitPid & msg)
  : msg_(msg)
  {}
  Init_ZitPid_max_a max_v(::zit6_interfaces::msg::ZitPid::_max_v_type arg)
  {
    msg_.max_v = std::move(arg);
    return Init_ZitPid_max_a(msg_);
  }

private:
  ::zit6_interfaces::msg::ZitPid msg_;
};

class Init_ZitPid_out_limit
{
public:
  explicit Init_ZitPid_out_limit(::zit6_interfaces::msg::ZitPid & msg)
  : msg_(msg)
  {}
  Init_ZitPid_max_v out_limit(::zit6_interfaces::msg::ZitPid::_out_limit_type arg)
  {
    msg_.out_limit = std::move(arg);
    return Init_ZitPid_max_v(msg_);
  }

private:
  ::zit6_interfaces::msg::ZitPid msg_;
};

class Init_ZitPid_i_limit
{
public:
  explicit Init_ZitPid_i_limit(::zit6_interfaces::msg::ZitPid & msg)
  : msg_(msg)
  {}
  Init_ZitPid_out_limit i_limit(::zit6_interfaces::msg::ZitPid::_i_limit_type arg)
  {
    msg_.i_limit = std::move(arg);
    return Init_ZitPid_out_limit(msg_);
  }

private:
  ::zit6_interfaces::msg::ZitPid msg_;
};

class Init_ZitPid_kd
{
public:
  explicit Init_ZitPid_kd(::zit6_interfaces::msg::ZitPid & msg)
  : msg_(msg)
  {}
  Init_ZitPid_i_limit kd(::zit6_interfaces::msg::ZitPid::_kd_type arg)
  {
    msg_.kd = std::move(arg);
    return Init_ZitPid_i_limit(msg_);
  }

private:
  ::zit6_interfaces::msg::ZitPid msg_;
};

class Init_ZitPid_ki
{
public:
  explicit Init_ZitPid_ki(::zit6_interfaces::msg::ZitPid & msg)
  : msg_(msg)
  {}
  Init_ZitPid_kd ki(::zit6_interfaces::msg::ZitPid::_ki_type arg)
  {
    msg_.ki = std::move(arg);
    return Init_ZitPid_kd(msg_);
  }

private:
  ::zit6_interfaces::msg::ZitPid msg_;
};

class Init_ZitPid_kp
{
public:
  explicit Init_ZitPid_kp(::zit6_interfaces::msg::ZitPid & msg)
  : msg_(msg)
  {}
  Init_ZitPid_ki kp(::zit6_interfaces::msg::ZitPid::_kp_type arg)
  {
    msg_.kp = std::move(arg);
    return Init_ZitPid_ki(msg_);
  }

private:
  ::zit6_interfaces::msg::ZitPid msg_;
};

class Init_ZitPid_is_pos_ring
{
public:
  explicit Init_ZitPid_is_pos_ring(::zit6_interfaces::msg::ZitPid & msg)
  : msg_(msg)
  {}
  Init_ZitPid_kp is_pos_ring(::zit6_interfaces::msg::ZitPid::_is_pos_ring_type arg)
  {
    msg_.is_pos_ring = std::move(arg);
    return Init_ZitPid_kp(msg_);
  }

private:
  ::zit6_interfaces::msg::ZitPid msg_;
};

class Init_ZitPid_axis
{
public:
  Init_ZitPid_axis()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_ZitPid_is_pos_ring axis(::zit6_interfaces::msg::ZitPid::_axis_type arg)
  {
    msg_.axis = std::move(arg);
    return Init_ZitPid_is_pos_ring(msg_);
  }

private:
  ::zit6_interfaces::msg::ZitPid msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::zit6_interfaces::msg::ZitPid>()
{
  return zit6_interfaces::msg::builder::Init_ZitPid_axis();
}

}  // namespace zit6_interfaces

#endif  // ZIT6_INTERFACES__MSG__DETAIL__ZIT_PID__BUILDER_HPP_
