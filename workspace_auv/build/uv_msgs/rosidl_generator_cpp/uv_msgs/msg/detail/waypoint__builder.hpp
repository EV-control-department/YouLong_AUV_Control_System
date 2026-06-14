// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from uv_msgs:msg/Waypoint.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "uv_msgs/msg/waypoint.hpp"


#ifndef UV_MSGS__MSG__DETAIL__WAYPOINT__BUILDER_HPP_
#define UV_MSGS__MSG__DETAIL__WAYPOINT__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "uv_msgs/msg/detail/waypoint__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace uv_msgs
{

namespace msg
{

namespace builder
{

class Init_Waypoint_speed
{
public:
  explicit Init_Waypoint_speed(::uv_msgs::msg::Waypoint & msg)
  : msg_(msg)
  {}
  ::uv_msgs::msg::Waypoint speed(::uv_msgs::msg::Waypoint::_speed_type arg)
  {
    msg_.speed = std::move(arg);
    return std::move(msg_);
  }

private:
  ::uv_msgs::msg::Waypoint msg_;
};

class Init_Waypoint_yaw
{
public:
  explicit Init_Waypoint_yaw(::uv_msgs::msg::Waypoint & msg)
  : msg_(msg)
  {}
  Init_Waypoint_speed yaw(::uv_msgs::msg::Waypoint::_yaw_type arg)
  {
    msg_.yaw = std::move(arg);
    return Init_Waypoint_speed(msg_);
  }

private:
  ::uv_msgs::msg::Waypoint msg_;
};

class Init_Waypoint_z
{
public:
  explicit Init_Waypoint_z(::uv_msgs::msg::Waypoint & msg)
  : msg_(msg)
  {}
  Init_Waypoint_yaw z(::uv_msgs::msg::Waypoint::_z_type arg)
  {
    msg_.z = std::move(arg);
    return Init_Waypoint_yaw(msg_);
  }

private:
  ::uv_msgs::msg::Waypoint msg_;
};

class Init_Waypoint_y
{
public:
  explicit Init_Waypoint_y(::uv_msgs::msg::Waypoint & msg)
  : msg_(msg)
  {}
  Init_Waypoint_z y(::uv_msgs::msg::Waypoint::_y_type arg)
  {
    msg_.y = std::move(arg);
    return Init_Waypoint_z(msg_);
  }

private:
  ::uv_msgs::msg::Waypoint msg_;
};

class Init_Waypoint_x
{
public:
  Init_Waypoint_x()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_Waypoint_y x(::uv_msgs::msg::Waypoint::_x_type arg)
  {
    msg_.x = std::move(arg);
    return Init_Waypoint_y(msg_);
  }

private:
  ::uv_msgs::msg::Waypoint msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::uv_msgs::msg::Waypoint>()
{
  return uv_msgs::msg::builder::Init_Waypoint_x();
}

}  // namespace uv_msgs

#endif  // UV_MSGS__MSG__DETAIL__WAYPOINT__BUILDER_HPP_
