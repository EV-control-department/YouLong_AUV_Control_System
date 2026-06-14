// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from uv_msgs:msg/WaypointPath.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "uv_msgs/msg/waypoint_path.hpp"


#ifndef UV_MSGS__MSG__DETAIL__WAYPOINT_PATH__BUILDER_HPP_
#define UV_MSGS__MSG__DETAIL__WAYPOINT_PATH__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "uv_msgs/msg/detail/waypoint_path__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace uv_msgs
{

namespace msg
{

namespace builder
{

class Init_WaypointPath_waypoints
{
public:
  explicit Init_WaypointPath_waypoints(::uv_msgs::msg::WaypointPath & msg)
  : msg_(msg)
  {}
  ::uv_msgs::msg::WaypointPath waypoints(::uv_msgs::msg::WaypointPath::_waypoints_type arg)
  {
    msg_.waypoints = std::move(arg);
    return std::move(msg_);
  }

private:
  ::uv_msgs::msg::WaypointPath msg_;
};

class Init_WaypointPath_header
{
public:
  Init_WaypointPath_header()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_WaypointPath_waypoints header(::uv_msgs::msg::WaypointPath::_header_type arg)
  {
    msg_.header = std::move(arg);
    return Init_WaypointPath_waypoints(msg_);
  }

private:
  ::uv_msgs::msg::WaypointPath msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::uv_msgs::msg::WaypointPath>()
{
  return uv_msgs::msg::builder::Init_WaypointPath_header();
}

}  // namespace uv_msgs

#endif  // UV_MSGS__MSG__DETAIL__WAYPOINT_PATH__BUILDER_HPP_
