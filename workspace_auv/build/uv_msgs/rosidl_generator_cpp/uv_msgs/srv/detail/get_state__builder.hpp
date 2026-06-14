// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from uv_msgs:srv/GetState.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "uv_msgs/srv/get_state.hpp"


#ifndef UV_MSGS__SRV__DETAIL__GET_STATE__BUILDER_HPP_
#define UV_MSGS__SRV__DETAIL__GET_STATE__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "uv_msgs/srv/detail/get_state__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace uv_msgs
{

namespace srv
{


}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::uv_msgs::srv::GetState_Request>()
{
  return ::uv_msgs::srv::GetState_Request(rosidl_runtime_cpp::MessageInitialization::ZERO);
}

}  // namespace uv_msgs


namespace uv_msgs
{

namespace srv
{

namespace builder
{

class Init_GetState_Response_state
{
public:
  Init_GetState_Response_state()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::uv_msgs::srv::GetState_Response state(::uv_msgs::srv::GetState_Response::_state_type arg)
  {
    msg_.state = std::move(arg);
    return std::move(msg_);
  }

private:
  ::uv_msgs::srv::GetState_Response msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::uv_msgs::srv::GetState_Response>()
{
  return uv_msgs::srv::builder::Init_GetState_Response_state();
}

}  // namespace uv_msgs


namespace uv_msgs
{

namespace srv
{

namespace builder
{

class Init_GetState_Event_response
{
public:
  explicit Init_GetState_Event_response(::uv_msgs::srv::GetState_Event & msg)
  : msg_(msg)
  {}
  ::uv_msgs::srv::GetState_Event response(::uv_msgs::srv::GetState_Event::_response_type arg)
  {
    msg_.response = std::move(arg);
    return std::move(msg_);
  }

private:
  ::uv_msgs::srv::GetState_Event msg_;
};

class Init_GetState_Event_request
{
public:
  explicit Init_GetState_Event_request(::uv_msgs::srv::GetState_Event & msg)
  : msg_(msg)
  {}
  Init_GetState_Event_response request(::uv_msgs::srv::GetState_Event::_request_type arg)
  {
    msg_.request = std::move(arg);
    return Init_GetState_Event_response(msg_);
  }

private:
  ::uv_msgs::srv::GetState_Event msg_;
};

class Init_GetState_Event_info
{
public:
  Init_GetState_Event_info()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_GetState_Event_request info(::uv_msgs::srv::GetState_Event::_info_type arg)
  {
    msg_.info = std::move(arg);
    return Init_GetState_Event_request(msg_);
  }

private:
  ::uv_msgs::srv::GetState_Event msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::uv_msgs::srv::GetState_Event>()
{
  return uv_msgs::srv::builder::Init_GetState_Event_info();
}

}  // namespace uv_msgs

#endif  // UV_MSGS__SRV__DETAIL__GET_STATE__BUILDER_HPP_
