// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from uv_msgs:srv/RunTask.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "uv_msgs/srv/run_task.hpp"


#ifndef UV_MSGS__SRV__DETAIL__RUN_TASK__BUILDER_HPP_
#define UV_MSGS__SRV__DETAIL__RUN_TASK__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "uv_msgs/srv/detail/run_task__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace uv_msgs
{

namespace srv
{

namespace builder
{

class Init_RunTask_Request_start
{
public:
  explicit Init_RunTask_Request_start(::uv_msgs::srv::RunTask_Request & msg)
  : msg_(msg)
  {}
  ::uv_msgs::srv::RunTask_Request start(::uv_msgs::srv::RunTask_Request::_start_type arg)
  {
    msg_.start = std::move(arg);
    return std::move(msg_);
  }

private:
  ::uv_msgs::srv::RunTask_Request msg_;
};

class Init_RunTask_Request_task_name
{
public:
  Init_RunTask_Request_task_name()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_RunTask_Request_start task_name(::uv_msgs::srv::RunTask_Request::_task_name_type arg)
  {
    msg_.task_name = std::move(arg);
    return Init_RunTask_Request_start(msg_);
  }

private:
  ::uv_msgs::srv::RunTask_Request msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::uv_msgs::srv::RunTask_Request>()
{
  return uv_msgs::srv::builder::Init_RunTask_Request_task_name();
}

}  // namespace uv_msgs


namespace uv_msgs
{

namespace srv
{

namespace builder
{

class Init_RunTask_Response_message
{
public:
  explicit Init_RunTask_Response_message(::uv_msgs::srv::RunTask_Response & msg)
  : msg_(msg)
  {}
  ::uv_msgs::srv::RunTask_Response message(::uv_msgs::srv::RunTask_Response::_message_type arg)
  {
    msg_.message = std::move(arg);
    return std::move(msg_);
  }

private:
  ::uv_msgs::srv::RunTask_Response msg_;
};

class Init_RunTask_Response_success
{
public:
  Init_RunTask_Response_success()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_RunTask_Response_message success(::uv_msgs::srv::RunTask_Response::_success_type arg)
  {
    msg_.success = std::move(arg);
    return Init_RunTask_Response_message(msg_);
  }

private:
  ::uv_msgs::srv::RunTask_Response msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::uv_msgs::srv::RunTask_Response>()
{
  return uv_msgs::srv::builder::Init_RunTask_Response_success();
}

}  // namespace uv_msgs


namespace uv_msgs
{

namespace srv
{

namespace builder
{

class Init_RunTask_Event_response
{
public:
  explicit Init_RunTask_Event_response(::uv_msgs::srv::RunTask_Event & msg)
  : msg_(msg)
  {}
  ::uv_msgs::srv::RunTask_Event response(::uv_msgs::srv::RunTask_Event::_response_type arg)
  {
    msg_.response = std::move(arg);
    return std::move(msg_);
  }

private:
  ::uv_msgs::srv::RunTask_Event msg_;
};

class Init_RunTask_Event_request
{
public:
  explicit Init_RunTask_Event_request(::uv_msgs::srv::RunTask_Event & msg)
  : msg_(msg)
  {}
  Init_RunTask_Event_response request(::uv_msgs::srv::RunTask_Event::_request_type arg)
  {
    msg_.request = std::move(arg);
    return Init_RunTask_Event_response(msg_);
  }

private:
  ::uv_msgs::srv::RunTask_Event msg_;
};

class Init_RunTask_Event_info
{
public:
  Init_RunTask_Event_info()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_RunTask_Event_request info(::uv_msgs::srv::RunTask_Event::_info_type arg)
  {
    msg_.info = std::move(arg);
    return Init_RunTask_Event_request(msg_);
  }

private:
  ::uv_msgs::srv::RunTask_Event msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::uv_msgs::srv::RunTask_Event>()
{
  return uv_msgs::srv::builder::Init_RunTask_Event_info();
}

}  // namespace uv_msgs

#endif  // UV_MSGS__SRV__DETAIL__RUN_TASK__BUILDER_HPP_
