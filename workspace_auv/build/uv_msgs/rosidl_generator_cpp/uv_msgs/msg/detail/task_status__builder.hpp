// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from uv_msgs:msg/TaskStatus.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "uv_msgs/msg/task_status.hpp"


#ifndef UV_MSGS__MSG__DETAIL__TASK_STATUS__BUILDER_HPP_
#define UV_MSGS__MSG__DETAIL__TASK_STATUS__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "uv_msgs/msg/detail/task_status__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace uv_msgs
{

namespace msg
{

namespace builder
{

class Init_TaskStatus_error_message
{
public:
  explicit Init_TaskStatus_error_message(::uv_msgs::msg::TaskStatus & msg)
  : msg_(msg)
  {}
  ::uv_msgs::msg::TaskStatus error_message(::uv_msgs::msg::TaskStatus::_error_message_type arg)
  {
    msg_.error_message = std::move(arg);
    return std::move(msg_);
  }

private:
  ::uv_msgs::msg::TaskStatus msg_;
};

class Init_TaskStatus_current_task_name
{
public:
  explicit Init_TaskStatus_current_task_name(::uv_msgs::msg::TaskStatus & msg)
  : msg_(msg)
  {}
  Init_TaskStatus_error_message current_task_name(::uv_msgs::msg::TaskStatus::_current_task_name_type arg)
  {
    msg_.current_task_name = std::move(arg);
    return Init_TaskStatus_error_message(msg_);
  }

private:
  ::uv_msgs::msg::TaskStatus msg_;
};

class Init_TaskStatus_total_tasks
{
public:
  explicit Init_TaskStatus_total_tasks(::uv_msgs::msg::TaskStatus & msg)
  : msg_(msg)
  {}
  Init_TaskStatus_current_task_name total_tasks(::uv_msgs::msg::TaskStatus::_total_tasks_type arg)
  {
    msg_.total_tasks = std::move(arg);
    return Init_TaskStatus_current_task_name(msg_);
  }

private:
  ::uv_msgs::msg::TaskStatus msg_;
};

class Init_TaskStatus_current_task_index
{
public:
  explicit Init_TaskStatus_current_task_index(::uv_msgs::msg::TaskStatus & msg)
  : msg_(msg)
  {}
  Init_TaskStatus_total_tasks current_task_index(::uv_msgs::msg::TaskStatus::_current_task_index_type arg)
  {
    msg_.current_task_index = std::move(arg);
    return Init_TaskStatus_total_tasks(msg_);
  }

private:
  ::uv_msgs::msg::TaskStatus msg_;
};

class Init_TaskStatus_status
{
public:
  Init_TaskStatus_status()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_TaskStatus_current_task_index status(::uv_msgs::msg::TaskStatus::_status_type arg)
  {
    msg_.status = std::move(arg);
    return Init_TaskStatus_current_task_index(msg_);
  }

private:
  ::uv_msgs::msg::TaskStatus msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::uv_msgs::msg::TaskStatus>()
{
  return uv_msgs::msg::builder::Init_TaskStatus_status();
}

}  // namespace uv_msgs

#endif  // UV_MSGS__MSG__DETAIL__TASK_STATUS__BUILDER_HPP_
