// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from uv_msgs:action/BasicMotion.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "uv_msgs/action/basic_motion.hpp"


#ifndef UV_MSGS__ACTION__DETAIL__BASIC_MOTION__BUILDER_HPP_
#define UV_MSGS__ACTION__DETAIL__BASIC_MOTION__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "uv_msgs/action/detail/basic_motion__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace uv_msgs
{

namespace action
{

namespace builder
{

class Init_BasicMotion_Goal_timeout
{
public:
  explicit Init_BasicMotion_Goal_timeout(::uv_msgs::action::BasicMotion_Goal & msg)
  : msg_(msg)
  {}
  ::uv_msgs::action::BasicMotion_Goal timeout(::uv_msgs::action::BasicMotion_Goal::_timeout_type arg)
  {
    msg_.timeout = std::move(arg);
    return std::move(msg_);
  }

private:
  ::uv_msgs::action::BasicMotion_Goal msg_;
};

class Init_BasicMotion_Goal_target
{
public:
  explicit Init_BasicMotion_Goal_target(::uv_msgs::action::BasicMotion_Goal & msg)
  : msg_(msg)
  {}
  Init_BasicMotion_Goal_timeout target(::uv_msgs::action::BasicMotion_Goal::_target_type arg)
  {
    msg_.target = std::move(arg);
    return Init_BasicMotion_Goal_timeout(msg_);
  }

private:
  ::uv_msgs::action::BasicMotion_Goal msg_;
};

class Init_BasicMotion_Goal_axes
{
public:
  explicit Init_BasicMotion_Goal_axes(::uv_msgs::action::BasicMotion_Goal & msg)
  : msg_(msg)
  {}
  Init_BasicMotion_Goal_target axes(::uv_msgs::action::BasicMotion_Goal::_axes_type arg)
  {
    msg_.axes = std::move(arg);
    return Init_BasicMotion_Goal_target(msg_);
  }

private:
  ::uv_msgs::action::BasicMotion_Goal msg_;
};

class Init_BasicMotion_Goal_cmd_type
{
public:
  Init_BasicMotion_Goal_cmd_type()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_BasicMotion_Goal_axes cmd_type(::uv_msgs::action::BasicMotion_Goal::_cmd_type_type arg)
  {
    msg_.cmd_type = std::move(arg);
    return Init_BasicMotion_Goal_axes(msg_);
  }

private:
  ::uv_msgs::action::BasicMotion_Goal msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::uv_msgs::action::BasicMotion_Goal>()
{
  return uv_msgs::action::builder::Init_BasicMotion_Goal_cmd_type();
}

}  // namespace uv_msgs


namespace uv_msgs
{

namespace action
{

namespace builder
{

class Init_BasicMotion_Result_message
{
public:
  explicit Init_BasicMotion_Result_message(::uv_msgs::action::BasicMotion_Result & msg)
  : msg_(msg)
  {}
  ::uv_msgs::action::BasicMotion_Result message(::uv_msgs::action::BasicMotion_Result::_message_type arg)
  {
    msg_.message = std::move(arg);
    return std::move(msg_);
  }

private:
  ::uv_msgs::action::BasicMotion_Result msg_;
};

class Init_BasicMotion_Result_success
{
public:
  Init_BasicMotion_Result_success()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_BasicMotion_Result_message success(::uv_msgs::action::BasicMotion_Result::_success_type arg)
  {
    msg_.success = std::move(arg);
    return Init_BasicMotion_Result_message(msg_);
  }

private:
  ::uv_msgs::action::BasicMotion_Result msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::uv_msgs::action::BasicMotion_Result>()
{
  return uv_msgs::action::builder::Init_BasicMotion_Result_success();
}

}  // namespace uv_msgs


namespace uv_msgs
{

namespace action
{

namespace builder
{

class Init_BasicMotion_Feedback_distance_remaining
{
public:
  Init_BasicMotion_Feedback_distance_remaining()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::uv_msgs::action::BasicMotion_Feedback distance_remaining(::uv_msgs::action::BasicMotion_Feedback::_distance_remaining_type arg)
  {
    msg_.distance_remaining = std::move(arg);
    return std::move(msg_);
  }

private:
  ::uv_msgs::action::BasicMotion_Feedback msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::uv_msgs::action::BasicMotion_Feedback>()
{
  return uv_msgs::action::builder::Init_BasicMotion_Feedback_distance_remaining();
}

}  // namespace uv_msgs


namespace uv_msgs
{

namespace action
{

namespace builder
{

class Init_BasicMotion_SendGoal_Request_goal
{
public:
  explicit Init_BasicMotion_SendGoal_Request_goal(::uv_msgs::action::BasicMotion_SendGoal_Request & msg)
  : msg_(msg)
  {}
  ::uv_msgs::action::BasicMotion_SendGoal_Request goal(::uv_msgs::action::BasicMotion_SendGoal_Request::_goal_type arg)
  {
    msg_.goal = std::move(arg);
    return std::move(msg_);
  }

private:
  ::uv_msgs::action::BasicMotion_SendGoal_Request msg_;
};

class Init_BasicMotion_SendGoal_Request_goal_id
{
public:
  Init_BasicMotion_SendGoal_Request_goal_id()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_BasicMotion_SendGoal_Request_goal goal_id(::uv_msgs::action::BasicMotion_SendGoal_Request::_goal_id_type arg)
  {
    msg_.goal_id = std::move(arg);
    return Init_BasicMotion_SendGoal_Request_goal(msg_);
  }

private:
  ::uv_msgs::action::BasicMotion_SendGoal_Request msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::uv_msgs::action::BasicMotion_SendGoal_Request>()
{
  return uv_msgs::action::builder::Init_BasicMotion_SendGoal_Request_goal_id();
}

}  // namespace uv_msgs


namespace uv_msgs
{

namespace action
{

namespace builder
{

class Init_BasicMotion_SendGoal_Response_stamp
{
public:
  explicit Init_BasicMotion_SendGoal_Response_stamp(::uv_msgs::action::BasicMotion_SendGoal_Response & msg)
  : msg_(msg)
  {}
  ::uv_msgs::action::BasicMotion_SendGoal_Response stamp(::uv_msgs::action::BasicMotion_SendGoal_Response::_stamp_type arg)
  {
    msg_.stamp = std::move(arg);
    return std::move(msg_);
  }

private:
  ::uv_msgs::action::BasicMotion_SendGoal_Response msg_;
};

class Init_BasicMotion_SendGoal_Response_accepted
{
public:
  Init_BasicMotion_SendGoal_Response_accepted()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_BasicMotion_SendGoal_Response_stamp accepted(::uv_msgs::action::BasicMotion_SendGoal_Response::_accepted_type arg)
  {
    msg_.accepted = std::move(arg);
    return Init_BasicMotion_SendGoal_Response_stamp(msg_);
  }

private:
  ::uv_msgs::action::BasicMotion_SendGoal_Response msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::uv_msgs::action::BasicMotion_SendGoal_Response>()
{
  return uv_msgs::action::builder::Init_BasicMotion_SendGoal_Response_accepted();
}

}  // namespace uv_msgs


namespace uv_msgs
{

namespace action
{

namespace builder
{

class Init_BasicMotion_SendGoal_Event_response
{
public:
  explicit Init_BasicMotion_SendGoal_Event_response(::uv_msgs::action::BasicMotion_SendGoal_Event & msg)
  : msg_(msg)
  {}
  ::uv_msgs::action::BasicMotion_SendGoal_Event response(::uv_msgs::action::BasicMotion_SendGoal_Event::_response_type arg)
  {
    msg_.response = std::move(arg);
    return std::move(msg_);
  }

private:
  ::uv_msgs::action::BasicMotion_SendGoal_Event msg_;
};

class Init_BasicMotion_SendGoal_Event_request
{
public:
  explicit Init_BasicMotion_SendGoal_Event_request(::uv_msgs::action::BasicMotion_SendGoal_Event & msg)
  : msg_(msg)
  {}
  Init_BasicMotion_SendGoal_Event_response request(::uv_msgs::action::BasicMotion_SendGoal_Event::_request_type arg)
  {
    msg_.request = std::move(arg);
    return Init_BasicMotion_SendGoal_Event_response(msg_);
  }

private:
  ::uv_msgs::action::BasicMotion_SendGoal_Event msg_;
};

class Init_BasicMotion_SendGoal_Event_info
{
public:
  Init_BasicMotion_SendGoal_Event_info()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_BasicMotion_SendGoal_Event_request info(::uv_msgs::action::BasicMotion_SendGoal_Event::_info_type arg)
  {
    msg_.info = std::move(arg);
    return Init_BasicMotion_SendGoal_Event_request(msg_);
  }

private:
  ::uv_msgs::action::BasicMotion_SendGoal_Event msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::uv_msgs::action::BasicMotion_SendGoal_Event>()
{
  return uv_msgs::action::builder::Init_BasicMotion_SendGoal_Event_info();
}

}  // namespace uv_msgs


namespace uv_msgs
{

namespace action
{

namespace builder
{

class Init_BasicMotion_GetResult_Request_goal_id
{
public:
  Init_BasicMotion_GetResult_Request_goal_id()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::uv_msgs::action::BasicMotion_GetResult_Request goal_id(::uv_msgs::action::BasicMotion_GetResult_Request::_goal_id_type arg)
  {
    msg_.goal_id = std::move(arg);
    return std::move(msg_);
  }

private:
  ::uv_msgs::action::BasicMotion_GetResult_Request msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::uv_msgs::action::BasicMotion_GetResult_Request>()
{
  return uv_msgs::action::builder::Init_BasicMotion_GetResult_Request_goal_id();
}

}  // namespace uv_msgs


namespace uv_msgs
{

namespace action
{

namespace builder
{

class Init_BasicMotion_GetResult_Response_result
{
public:
  explicit Init_BasicMotion_GetResult_Response_result(::uv_msgs::action::BasicMotion_GetResult_Response & msg)
  : msg_(msg)
  {}
  ::uv_msgs::action::BasicMotion_GetResult_Response result(::uv_msgs::action::BasicMotion_GetResult_Response::_result_type arg)
  {
    msg_.result = std::move(arg);
    return std::move(msg_);
  }

private:
  ::uv_msgs::action::BasicMotion_GetResult_Response msg_;
};

class Init_BasicMotion_GetResult_Response_status
{
public:
  Init_BasicMotion_GetResult_Response_status()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_BasicMotion_GetResult_Response_result status(::uv_msgs::action::BasicMotion_GetResult_Response::_status_type arg)
  {
    msg_.status = std::move(arg);
    return Init_BasicMotion_GetResult_Response_result(msg_);
  }

private:
  ::uv_msgs::action::BasicMotion_GetResult_Response msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::uv_msgs::action::BasicMotion_GetResult_Response>()
{
  return uv_msgs::action::builder::Init_BasicMotion_GetResult_Response_status();
}

}  // namespace uv_msgs


namespace uv_msgs
{

namespace action
{

namespace builder
{

class Init_BasicMotion_GetResult_Event_response
{
public:
  explicit Init_BasicMotion_GetResult_Event_response(::uv_msgs::action::BasicMotion_GetResult_Event & msg)
  : msg_(msg)
  {}
  ::uv_msgs::action::BasicMotion_GetResult_Event response(::uv_msgs::action::BasicMotion_GetResult_Event::_response_type arg)
  {
    msg_.response = std::move(arg);
    return std::move(msg_);
  }

private:
  ::uv_msgs::action::BasicMotion_GetResult_Event msg_;
};

class Init_BasicMotion_GetResult_Event_request
{
public:
  explicit Init_BasicMotion_GetResult_Event_request(::uv_msgs::action::BasicMotion_GetResult_Event & msg)
  : msg_(msg)
  {}
  Init_BasicMotion_GetResult_Event_response request(::uv_msgs::action::BasicMotion_GetResult_Event::_request_type arg)
  {
    msg_.request = std::move(arg);
    return Init_BasicMotion_GetResult_Event_response(msg_);
  }

private:
  ::uv_msgs::action::BasicMotion_GetResult_Event msg_;
};

class Init_BasicMotion_GetResult_Event_info
{
public:
  Init_BasicMotion_GetResult_Event_info()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_BasicMotion_GetResult_Event_request info(::uv_msgs::action::BasicMotion_GetResult_Event::_info_type arg)
  {
    msg_.info = std::move(arg);
    return Init_BasicMotion_GetResult_Event_request(msg_);
  }

private:
  ::uv_msgs::action::BasicMotion_GetResult_Event msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::uv_msgs::action::BasicMotion_GetResult_Event>()
{
  return uv_msgs::action::builder::Init_BasicMotion_GetResult_Event_info();
}

}  // namespace uv_msgs


namespace uv_msgs
{

namespace action
{

namespace builder
{

class Init_BasicMotion_FeedbackMessage_feedback
{
public:
  explicit Init_BasicMotion_FeedbackMessage_feedback(::uv_msgs::action::BasicMotion_FeedbackMessage & msg)
  : msg_(msg)
  {}
  ::uv_msgs::action::BasicMotion_FeedbackMessage feedback(::uv_msgs::action::BasicMotion_FeedbackMessage::_feedback_type arg)
  {
    msg_.feedback = std::move(arg);
    return std::move(msg_);
  }

private:
  ::uv_msgs::action::BasicMotion_FeedbackMessage msg_;
};

class Init_BasicMotion_FeedbackMessage_goal_id
{
public:
  Init_BasicMotion_FeedbackMessage_goal_id()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_BasicMotion_FeedbackMessage_feedback goal_id(::uv_msgs::action::BasicMotion_FeedbackMessage::_goal_id_type arg)
  {
    msg_.goal_id = std::move(arg);
    return Init_BasicMotion_FeedbackMessage_feedback(msg_);
  }

private:
  ::uv_msgs::action::BasicMotion_FeedbackMessage msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::uv_msgs::action::BasicMotion_FeedbackMessage>()
{
  return uv_msgs::action::builder::Init_BasicMotion_FeedbackMessage_goal_id();
}

}  // namespace uv_msgs

#endif  // UV_MSGS__ACTION__DETAIL__BASIC_MOTION__BUILDER_HPP_
