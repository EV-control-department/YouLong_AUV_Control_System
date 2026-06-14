// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from zit6_interfaces:srv/GetParams.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "zit6_interfaces/srv/get_params.hpp"


#ifndef ZIT6_INTERFACES__SRV__DETAIL__GET_PARAMS__BUILDER_HPP_
#define ZIT6_INTERFACES__SRV__DETAIL__GET_PARAMS__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "zit6_interfaces/srv/detail/get_params__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace zit6_interfaces
{

namespace srv
{

namespace builder
{

class Init_GetParams_Request_paths
{
public:
  Init_GetParams_Request_paths()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::zit6_interfaces::srv::GetParams_Request paths(::zit6_interfaces::srv::GetParams_Request::_paths_type arg)
  {
    msg_.paths = std::move(arg);
    return std::move(msg_);
  }

private:
  ::zit6_interfaces::srv::GetParams_Request msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::zit6_interfaces::srv::GetParams_Request>()
{
  return zit6_interfaces::srv::builder::Init_GetParams_Request_paths();
}

}  // namespace zit6_interfaces


namespace zit6_interfaces
{

namespace srv
{

namespace builder
{

class Init_GetParams_Response_config_json
{
public:
  explicit Init_GetParams_Response_config_json(::zit6_interfaces::srv::GetParams_Response & msg)
  : msg_(msg)
  {}
  ::zit6_interfaces::srv::GetParams_Response config_json(::zit6_interfaces::srv::GetParams_Response::_config_json_type arg)
  {
    msg_.config_json = std::move(arg);
    return std::move(msg_);
  }

private:
  ::zit6_interfaces::srv::GetParams_Response msg_;
};

class Init_GetParams_Response_message
{
public:
  explicit Init_GetParams_Response_message(::zit6_interfaces::srv::GetParams_Response & msg)
  : msg_(msg)
  {}
  Init_GetParams_Response_config_json message(::zit6_interfaces::srv::GetParams_Response::_message_type arg)
  {
    msg_.message = std::move(arg);
    return Init_GetParams_Response_config_json(msg_);
  }

private:
  ::zit6_interfaces::srv::GetParams_Response msg_;
};

class Init_GetParams_Response_success
{
public:
  Init_GetParams_Response_success()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_GetParams_Response_message success(::zit6_interfaces::srv::GetParams_Response::_success_type arg)
  {
    msg_.success = std::move(arg);
    return Init_GetParams_Response_message(msg_);
  }

private:
  ::zit6_interfaces::srv::GetParams_Response msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::zit6_interfaces::srv::GetParams_Response>()
{
  return zit6_interfaces::srv::builder::Init_GetParams_Response_success();
}

}  // namespace zit6_interfaces


namespace zit6_interfaces
{

namespace srv
{

namespace builder
{

class Init_GetParams_Event_response
{
public:
  explicit Init_GetParams_Event_response(::zit6_interfaces::srv::GetParams_Event & msg)
  : msg_(msg)
  {}
  ::zit6_interfaces::srv::GetParams_Event response(::zit6_interfaces::srv::GetParams_Event::_response_type arg)
  {
    msg_.response = std::move(arg);
    return std::move(msg_);
  }

private:
  ::zit6_interfaces::srv::GetParams_Event msg_;
};

class Init_GetParams_Event_request
{
public:
  explicit Init_GetParams_Event_request(::zit6_interfaces::srv::GetParams_Event & msg)
  : msg_(msg)
  {}
  Init_GetParams_Event_response request(::zit6_interfaces::srv::GetParams_Event::_request_type arg)
  {
    msg_.request = std::move(arg);
    return Init_GetParams_Event_response(msg_);
  }

private:
  ::zit6_interfaces::srv::GetParams_Event msg_;
};

class Init_GetParams_Event_info
{
public:
  Init_GetParams_Event_info()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_GetParams_Event_request info(::zit6_interfaces::srv::GetParams_Event::_info_type arg)
  {
    msg_.info = std::move(arg);
    return Init_GetParams_Event_request(msg_);
  }

private:
  ::zit6_interfaces::srv::GetParams_Event msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::zit6_interfaces::srv::GetParams_Event>()
{
  return zit6_interfaces::srv::builder::Init_GetParams_Event_info();
}

}  // namespace zit6_interfaces

#endif  // ZIT6_INTERFACES__SRV__DETAIL__GET_PARAMS__BUILDER_HPP_
