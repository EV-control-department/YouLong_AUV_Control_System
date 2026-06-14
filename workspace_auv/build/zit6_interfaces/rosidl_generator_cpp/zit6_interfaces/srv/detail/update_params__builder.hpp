// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from zit6_interfaces:srv/UpdateParams.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "zit6_interfaces/srv/update_params.hpp"


#ifndef ZIT6_INTERFACES__SRV__DETAIL__UPDATE_PARAMS__BUILDER_HPP_
#define ZIT6_INTERFACES__SRV__DETAIL__UPDATE_PARAMS__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "zit6_interfaces/srv/detail/update_params__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace zit6_interfaces
{

namespace srv
{

namespace builder
{

class Init_UpdateParams_Request_values
{
public:
  explicit Init_UpdateParams_Request_values(::zit6_interfaces::srv::UpdateParams_Request & msg)
  : msg_(msg)
  {}
  ::zit6_interfaces::srv::UpdateParams_Request values(::zit6_interfaces::srv::UpdateParams_Request::_values_type arg)
  {
    msg_.values = std::move(arg);
    return std::move(msg_);
  }

private:
  ::zit6_interfaces::srv::UpdateParams_Request msg_;
};

class Init_UpdateParams_Request_paths
{
public:
  explicit Init_UpdateParams_Request_paths(::zit6_interfaces::srv::UpdateParams_Request & msg)
  : msg_(msg)
  {}
  Init_UpdateParams_Request_values paths(::zit6_interfaces::srv::UpdateParams_Request::_paths_type arg)
  {
    msg_.paths = std::move(arg);
    return Init_UpdateParams_Request_values(msg_);
  }

private:
  ::zit6_interfaces::srv::UpdateParams_Request msg_;
};

class Init_UpdateParams_Request_json
{
public:
  Init_UpdateParams_Request_json()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_UpdateParams_Request_paths json(::zit6_interfaces::srv::UpdateParams_Request::_json_type arg)
  {
    msg_.json = std::move(arg);
    return Init_UpdateParams_Request_paths(msg_);
  }

private:
  ::zit6_interfaces::srv::UpdateParams_Request msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::zit6_interfaces::srv::UpdateParams_Request>()
{
  return zit6_interfaces::srv::builder::Init_UpdateParams_Request_json();
}

}  // namespace zit6_interfaces


namespace zit6_interfaces
{

namespace srv
{

namespace builder
{

class Init_UpdateParams_Response_message
{
public:
  explicit Init_UpdateParams_Response_message(::zit6_interfaces::srv::UpdateParams_Response & msg)
  : msg_(msg)
  {}
  ::zit6_interfaces::srv::UpdateParams_Response message(::zit6_interfaces::srv::UpdateParams_Response::_message_type arg)
  {
    msg_.message = std::move(arg);
    return std::move(msg_);
  }

private:
  ::zit6_interfaces::srv::UpdateParams_Response msg_;
};

class Init_UpdateParams_Response_success
{
public:
  Init_UpdateParams_Response_success()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_UpdateParams_Response_message success(::zit6_interfaces::srv::UpdateParams_Response::_success_type arg)
  {
    msg_.success = std::move(arg);
    return Init_UpdateParams_Response_message(msg_);
  }

private:
  ::zit6_interfaces::srv::UpdateParams_Response msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::zit6_interfaces::srv::UpdateParams_Response>()
{
  return zit6_interfaces::srv::builder::Init_UpdateParams_Response_success();
}

}  // namespace zit6_interfaces


namespace zit6_interfaces
{

namespace srv
{

namespace builder
{

class Init_UpdateParams_Event_response
{
public:
  explicit Init_UpdateParams_Event_response(::zit6_interfaces::srv::UpdateParams_Event & msg)
  : msg_(msg)
  {}
  ::zit6_interfaces::srv::UpdateParams_Event response(::zit6_interfaces::srv::UpdateParams_Event::_response_type arg)
  {
    msg_.response = std::move(arg);
    return std::move(msg_);
  }

private:
  ::zit6_interfaces::srv::UpdateParams_Event msg_;
};

class Init_UpdateParams_Event_request
{
public:
  explicit Init_UpdateParams_Event_request(::zit6_interfaces::srv::UpdateParams_Event & msg)
  : msg_(msg)
  {}
  Init_UpdateParams_Event_response request(::zit6_interfaces::srv::UpdateParams_Event::_request_type arg)
  {
    msg_.request = std::move(arg);
    return Init_UpdateParams_Event_response(msg_);
  }

private:
  ::zit6_interfaces::srv::UpdateParams_Event msg_;
};

class Init_UpdateParams_Event_info
{
public:
  Init_UpdateParams_Event_info()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_UpdateParams_Event_request info(::zit6_interfaces::srv::UpdateParams_Event::_info_type arg)
  {
    msg_.info = std::move(arg);
    return Init_UpdateParams_Event_request(msg_);
  }

private:
  ::zit6_interfaces::srv::UpdateParams_Event msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::zit6_interfaces::srv::UpdateParams_Event>()
{
  return zit6_interfaces::srv::builder::Init_UpdateParams_Event_info();
}

}  // namespace zit6_interfaces

#endif  // ZIT6_INTERFACES__SRV__DETAIL__UPDATE_PARAMS__BUILDER_HPP_
