// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from uv_msgs:srv/RunTask.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "uv_msgs/srv/run_task.hpp"


#ifndef UV_MSGS__SRV__DETAIL__RUN_TASK__STRUCT_HPP_
#define UV_MSGS__SRV__DETAIL__RUN_TASK__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__uv_msgs__srv__RunTask_Request __attribute__((deprecated))
#else
# define DEPRECATED__uv_msgs__srv__RunTask_Request __declspec(deprecated)
#endif

namespace uv_msgs
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct RunTask_Request_
{
  using Type = RunTask_Request_<ContainerAllocator>;

  explicit RunTask_Request_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->task_name = "";
      this->start = false;
    }
  }

  explicit RunTask_Request_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : task_name(_alloc)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->task_name = "";
      this->start = false;
    }
  }

  // field types and members
  using _task_name_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _task_name_type task_name;
  using _start_type =
    bool;
  _start_type start;

  // setters for named parameter idiom
  Type & set__task_name(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->task_name = _arg;
    return *this;
  }
  Type & set__start(
    const bool & _arg)
  {
    this->start = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    uv_msgs::srv::RunTask_Request_<ContainerAllocator> *;
  using ConstRawPtr =
    const uv_msgs::srv::RunTask_Request_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<uv_msgs::srv::RunTask_Request_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<uv_msgs::srv::RunTask_Request_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      uv_msgs::srv::RunTask_Request_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<uv_msgs::srv::RunTask_Request_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      uv_msgs::srv::RunTask_Request_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<uv_msgs::srv::RunTask_Request_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<uv_msgs::srv::RunTask_Request_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<uv_msgs::srv::RunTask_Request_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__uv_msgs__srv__RunTask_Request
    std::shared_ptr<uv_msgs::srv::RunTask_Request_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__uv_msgs__srv__RunTask_Request
    std::shared_ptr<uv_msgs::srv::RunTask_Request_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const RunTask_Request_ & other) const
  {
    if (this->task_name != other.task_name) {
      return false;
    }
    if (this->start != other.start) {
      return false;
    }
    return true;
  }
  bool operator!=(const RunTask_Request_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct RunTask_Request_

// alias to use template instance with default allocator
using RunTask_Request =
  uv_msgs::srv::RunTask_Request_<std::allocator<void>>;

// constant definitions

}  // namespace srv

}  // namespace uv_msgs


#ifndef _WIN32
# define DEPRECATED__uv_msgs__srv__RunTask_Response __attribute__((deprecated))
#else
# define DEPRECATED__uv_msgs__srv__RunTask_Response __declspec(deprecated)
#endif

namespace uv_msgs
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct RunTask_Response_
{
  using Type = RunTask_Response_<ContainerAllocator>;

  explicit RunTask_Response_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->success = false;
      this->message = "";
    }
  }

  explicit RunTask_Response_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : message(_alloc)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->success = false;
      this->message = "";
    }
  }

  // field types and members
  using _success_type =
    bool;
  _success_type success;
  using _message_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _message_type message;

  // setters for named parameter idiom
  Type & set__success(
    const bool & _arg)
  {
    this->success = _arg;
    return *this;
  }
  Type & set__message(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->message = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    uv_msgs::srv::RunTask_Response_<ContainerAllocator> *;
  using ConstRawPtr =
    const uv_msgs::srv::RunTask_Response_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<uv_msgs::srv::RunTask_Response_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<uv_msgs::srv::RunTask_Response_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      uv_msgs::srv::RunTask_Response_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<uv_msgs::srv::RunTask_Response_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      uv_msgs::srv::RunTask_Response_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<uv_msgs::srv::RunTask_Response_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<uv_msgs::srv::RunTask_Response_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<uv_msgs::srv::RunTask_Response_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__uv_msgs__srv__RunTask_Response
    std::shared_ptr<uv_msgs::srv::RunTask_Response_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__uv_msgs__srv__RunTask_Response
    std::shared_ptr<uv_msgs::srv::RunTask_Response_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const RunTask_Response_ & other) const
  {
    if (this->success != other.success) {
      return false;
    }
    if (this->message != other.message) {
      return false;
    }
    return true;
  }
  bool operator!=(const RunTask_Response_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct RunTask_Response_

// alias to use template instance with default allocator
using RunTask_Response =
  uv_msgs::srv::RunTask_Response_<std::allocator<void>>;

// constant definitions

}  // namespace srv

}  // namespace uv_msgs


// Include directives for member types
// Member 'info'
#include "service_msgs/msg/detail/service_event_info__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__uv_msgs__srv__RunTask_Event __attribute__((deprecated))
#else
# define DEPRECATED__uv_msgs__srv__RunTask_Event __declspec(deprecated)
#endif

namespace uv_msgs
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct RunTask_Event_
{
  using Type = RunTask_Event_<ContainerAllocator>;

  explicit RunTask_Event_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : info(_init)
  {
    (void)_init;
  }

  explicit RunTask_Event_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : info(_alloc, _init)
  {
    (void)_init;
  }

  // field types and members
  using _info_type =
    service_msgs::msg::ServiceEventInfo_<ContainerAllocator>;
  _info_type info;
  using _request_type =
    rosidl_runtime_cpp::BoundedVector<uv_msgs::srv::RunTask_Request_<ContainerAllocator>, 1, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<uv_msgs::srv::RunTask_Request_<ContainerAllocator>>>;
  _request_type request;
  using _response_type =
    rosidl_runtime_cpp::BoundedVector<uv_msgs::srv::RunTask_Response_<ContainerAllocator>, 1, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<uv_msgs::srv::RunTask_Response_<ContainerAllocator>>>;
  _response_type response;

  // setters for named parameter idiom
  Type & set__info(
    const service_msgs::msg::ServiceEventInfo_<ContainerAllocator> & _arg)
  {
    this->info = _arg;
    return *this;
  }
  Type & set__request(
    const rosidl_runtime_cpp::BoundedVector<uv_msgs::srv::RunTask_Request_<ContainerAllocator>, 1, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<uv_msgs::srv::RunTask_Request_<ContainerAllocator>>> & _arg)
  {
    this->request = _arg;
    return *this;
  }
  Type & set__response(
    const rosidl_runtime_cpp::BoundedVector<uv_msgs::srv::RunTask_Response_<ContainerAllocator>, 1, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<uv_msgs::srv::RunTask_Response_<ContainerAllocator>>> & _arg)
  {
    this->response = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    uv_msgs::srv::RunTask_Event_<ContainerAllocator> *;
  using ConstRawPtr =
    const uv_msgs::srv::RunTask_Event_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<uv_msgs::srv::RunTask_Event_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<uv_msgs::srv::RunTask_Event_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      uv_msgs::srv::RunTask_Event_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<uv_msgs::srv::RunTask_Event_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      uv_msgs::srv::RunTask_Event_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<uv_msgs::srv::RunTask_Event_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<uv_msgs::srv::RunTask_Event_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<uv_msgs::srv::RunTask_Event_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__uv_msgs__srv__RunTask_Event
    std::shared_ptr<uv_msgs::srv::RunTask_Event_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__uv_msgs__srv__RunTask_Event
    std::shared_ptr<uv_msgs::srv::RunTask_Event_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const RunTask_Event_ & other) const
  {
    if (this->info != other.info) {
      return false;
    }
    if (this->request != other.request) {
      return false;
    }
    if (this->response != other.response) {
      return false;
    }
    return true;
  }
  bool operator!=(const RunTask_Event_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct RunTask_Event_

// alias to use template instance with default allocator
using RunTask_Event =
  uv_msgs::srv::RunTask_Event_<std::allocator<void>>;

// constant definitions

}  // namespace srv

}  // namespace uv_msgs

namespace uv_msgs
{

namespace srv
{

struct RunTask
{
  using Request = uv_msgs::srv::RunTask_Request;
  using Response = uv_msgs::srv::RunTask_Response;
  using Event = uv_msgs::srv::RunTask_Event;
};

}  // namespace srv

}  // namespace uv_msgs

#endif  // UV_MSGS__SRV__DETAIL__RUN_TASK__STRUCT_HPP_
