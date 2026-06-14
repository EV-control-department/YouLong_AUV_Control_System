// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from uv_msgs:msg/DetectionArray.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "uv_msgs/msg/detection_array.hpp"


#ifndef UV_MSGS__MSG__DETAIL__DETECTION_ARRAY__BUILDER_HPP_
#define UV_MSGS__MSG__DETAIL__DETECTION_ARRAY__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "uv_msgs/msg/detail/detection_array__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace uv_msgs
{

namespace msg
{

namespace builder
{

class Init_DetectionArray_detections
{
public:
  explicit Init_DetectionArray_detections(::uv_msgs::msg::DetectionArray & msg)
  : msg_(msg)
  {}
  ::uv_msgs::msg::DetectionArray detections(::uv_msgs::msg::DetectionArray::_detections_type arg)
  {
    msg_.detections = std::move(arg);
    return std::move(msg_);
  }

private:
  ::uv_msgs::msg::DetectionArray msg_;
};

class Init_DetectionArray_camera_name
{
public:
  explicit Init_DetectionArray_camera_name(::uv_msgs::msg::DetectionArray & msg)
  : msg_(msg)
  {}
  Init_DetectionArray_detections camera_name(::uv_msgs::msg::DetectionArray::_camera_name_type arg)
  {
    msg_.camera_name = std::move(arg);
    return Init_DetectionArray_detections(msg_);
  }

private:
  ::uv_msgs::msg::DetectionArray msg_;
};

class Init_DetectionArray_header
{
public:
  Init_DetectionArray_header()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_DetectionArray_camera_name header(::uv_msgs::msg::DetectionArray::_header_type arg)
  {
    msg_.header = std::move(arg);
    return Init_DetectionArray_camera_name(msg_);
  }

private:
  ::uv_msgs::msg::DetectionArray msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::uv_msgs::msg::DetectionArray>()
{
  return uv_msgs::msg::builder::Init_DetectionArray_header();
}

}  // namespace uv_msgs

#endif  // UV_MSGS__MSG__DETAIL__DETECTION_ARRAY__BUILDER_HPP_
