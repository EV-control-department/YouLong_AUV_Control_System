// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from uv_msgs:msg/Detection.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "uv_msgs/msg/detection.hpp"


#ifndef UV_MSGS__MSG__DETAIL__DETECTION__BUILDER_HPP_
#define UV_MSGS__MSG__DETAIL__DETECTION__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "uv_msgs/msg/detail/detection__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace uv_msgs
{

namespace msg
{

namespace builder
{

class Init_Detection_bbox_y2
{
public:
  explicit Init_Detection_bbox_y2(::uv_msgs::msg::Detection & msg)
  : msg_(msg)
  {}
  ::uv_msgs::msg::Detection bbox_y2(::uv_msgs::msg::Detection::_bbox_y2_type arg)
  {
    msg_.bbox_y2 = std::move(arg);
    return std::move(msg_);
  }

private:
  ::uv_msgs::msg::Detection msg_;
};

class Init_Detection_bbox_x2
{
public:
  explicit Init_Detection_bbox_x2(::uv_msgs::msg::Detection & msg)
  : msg_(msg)
  {}
  Init_Detection_bbox_y2 bbox_x2(::uv_msgs::msg::Detection::_bbox_x2_type arg)
  {
    msg_.bbox_x2 = std::move(arg);
    return Init_Detection_bbox_y2(msg_);
  }

private:
  ::uv_msgs::msg::Detection msg_;
};

class Init_Detection_bbox_y1
{
public:
  explicit Init_Detection_bbox_y1(::uv_msgs::msg::Detection & msg)
  : msg_(msg)
  {}
  Init_Detection_bbox_x2 bbox_y1(::uv_msgs::msg::Detection::_bbox_y1_type arg)
  {
    msg_.bbox_y1 = std::move(arg);
    return Init_Detection_bbox_x2(msg_);
  }

private:
  ::uv_msgs::msg::Detection msg_;
};

class Init_Detection_bbox_x1
{
public:
  explicit Init_Detection_bbox_x1(::uv_msgs::msg::Detection & msg)
  : msg_(msg)
  {}
  Init_Detection_bbox_y1 bbox_x1(::uv_msgs::msg::Detection::_bbox_x1_type arg)
  {
    msg_.bbox_x1 = std::move(arg);
    return Init_Detection_bbox_y1(msg_);
  }

private:
  ::uv_msgs::msg::Detection msg_;
};

class Init_Detection_pixel_y
{
public:
  explicit Init_Detection_pixel_y(::uv_msgs::msg::Detection & msg)
  : msg_(msg)
  {}
  Init_Detection_bbox_x1 pixel_y(::uv_msgs::msg::Detection::_pixel_y_type arg)
  {
    msg_.pixel_y = std::move(arg);
    return Init_Detection_bbox_x1(msg_);
  }

private:
  ::uv_msgs::msg::Detection msg_;
};

class Init_Detection_pixel_x
{
public:
  explicit Init_Detection_pixel_x(::uv_msgs::msg::Detection & msg)
  : msg_(msg)
  {}
  Init_Detection_pixel_y pixel_x(::uv_msgs::msg::Detection::_pixel_x_type arg)
  {
    msg_.pixel_x = std::move(arg);
    return Init_Detection_pixel_y(msg_);
  }

private:
  ::uv_msgs::msg::Detection msg_;
};

class Init_Detection_confidence
{
public:
  explicit Init_Detection_confidence(::uv_msgs::msg::Detection & msg)
  : msg_(msg)
  {}
  Init_Detection_pixel_x confidence(::uv_msgs::msg::Detection::_confidence_type arg)
  {
    msg_.confidence = std::move(arg);
    return Init_Detection_pixel_x(msg_);
  }

private:
  ::uv_msgs::msg::Detection msg_;
};

class Init_Detection_class_id
{
public:
  Init_Detection_class_id()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_Detection_confidence class_id(::uv_msgs::msg::Detection::_class_id_type arg)
  {
    msg_.class_id = std::move(arg);
    return Init_Detection_confidence(msg_);
  }

private:
  ::uv_msgs::msg::Detection msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::uv_msgs::msg::Detection>()
{
  return uv_msgs::msg::builder::Init_Detection_class_id();
}

}  // namespace uv_msgs

#endif  // UV_MSGS__MSG__DETAIL__DETECTION__BUILDER_HPP_
