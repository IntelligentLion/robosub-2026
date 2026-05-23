// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from auv_msgs:msg/ObjectDetection.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "auv_msgs/msg/object_detection.hpp"


#ifndef AUV_MSGS__MSG__DETAIL__OBJECT_DETECTION__BUILDER_HPP_
#define AUV_MSGS__MSG__DETAIL__OBJECT_DETECTION__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "auv_msgs/msg/detail/object_detection__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace auv_msgs
{

namespace msg
{

namespace builder
{

class Init_ObjectDetection_bbox_height
{
public:
  explicit Init_ObjectDetection_bbox_height(::auv_msgs::msg::ObjectDetection & msg)
  : msg_(msg)
  {}
  ::auv_msgs::msg::ObjectDetection bbox_height(::auv_msgs::msg::ObjectDetection::_bbox_height_type arg)
  {
    msg_.bbox_height = std::move(arg);
    return std::move(msg_);
  }

private:
  ::auv_msgs::msg::ObjectDetection msg_;
};

class Init_ObjectDetection_bbox_width
{
public:
  explicit Init_ObjectDetection_bbox_width(::auv_msgs::msg::ObjectDetection & msg)
  : msg_(msg)
  {}
  Init_ObjectDetection_bbox_height bbox_width(::auv_msgs::msg::ObjectDetection::_bbox_width_type arg)
  {
    msg_.bbox_width = std::move(arg);
    return Init_ObjectDetection_bbox_height(msg_);
  }

private:
  ::auv_msgs::msg::ObjectDetection msg_;
};

class Init_ObjectDetection_position
{
public:
  explicit Init_ObjectDetection_position(::auv_msgs::msg::ObjectDetection & msg)
  : msg_(msg)
  {}
  Init_ObjectDetection_bbox_width position(::auv_msgs::msg::ObjectDetection::_position_type arg)
  {
    msg_.position = std::move(arg);
    return Init_ObjectDetection_bbox_width(msg_);
  }

private:
  ::auv_msgs::msg::ObjectDetection msg_;
};

class Init_ObjectDetection_confidence
{
public:
  explicit Init_ObjectDetection_confidence(::auv_msgs::msg::ObjectDetection & msg)
  : msg_(msg)
  {}
  Init_ObjectDetection_position confidence(::auv_msgs::msg::ObjectDetection::_confidence_type arg)
  {
    msg_.confidence = std::move(arg);
    return Init_ObjectDetection_position(msg_);
  }

private:
  ::auv_msgs::msg::ObjectDetection msg_;
};

class Init_ObjectDetection_label
{
public:
  Init_ObjectDetection_label()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_ObjectDetection_confidence label(::auv_msgs::msg::ObjectDetection::_label_type arg)
  {
    msg_.label = std::move(arg);
    return Init_ObjectDetection_confidence(msg_);
  }

private:
  ::auv_msgs::msg::ObjectDetection msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::auv_msgs::msg::ObjectDetection>()
{
  return auv_msgs::msg::builder::Init_ObjectDetection_label();
}

}  // namespace auv_msgs

#endif  // AUV_MSGS__MSG__DETAIL__OBJECT_DETECTION__BUILDER_HPP_
