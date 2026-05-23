// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from auv_msgs:msg/ObjectDetectionArray.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "auv_msgs/msg/object_detection_array.hpp"


#ifndef AUV_MSGS__MSG__DETAIL__OBJECT_DETECTION_ARRAY__BUILDER_HPP_
#define AUV_MSGS__MSG__DETAIL__OBJECT_DETECTION_ARRAY__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "auv_msgs/msg/detail/object_detection_array__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace auv_msgs
{

namespace msg
{

namespace builder
{

class Init_ObjectDetectionArray_detections
{
public:
  Init_ObjectDetectionArray_detections()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::auv_msgs::msg::ObjectDetectionArray detections(::auv_msgs::msg::ObjectDetectionArray::_detections_type arg)
  {
    msg_.detections = std::move(arg);
    return std::move(msg_);
  }

private:
  ::auv_msgs::msg::ObjectDetectionArray msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::auv_msgs::msg::ObjectDetectionArray>()
{
  return auv_msgs::msg::builder::Init_ObjectDetectionArray_detections();
}

}  // namespace auv_msgs

#endif  // AUV_MSGS__MSG__DETAIL__OBJECT_DETECTION_ARRAY__BUILDER_HPP_
