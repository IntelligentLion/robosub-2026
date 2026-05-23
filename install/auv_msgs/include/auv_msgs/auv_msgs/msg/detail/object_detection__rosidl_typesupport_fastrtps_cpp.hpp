// generated from rosidl_typesupport_fastrtps_cpp/resource/idl__rosidl_typesupport_fastrtps_cpp.hpp.em
// with input from auv_msgs:msg/ObjectDetection.idl
// generated code does not contain a copyright notice

#ifndef AUV_MSGS__MSG__DETAIL__OBJECT_DETECTION__ROSIDL_TYPESUPPORT_FASTRTPS_CPP_HPP_
#define AUV_MSGS__MSG__DETAIL__OBJECT_DETECTION__ROSIDL_TYPESUPPORT_FASTRTPS_CPP_HPP_

#include <cstddef>
#include "rosidl_runtime_c/message_type_support_struct.h"
#include "rosidl_typesupport_interface/macros.h"
#include "auv_msgs/msg/rosidl_typesupport_fastrtps_cpp__visibility_control.h"
#include "auv_msgs/msg/detail/object_detection__struct.hpp"

#ifndef _WIN32
# pragma GCC diagnostic push
# pragma GCC diagnostic ignored "-Wunused-parameter"
# ifdef __clang__
#  pragma clang diagnostic ignored "-Wdeprecated-register"
#  pragma clang diagnostic ignored "-Wreturn-type-c-linkage"
# endif
#endif
#ifndef _WIN32
# pragma GCC diagnostic pop
#endif

#include "fastcdr/Cdr.h"

namespace auv_msgs
{

namespace msg
{

namespace typesupport_fastrtps_cpp
{

bool
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_auv_msgs
cdr_serialize(
  const auv_msgs::msg::ObjectDetection & ros_message,
  eprosima::fastcdr::Cdr & cdr);

bool
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_auv_msgs
cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  auv_msgs::msg::ObjectDetection & ros_message);

size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_auv_msgs
get_serialized_size(
  const auv_msgs::msg::ObjectDetection & ros_message,
  size_t current_alignment);

size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_auv_msgs
max_serialized_size_ObjectDetection(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment);

bool
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_auv_msgs
cdr_serialize_key(
  const auv_msgs::msg::ObjectDetection & ros_message,
  eprosima::fastcdr::Cdr &);

size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_auv_msgs
get_serialized_size_key(
  const auv_msgs::msg::ObjectDetection & ros_message,
  size_t current_alignment);

size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_auv_msgs
max_serialized_size_key_ObjectDetection(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment);

}  // namespace typesupport_fastrtps_cpp

}  // namespace msg

}  // namespace auv_msgs

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_auv_msgs
const rosidl_message_type_support_t *
  ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_cpp, auv_msgs, msg, ObjectDetection)();

#ifdef __cplusplus
}
#endif

#endif  // AUV_MSGS__MSG__DETAIL__OBJECT_DETECTION__ROSIDL_TYPESUPPORT_FASTRTPS_CPP_HPP_
