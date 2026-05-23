// generated from rosidl_typesupport_fastrtps_c/resource/idl__rosidl_typesupport_fastrtps_c.h.em
// with input from auv_msgs:msg/ObjectDetectionArray.idl
// generated code does not contain a copyright notice
#ifndef AUV_MSGS__MSG__DETAIL__OBJECT_DETECTION_ARRAY__ROSIDL_TYPESUPPORT_FASTRTPS_C_H_
#define AUV_MSGS__MSG__DETAIL__OBJECT_DETECTION_ARRAY__ROSIDL_TYPESUPPORT_FASTRTPS_C_H_


#include <stddef.h>
#include "rosidl_runtime_c/message_type_support_struct.h"
#include "rosidl_typesupport_interface/macros.h"
#include "auv_msgs/msg/rosidl_typesupport_fastrtps_c__visibility_control.h"
#include "auv_msgs/msg/detail/object_detection_array__struct.h"
#include "fastcdr/Cdr.h"

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_auv_msgs
bool cdr_serialize_auv_msgs__msg__ObjectDetectionArray(
  const auv_msgs__msg__ObjectDetectionArray * ros_message,
  eprosima::fastcdr::Cdr & cdr);

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_auv_msgs
bool cdr_deserialize_auv_msgs__msg__ObjectDetectionArray(
  eprosima::fastcdr::Cdr &,
  auv_msgs__msg__ObjectDetectionArray * ros_message);

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_auv_msgs
size_t get_serialized_size_auv_msgs__msg__ObjectDetectionArray(
  const void * untyped_ros_message,
  size_t current_alignment);

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_auv_msgs
size_t max_serialized_size_auv_msgs__msg__ObjectDetectionArray(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment);

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_auv_msgs
bool cdr_serialize_key_auv_msgs__msg__ObjectDetectionArray(
  const auv_msgs__msg__ObjectDetectionArray * ros_message,
  eprosima::fastcdr::Cdr & cdr);

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_auv_msgs
size_t get_serialized_size_key_auv_msgs__msg__ObjectDetectionArray(
  const void * untyped_ros_message,
  size_t current_alignment);

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_auv_msgs
size_t max_serialized_size_key_auv_msgs__msg__ObjectDetectionArray(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment);

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_auv_msgs
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, auv_msgs, msg, ObjectDetectionArray)();

#ifdef __cplusplus
}
#endif

#endif  // AUV_MSGS__MSG__DETAIL__OBJECT_DETECTION_ARRAY__ROSIDL_TYPESUPPORT_FASTRTPS_C_H_
