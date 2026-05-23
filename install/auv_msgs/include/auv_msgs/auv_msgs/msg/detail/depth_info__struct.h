// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from auv_msgs:msg/DepthInfo.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "auv_msgs/msg/depth_info.h"


#ifndef AUV_MSGS__MSG__DETAIL__DEPTH_INFO__STRUCT_H_
#define AUV_MSGS__MSG__DETAIL__DEPTH_INFO__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

// Constants defined in the message

// Include directives for member types
// Member 'stamp'
#include "builtin_interfaces/msg/detail/time__struct.h"

/// Struct defined in msg/DepthInfo in the package auv_msgs.
typedef struct auv_msgs__msg__DepthInfo
{
  builtin_interfaces__msg__Time stamp;
  float sub_depth_m;
  float stop_distance_m;
} auv_msgs__msg__DepthInfo;

// Struct for a sequence of auv_msgs__msg__DepthInfo.
typedef struct auv_msgs__msg__DepthInfo__Sequence
{
  auv_msgs__msg__DepthInfo * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} auv_msgs__msg__DepthInfo__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // AUV_MSGS__MSG__DETAIL__DEPTH_INFO__STRUCT_H_
