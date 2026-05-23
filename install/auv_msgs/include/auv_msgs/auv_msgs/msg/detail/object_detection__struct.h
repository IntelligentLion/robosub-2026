// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from auv_msgs:msg/ObjectDetection.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "auv_msgs/msg/object_detection.h"


#ifndef AUV_MSGS__MSG__DETAIL__OBJECT_DETECTION__STRUCT_H_
#define AUV_MSGS__MSG__DETAIL__OBJECT_DETECTION__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

// Constants defined in the message

// Include directives for member types
// Member 'label'
#include "rosidl_runtime_c/string.h"
// Member 'position'
#include "geometry_msgs/msg/detail/point__struct.h"

/// Struct defined in msg/ObjectDetection in the package auv_msgs.
typedef struct auv_msgs__msg__ObjectDetection
{
  rosidl_runtime_c__String label;
  float confidence;
  geometry_msgs__msg__Point position;
  float bbox_width;
  float bbox_height;
} auv_msgs__msg__ObjectDetection;

// Struct for a sequence of auv_msgs__msg__ObjectDetection.
typedef struct auv_msgs__msg__ObjectDetection__Sequence
{
  auv_msgs__msg__ObjectDetection * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} auv_msgs__msg__ObjectDetection__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // AUV_MSGS__MSG__DETAIL__OBJECT_DETECTION__STRUCT_H_
