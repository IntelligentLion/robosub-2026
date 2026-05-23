// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from auv_msgs:msg/ObjectDetectionArray.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "auv_msgs/msg/object_detection_array.h"


#ifndef AUV_MSGS__MSG__DETAIL__OBJECT_DETECTION_ARRAY__STRUCT_H_
#define AUV_MSGS__MSG__DETAIL__OBJECT_DETECTION_ARRAY__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

// Constants defined in the message

// Include directives for member types
// Member 'detections'
#include "auv_msgs/msg/detail/object_detection__struct.h"

/// Struct defined in msg/ObjectDetectionArray in the package auv_msgs.
typedef struct auv_msgs__msg__ObjectDetectionArray
{
  auv_msgs__msg__ObjectDetection__Sequence detections;
} auv_msgs__msg__ObjectDetectionArray;

// Struct for a sequence of auv_msgs__msg__ObjectDetectionArray.
typedef struct auv_msgs__msg__ObjectDetectionArray__Sequence
{
  auv_msgs__msg__ObjectDetectionArray * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} auv_msgs__msg__ObjectDetectionArray__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // AUV_MSGS__MSG__DETAIL__OBJECT_DETECTION_ARRAY__STRUCT_H_
