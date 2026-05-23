// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from auv_msgs:msg/BehaviorStatus.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "auv_msgs/msg/behavior_status.h"


#ifndef AUV_MSGS__MSG__DETAIL__BEHAVIOR_STATUS__STRUCT_H_
#define AUV_MSGS__MSG__DETAIL__BEHAVIOR_STATUS__STRUCT_H_

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
// Member 'action_name'
// Member 'status'
// Member 'reason'
#include "rosidl_runtime_c/string.h"

/// Struct defined in msg/BehaviorStatus in the package auv_msgs.
typedef struct auv_msgs__msg__BehaviorStatus
{
  builtin_interfaces__msg__Time stamp;
  rosidl_runtime_c__String action_name;
  rosidl_runtime_c__String status;
  rosidl_runtime_c__String reason;
} auv_msgs__msg__BehaviorStatus;

// Struct for a sequence of auv_msgs__msg__BehaviorStatus.
typedef struct auv_msgs__msg__BehaviorStatus__Sequence
{
  auv_msgs__msg__BehaviorStatus * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} auv_msgs__msg__BehaviorStatus__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // AUV_MSGS__MSG__DETAIL__BEHAVIOR_STATUS__STRUCT_H_
