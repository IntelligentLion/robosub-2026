// NOLINT: This file starts with a BOM since it contain non-ASCII characters
// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from auv_msgs:msg/MovementCommand.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "auv_msgs/msg/movement_command.h"


#ifndef AUV_MSGS__MSG__DETAIL__MOVEMENT_COMMAND__STRUCT_H_
#define AUV_MSGS__MSG__DETAIL__MOVEMENT_COMMAND__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

// Constants defined in the message

// Include directives for member types
// Member 'command'
#include "rosidl_runtime_c/string.h"

/// Struct defined in msg/MovementCommand in the package auv_msgs.
/**
  * Modular movement command for thruster control
  *
  * Supported commands:
  *   submerge       - move downward          (speed: 0.0–1.0)
  *   emerge         - move upward            (speed: 0.0–1.0)
  *   surge_forward  - move forward           (speed: 0.0–1.0)
  *   surge_backward - move backward          (speed: 0.0–1.0)
  *   strafe_left    - move left              (speed: 0.0–1.0)
  *   strafe_right   - move right             (speed: 0.0–1.0)
  *   rotate_cw      - yaw clockwise          (speed: 0.0–1.0)
  *   rotate_ccw     - yaw counter-clockwise  (speed: 0.0–1.0)
  *   stop           - halt all movement
  *   depth_hold     - maintain current depth
 */
typedef struct auv_msgs__msg__MovementCommand
{
  rosidl_runtime_c__String command;
  /// 0.0 to 1.0 normalized intensity
  float speed;
  /// seconds (0 = until next command)
  float duration;
} auv_msgs__msg__MovementCommand;

// Struct for a sequence of auv_msgs__msg__MovementCommand.
typedef struct auv_msgs__msg__MovementCommand__Sequence
{
  auv_msgs__msg__MovementCommand * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} auv_msgs__msg__MovementCommand__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // AUV_MSGS__MSG__DETAIL__MOVEMENT_COMMAND__STRUCT_H_
