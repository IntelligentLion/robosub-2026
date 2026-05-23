// generated from rosidl_generator_c/resource/idl__description.c.em
// with input from auv_msgs:msg/MovementCommand.idl
// generated code does not contain a copyright notice

#include "auv_msgs/msg/detail/movement_command__functions.h"

ROSIDL_GENERATOR_C_PUBLIC_auv_msgs
const rosidl_type_hash_t *
auv_msgs__msg__MovementCommand__get_type_hash(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0xeb, 0x5b, 0x55, 0xea, 0x46, 0x73, 0x6e, 0x1f,
      0x8d, 0xf9, 0x97, 0x60, 0x5a, 0xf9, 0x31, 0x73,
      0x6c, 0xe2, 0xb8, 0x8a, 0x9b, 0x38, 0xcc, 0x2a,
      0xc4, 0x6b, 0x65, 0x20, 0xaf, 0x35, 0xb3, 0xb7,
    }};
  return &hash;
}

#include <assert.h>
#include <string.h>

// Include directives for referenced types

// Hashes for external referenced types
#ifndef NDEBUG
#endif

static char auv_msgs__msg__MovementCommand__TYPE_NAME[] = "auv_msgs/msg/MovementCommand";

// Define type names, field names, and default values
static char auv_msgs__msg__MovementCommand__FIELD_NAME__command[] = "command";
static char auv_msgs__msg__MovementCommand__FIELD_NAME__speed[] = "speed";
static char auv_msgs__msg__MovementCommand__FIELD_NAME__duration[] = "duration";

static rosidl_runtime_c__type_description__Field auv_msgs__msg__MovementCommand__FIELDS[] = {
  {
    {auv_msgs__msg__MovementCommand__FIELD_NAME__command, 7, 7},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_STRING,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {auv_msgs__msg__MovementCommand__FIELD_NAME__speed, 5, 5},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {auv_msgs__msg__MovementCommand__FIELD_NAME__duration, 8, 8},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
};

const rosidl_runtime_c__type_description__TypeDescription *
auv_msgs__msg__MovementCommand__get_type_description(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {auv_msgs__msg__MovementCommand__TYPE_NAME, 28, 28},
      {auv_msgs__msg__MovementCommand__FIELDS, 3, 3},
    },
    {NULL, 0, 0},
  };
  if (!constructed) {
    constructed = true;
  }
  return &description;
}

static char toplevel_type_raw_source[] =
  "# Modular movement command for thruster control\n"
  "#\n"
  "# Supported commands:\n"
  "#   submerge       - move downward          (speed: 0.0\\xe2\\x80\\x931.0)\n"
  "#   emerge         - move upward            (speed: 0.0\\xe2\\x80\\x931.0)\n"
  "#   surge_forward  - move forward           (speed: 0.0\\xe2\\x80\\x931.0)\n"
  "#   surge_backward - move backward          (speed: 0.0\\xe2\\x80\\x931.0)\n"
  "#   strafe_left    - move left              (speed: 0.0\\xe2\\x80\\x931.0)\n"
  "#   strafe_right   - move right             (speed: 0.0\\xe2\\x80\\x931.0)\n"
  "#   rotate_cw      - yaw clockwise          (speed: 0.0\\xe2\\x80\\x931.0)\n"
  "#   rotate_ccw     - yaw counter-clockwise  (speed: 0.0\\xe2\\x80\\x931.0)\n"
  "#   stop           - halt all movement\n"
  "#   depth_hold     - maintain current depth\n"
  "#\n"
  "string command\n"
  "float32 speed        # 0.0 to 1.0 normalized intensity\n"
  "float32 duration     # seconds (0 = until next command)";

static char msg_encoding[] = "msg";

// Define all individual source functions

const rosidl_runtime_c__type_description__TypeSource *
auv_msgs__msg__MovementCommand__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {auv_msgs__msg__MovementCommand__TYPE_NAME, 28, 28},
    {msg_encoding, 3, 3},
    {toplevel_type_raw_source, 771, 771},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
auv_msgs__msg__MovementCommand__get_type_description_sources(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[1];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 1, 1};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *auv_msgs__msg__MovementCommand__get_individual_type_description_source(NULL),
    constructed = true;
  }
  return &source_sequence;
}
