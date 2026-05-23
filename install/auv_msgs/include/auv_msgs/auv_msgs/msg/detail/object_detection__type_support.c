// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from auv_msgs:msg/ObjectDetection.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "auv_msgs/msg/detail/object_detection__rosidl_typesupport_introspection_c.h"
#include "auv_msgs/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "auv_msgs/msg/detail/object_detection__functions.h"
#include "auv_msgs/msg/detail/object_detection__struct.h"


// Include directives for member types
// Member `label`
#include "rosidl_runtime_c/string_functions.h"
// Member `position`
#include "geometry_msgs/msg/point.h"
// Member `position`
#include "geometry_msgs/msg/detail/point__rosidl_typesupport_introspection_c.h"

#ifdef __cplusplus
extern "C"
{
#endif

void auv_msgs__msg__ObjectDetection__rosidl_typesupport_introspection_c__ObjectDetection_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  auv_msgs__msg__ObjectDetection__init(message_memory);
}

void auv_msgs__msg__ObjectDetection__rosidl_typesupport_introspection_c__ObjectDetection_fini_function(void * message_memory)
{
  auv_msgs__msg__ObjectDetection__fini(message_memory);
}

static rosidl_typesupport_introspection_c__MessageMember auv_msgs__msg__ObjectDetection__rosidl_typesupport_introspection_c__ObjectDetection_message_member_array[5] = {
  {
    "label",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_STRING,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(auv_msgs__msg__ObjectDetection, label),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "confidence",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_FLOAT,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(auv_msgs__msg__ObjectDetection, confidence),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "position",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(auv_msgs__msg__ObjectDetection, position),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "bbox_width",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_FLOAT,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(auv_msgs__msg__ObjectDetection, bbox_width),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "bbox_height",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_FLOAT,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(auv_msgs__msg__ObjectDetection, bbox_height),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers auv_msgs__msg__ObjectDetection__rosidl_typesupport_introspection_c__ObjectDetection_message_members = {
  "auv_msgs__msg",  // message namespace
  "ObjectDetection",  // message name
  5,  // number of fields
  sizeof(auv_msgs__msg__ObjectDetection),
  false,  // has_any_key_member_
  auv_msgs__msg__ObjectDetection__rosidl_typesupport_introspection_c__ObjectDetection_message_member_array,  // message members
  auv_msgs__msg__ObjectDetection__rosidl_typesupport_introspection_c__ObjectDetection_init_function,  // function to initialize message memory (memory has to be allocated)
  auv_msgs__msg__ObjectDetection__rosidl_typesupport_introspection_c__ObjectDetection_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t auv_msgs__msg__ObjectDetection__rosidl_typesupport_introspection_c__ObjectDetection_message_type_support_handle = {
  0,
  &auv_msgs__msg__ObjectDetection__rosidl_typesupport_introspection_c__ObjectDetection_message_members,
  get_message_typesupport_handle_function,
  &auv_msgs__msg__ObjectDetection__get_type_hash,
  &auv_msgs__msg__ObjectDetection__get_type_description,
  &auv_msgs__msg__ObjectDetection__get_type_description_sources,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_auv_msgs
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, auv_msgs, msg, ObjectDetection)() {
  auv_msgs__msg__ObjectDetection__rosidl_typesupport_introspection_c__ObjectDetection_message_member_array[2].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, geometry_msgs, msg, Point)();
  if (!auv_msgs__msg__ObjectDetection__rosidl_typesupport_introspection_c__ObjectDetection_message_type_support_handle.typesupport_identifier) {
    auv_msgs__msg__ObjectDetection__rosidl_typesupport_introspection_c__ObjectDetection_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &auv_msgs__msg__ObjectDetection__rosidl_typesupport_introspection_c__ObjectDetection_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif
