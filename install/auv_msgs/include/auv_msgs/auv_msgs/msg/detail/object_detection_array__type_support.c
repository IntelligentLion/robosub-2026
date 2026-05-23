// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from auv_msgs:msg/ObjectDetectionArray.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "auv_msgs/msg/detail/object_detection_array__rosidl_typesupport_introspection_c.h"
#include "auv_msgs/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "auv_msgs/msg/detail/object_detection_array__functions.h"
#include "auv_msgs/msg/detail/object_detection_array__struct.h"


// Include directives for member types
// Member `detections`
#include "auv_msgs/msg/object_detection.h"
// Member `detections`
#include "auv_msgs/msg/detail/object_detection__rosidl_typesupport_introspection_c.h"

#ifdef __cplusplus
extern "C"
{
#endif

void auv_msgs__msg__ObjectDetectionArray__rosidl_typesupport_introspection_c__ObjectDetectionArray_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  auv_msgs__msg__ObjectDetectionArray__init(message_memory);
}

void auv_msgs__msg__ObjectDetectionArray__rosidl_typesupport_introspection_c__ObjectDetectionArray_fini_function(void * message_memory)
{
  auv_msgs__msg__ObjectDetectionArray__fini(message_memory);
}

size_t auv_msgs__msg__ObjectDetectionArray__rosidl_typesupport_introspection_c__size_function__ObjectDetectionArray__detections(
  const void * untyped_member)
{
  const auv_msgs__msg__ObjectDetection__Sequence * member =
    (const auv_msgs__msg__ObjectDetection__Sequence *)(untyped_member);
  return member->size;
}

const void * auv_msgs__msg__ObjectDetectionArray__rosidl_typesupport_introspection_c__get_const_function__ObjectDetectionArray__detections(
  const void * untyped_member, size_t index)
{
  const auv_msgs__msg__ObjectDetection__Sequence * member =
    (const auv_msgs__msg__ObjectDetection__Sequence *)(untyped_member);
  return &member->data[index];
}

void * auv_msgs__msg__ObjectDetectionArray__rosidl_typesupport_introspection_c__get_function__ObjectDetectionArray__detections(
  void * untyped_member, size_t index)
{
  auv_msgs__msg__ObjectDetection__Sequence * member =
    (auv_msgs__msg__ObjectDetection__Sequence *)(untyped_member);
  return &member->data[index];
}

void auv_msgs__msg__ObjectDetectionArray__rosidl_typesupport_introspection_c__fetch_function__ObjectDetectionArray__detections(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const auv_msgs__msg__ObjectDetection * item =
    ((const auv_msgs__msg__ObjectDetection *)
    auv_msgs__msg__ObjectDetectionArray__rosidl_typesupport_introspection_c__get_const_function__ObjectDetectionArray__detections(untyped_member, index));
  auv_msgs__msg__ObjectDetection * value =
    (auv_msgs__msg__ObjectDetection *)(untyped_value);
  *value = *item;
}

void auv_msgs__msg__ObjectDetectionArray__rosidl_typesupport_introspection_c__assign_function__ObjectDetectionArray__detections(
  void * untyped_member, size_t index, const void * untyped_value)
{
  auv_msgs__msg__ObjectDetection * item =
    ((auv_msgs__msg__ObjectDetection *)
    auv_msgs__msg__ObjectDetectionArray__rosidl_typesupport_introspection_c__get_function__ObjectDetectionArray__detections(untyped_member, index));
  const auv_msgs__msg__ObjectDetection * value =
    (const auv_msgs__msg__ObjectDetection *)(untyped_value);
  *item = *value;
}

bool auv_msgs__msg__ObjectDetectionArray__rosidl_typesupport_introspection_c__resize_function__ObjectDetectionArray__detections(
  void * untyped_member, size_t size)
{
  auv_msgs__msg__ObjectDetection__Sequence * member =
    (auv_msgs__msg__ObjectDetection__Sequence *)(untyped_member);
  auv_msgs__msg__ObjectDetection__Sequence__fini(member);
  return auv_msgs__msg__ObjectDetection__Sequence__init(member, size);
}

static rosidl_typesupport_introspection_c__MessageMember auv_msgs__msg__ObjectDetectionArray__rosidl_typesupport_introspection_c__ObjectDetectionArray_message_member_array[1] = {
  {
    "detections",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is key
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(auv_msgs__msg__ObjectDetectionArray, detections),  // bytes offset in struct
    NULL,  // default value
    auv_msgs__msg__ObjectDetectionArray__rosidl_typesupport_introspection_c__size_function__ObjectDetectionArray__detections,  // size() function pointer
    auv_msgs__msg__ObjectDetectionArray__rosidl_typesupport_introspection_c__get_const_function__ObjectDetectionArray__detections,  // get_const(index) function pointer
    auv_msgs__msg__ObjectDetectionArray__rosidl_typesupport_introspection_c__get_function__ObjectDetectionArray__detections,  // get(index) function pointer
    auv_msgs__msg__ObjectDetectionArray__rosidl_typesupport_introspection_c__fetch_function__ObjectDetectionArray__detections,  // fetch(index, &value) function pointer
    auv_msgs__msg__ObjectDetectionArray__rosidl_typesupport_introspection_c__assign_function__ObjectDetectionArray__detections,  // assign(index, value) function pointer
    auv_msgs__msg__ObjectDetectionArray__rosidl_typesupport_introspection_c__resize_function__ObjectDetectionArray__detections  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers auv_msgs__msg__ObjectDetectionArray__rosidl_typesupport_introspection_c__ObjectDetectionArray_message_members = {
  "auv_msgs__msg",  // message namespace
  "ObjectDetectionArray",  // message name
  1,  // number of fields
  sizeof(auv_msgs__msg__ObjectDetectionArray),
  false,  // has_any_key_member_
  auv_msgs__msg__ObjectDetectionArray__rosidl_typesupport_introspection_c__ObjectDetectionArray_message_member_array,  // message members
  auv_msgs__msg__ObjectDetectionArray__rosidl_typesupport_introspection_c__ObjectDetectionArray_init_function,  // function to initialize message memory (memory has to be allocated)
  auv_msgs__msg__ObjectDetectionArray__rosidl_typesupport_introspection_c__ObjectDetectionArray_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t auv_msgs__msg__ObjectDetectionArray__rosidl_typesupport_introspection_c__ObjectDetectionArray_message_type_support_handle = {
  0,
  &auv_msgs__msg__ObjectDetectionArray__rosidl_typesupport_introspection_c__ObjectDetectionArray_message_members,
  get_message_typesupport_handle_function,
  &auv_msgs__msg__ObjectDetectionArray__get_type_hash,
  &auv_msgs__msg__ObjectDetectionArray__get_type_description,
  &auv_msgs__msg__ObjectDetectionArray__get_type_description_sources,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_auv_msgs
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, auv_msgs, msg, ObjectDetectionArray)() {
  auv_msgs__msg__ObjectDetectionArray__rosidl_typesupport_introspection_c__ObjectDetectionArray_message_member_array[0].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, auv_msgs, msg, ObjectDetection)();
  if (!auv_msgs__msg__ObjectDetectionArray__rosidl_typesupport_introspection_c__ObjectDetectionArray_message_type_support_handle.typesupport_identifier) {
    auv_msgs__msg__ObjectDetectionArray__rosidl_typesupport_introspection_c__ObjectDetectionArray_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &auv_msgs__msg__ObjectDetectionArray__rosidl_typesupport_introspection_c__ObjectDetectionArray_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif
