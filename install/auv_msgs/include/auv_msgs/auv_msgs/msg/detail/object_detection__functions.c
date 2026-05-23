// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from auv_msgs:msg/ObjectDetection.idl
// generated code does not contain a copyright notice
#include "auv_msgs/msg/detail/object_detection__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


// Include directives for member types
// Member `label`
#include "rosidl_runtime_c/string_functions.h"
// Member `position`
#include "geometry_msgs/msg/detail/point__functions.h"

bool
auv_msgs__msg__ObjectDetection__init(auv_msgs__msg__ObjectDetection * msg)
{
  if (!msg) {
    return false;
  }
  // label
  if (!rosidl_runtime_c__String__init(&msg->label)) {
    auv_msgs__msg__ObjectDetection__fini(msg);
    return false;
  }
  // confidence
  // position
  if (!geometry_msgs__msg__Point__init(&msg->position)) {
    auv_msgs__msg__ObjectDetection__fini(msg);
    return false;
  }
  // bbox_width
  // bbox_height
  return true;
}

void
auv_msgs__msg__ObjectDetection__fini(auv_msgs__msg__ObjectDetection * msg)
{
  if (!msg) {
    return;
  }
  // label
  rosidl_runtime_c__String__fini(&msg->label);
  // confidence
  // position
  geometry_msgs__msg__Point__fini(&msg->position);
  // bbox_width
  // bbox_height
}

bool
auv_msgs__msg__ObjectDetection__are_equal(const auv_msgs__msg__ObjectDetection * lhs, const auv_msgs__msg__ObjectDetection * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // label
  if (!rosidl_runtime_c__String__are_equal(
      &(lhs->label), &(rhs->label)))
  {
    return false;
  }
  // confidence
  if (lhs->confidence != rhs->confidence) {
    return false;
  }
  // position
  if (!geometry_msgs__msg__Point__are_equal(
      &(lhs->position), &(rhs->position)))
  {
    return false;
  }
  // bbox_width
  if (lhs->bbox_width != rhs->bbox_width) {
    return false;
  }
  // bbox_height
  if (lhs->bbox_height != rhs->bbox_height) {
    return false;
  }
  return true;
}

bool
auv_msgs__msg__ObjectDetection__copy(
  const auv_msgs__msg__ObjectDetection * input,
  auv_msgs__msg__ObjectDetection * output)
{
  if (!input || !output) {
    return false;
  }
  // label
  if (!rosidl_runtime_c__String__copy(
      &(input->label), &(output->label)))
  {
    return false;
  }
  // confidence
  output->confidence = input->confidence;
  // position
  if (!geometry_msgs__msg__Point__copy(
      &(input->position), &(output->position)))
  {
    return false;
  }
  // bbox_width
  output->bbox_width = input->bbox_width;
  // bbox_height
  output->bbox_height = input->bbox_height;
  return true;
}

auv_msgs__msg__ObjectDetection *
auv_msgs__msg__ObjectDetection__create(void)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  auv_msgs__msg__ObjectDetection * msg = (auv_msgs__msg__ObjectDetection *)allocator.allocate(sizeof(auv_msgs__msg__ObjectDetection), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(auv_msgs__msg__ObjectDetection));
  bool success = auv_msgs__msg__ObjectDetection__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
auv_msgs__msg__ObjectDetection__destroy(auv_msgs__msg__ObjectDetection * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    auv_msgs__msg__ObjectDetection__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
auv_msgs__msg__ObjectDetection__Sequence__init(auv_msgs__msg__ObjectDetection__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  auv_msgs__msg__ObjectDetection * data = NULL;

  if (size) {
    data = (auv_msgs__msg__ObjectDetection *)allocator.zero_allocate(size, sizeof(auv_msgs__msg__ObjectDetection), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = auv_msgs__msg__ObjectDetection__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        auv_msgs__msg__ObjectDetection__fini(&data[i - 1]);
      }
      allocator.deallocate(data, allocator.state);
      return false;
    }
  }
  array->data = data;
  array->size = size;
  array->capacity = size;
  return true;
}

void
auv_msgs__msg__ObjectDetection__Sequence__fini(auv_msgs__msg__ObjectDetection__Sequence * array)
{
  if (!array) {
    return;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();

  if (array->data) {
    // ensure that data and capacity values are consistent
    assert(array->capacity > 0);
    // finalize all array elements
    for (size_t i = 0; i < array->capacity; ++i) {
      auv_msgs__msg__ObjectDetection__fini(&array->data[i]);
    }
    allocator.deallocate(array->data, allocator.state);
    array->data = NULL;
    array->size = 0;
    array->capacity = 0;
  } else {
    // ensure that data, size, and capacity values are consistent
    assert(0 == array->size);
    assert(0 == array->capacity);
  }
}

auv_msgs__msg__ObjectDetection__Sequence *
auv_msgs__msg__ObjectDetection__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  auv_msgs__msg__ObjectDetection__Sequence * array = (auv_msgs__msg__ObjectDetection__Sequence *)allocator.allocate(sizeof(auv_msgs__msg__ObjectDetection__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = auv_msgs__msg__ObjectDetection__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
auv_msgs__msg__ObjectDetection__Sequence__destroy(auv_msgs__msg__ObjectDetection__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    auv_msgs__msg__ObjectDetection__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
auv_msgs__msg__ObjectDetection__Sequence__are_equal(const auv_msgs__msg__ObjectDetection__Sequence * lhs, const auv_msgs__msg__ObjectDetection__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!auv_msgs__msg__ObjectDetection__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
auv_msgs__msg__ObjectDetection__Sequence__copy(
  const auv_msgs__msg__ObjectDetection__Sequence * input,
  auv_msgs__msg__ObjectDetection__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(auv_msgs__msg__ObjectDetection);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    auv_msgs__msg__ObjectDetection * data =
      (auv_msgs__msg__ObjectDetection *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!auv_msgs__msg__ObjectDetection__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          auv_msgs__msg__ObjectDetection__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!auv_msgs__msg__ObjectDetection__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
