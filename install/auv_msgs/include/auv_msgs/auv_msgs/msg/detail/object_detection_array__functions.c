// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from auv_msgs:msg/ObjectDetectionArray.idl
// generated code does not contain a copyright notice
#include "auv_msgs/msg/detail/object_detection_array__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


// Include directives for member types
// Member `detections`
#include "auv_msgs/msg/detail/object_detection__functions.h"

bool
auv_msgs__msg__ObjectDetectionArray__init(auv_msgs__msg__ObjectDetectionArray * msg)
{
  if (!msg) {
    return false;
  }
  // detections
  if (!auv_msgs__msg__ObjectDetection__Sequence__init(&msg->detections, 0)) {
    auv_msgs__msg__ObjectDetectionArray__fini(msg);
    return false;
  }
  return true;
}

void
auv_msgs__msg__ObjectDetectionArray__fini(auv_msgs__msg__ObjectDetectionArray * msg)
{
  if (!msg) {
    return;
  }
  // detections
  auv_msgs__msg__ObjectDetection__Sequence__fini(&msg->detections);
}

bool
auv_msgs__msg__ObjectDetectionArray__are_equal(const auv_msgs__msg__ObjectDetectionArray * lhs, const auv_msgs__msg__ObjectDetectionArray * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // detections
  if (!auv_msgs__msg__ObjectDetection__Sequence__are_equal(
      &(lhs->detections), &(rhs->detections)))
  {
    return false;
  }
  return true;
}

bool
auv_msgs__msg__ObjectDetectionArray__copy(
  const auv_msgs__msg__ObjectDetectionArray * input,
  auv_msgs__msg__ObjectDetectionArray * output)
{
  if (!input || !output) {
    return false;
  }
  // detections
  if (!auv_msgs__msg__ObjectDetection__Sequence__copy(
      &(input->detections), &(output->detections)))
  {
    return false;
  }
  return true;
}

auv_msgs__msg__ObjectDetectionArray *
auv_msgs__msg__ObjectDetectionArray__create(void)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  auv_msgs__msg__ObjectDetectionArray * msg = (auv_msgs__msg__ObjectDetectionArray *)allocator.allocate(sizeof(auv_msgs__msg__ObjectDetectionArray), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(auv_msgs__msg__ObjectDetectionArray));
  bool success = auv_msgs__msg__ObjectDetectionArray__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
auv_msgs__msg__ObjectDetectionArray__destroy(auv_msgs__msg__ObjectDetectionArray * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    auv_msgs__msg__ObjectDetectionArray__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
auv_msgs__msg__ObjectDetectionArray__Sequence__init(auv_msgs__msg__ObjectDetectionArray__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  auv_msgs__msg__ObjectDetectionArray * data = NULL;

  if (size) {
    data = (auv_msgs__msg__ObjectDetectionArray *)allocator.zero_allocate(size, sizeof(auv_msgs__msg__ObjectDetectionArray), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = auv_msgs__msg__ObjectDetectionArray__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        auv_msgs__msg__ObjectDetectionArray__fini(&data[i - 1]);
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
auv_msgs__msg__ObjectDetectionArray__Sequence__fini(auv_msgs__msg__ObjectDetectionArray__Sequence * array)
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
      auv_msgs__msg__ObjectDetectionArray__fini(&array->data[i]);
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

auv_msgs__msg__ObjectDetectionArray__Sequence *
auv_msgs__msg__ObjectDetectionArray__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  auv_msgs__msg__ObjectDetectionArray__Sequence * array = (auv_msgs__msg__ObjectDetectionArray__Sequence *)allocator.allocate(sizeof(auv_msgs__msg__ObjectDetectionArray__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = auv_msgs__msg__ObjectDetectionArray__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
auv_msgs__msg__ObjectDetectionArray__Sequence__destroy(auv_msgs__msg__ObjectDetectionArray__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    auv_msgs__msg__ObjectDetectionArray__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
auv_msgs__msg__ObjectDetectionArray__Sequence__are_equal(const auv_msgs__msg__ObjectDetectionArray__Sequence * lhs, const auv_msgs__msg__ObjectDetectionArray__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!auv_msgs__msg__ObjectDetectionArray__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
auv_msgs__msg__ObjectDetectionArray__Sequence__copy(
  const auv_msgs__msg__ObjectDetectionArray__Sequence * input,
  auv_msgs__msg__ObjectDetectionArray__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(auv_msgs__msg__ObjectDetectionArray);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    auv_msgs__msg__ObjectDetectionArray * data =
      (auv_msgs__msg__ObjectDetectionArray *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!auv_msgs__msg__ObjectDetectionArray__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          auv_msgs__msg__ObjectDetectionArray__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!auv_msgs__msg__ObjectDetectionArray__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
