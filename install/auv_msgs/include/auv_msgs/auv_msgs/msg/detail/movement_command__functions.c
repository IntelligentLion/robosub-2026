// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from auv_msgs:msg/MovementCommand.idl
// generated code does not contain a copyright notice
#include "auv_msgs/msg/detail/movement_command__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


// Include directives for member types
// Member `command`
#include "rosidl_runtime_c/string_functions.h"

bool
auv_msgs__msg__MovementCommand__init(auv_msgs__msg__MovementCommand * msg)
{
  if (!msg) {
    return false;
  }
  // command
  if (!rosidl_runtime_c__String__init(&msg->command)) {
    auv_msgs__msg__MovementCommand__fini(msg);
    return false;
  }
  // speed
  // duration
  return true;
}

void
auv_msgs__msg__MovementCommand__fini(auv_msgs__msg__MovementCommand * msg)
{
  if (!msg) {
    return;
  }
  // command
  rosidl_runtime_c__String__fini(&msg->command);
  // speed
  // duration
}

bool
auv_msgs__msg__MovementCommand__are_equal(const auv_msgs__msg__MovementCommand * lhs, const auv_msgs__msg__MovementCommand * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // command
  if (!rosidl_runtime_c__String__are_equal(
      &(lhs->command), &(rhs->command)))
  {
    return false;
  }
  // speed
  if (lhs->speed != rhs->speed) {
    return false;
  }
  // duration
  if (lhs->duration != rhs->duration) {
    return false;
  }
  return true;
}

bool
auv_msgs__msg__MovementCommand__copy(
  const auv_msgs__msg__MovementCommand * input,
  auv_msgs__msg__MovementCommand * output)
{
  if (!input || !output) {
    return false;
  }
  // command
  if (!rosidl_runtime_c__String__copy(
      &(input->command), &(output->command)))
  {
    return false;
  }
  // speed
  output->speed = input->speed;
  // duration
  output->duration = input->duration;
  return true;
}

auv_msgs__msg__MovementCommand *
auv_msgs__msg__MovementCommand__create(void)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  auv_msgs__msg__MovementCommand * msg = (auv_msgs__msg__MovementCommand *)allocator.allocate(sizeof(auv_msgs__msg__MovementCommand), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(auv_msgs__msg__MovementCommand));
  bool success = auv_msgs__msg__MovementCommand__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
auv_msgs__msg__MovementCommand__destroy(auv_msgs__msg__MovementCommand * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    auv_msgs__msg__MovementCommand__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
auv_msgs__msg__MovementCommand__Sequence__init(auv_msgs__msg__MovementCommand__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  auv_msgs__msg__MovementCommand * data = NULL;

  if (size) {
    data = (auv_msgs__msg__MovementCommand *)allocator.zero_allocate(size, sizeof(auv_msgs__msg__MovementCommand), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = auv_msgs__msg__MovementCommand__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        auv_msgs__msg__MovementCommand__fini(&data[i - 1]);
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
auv_msgs__msg__MovementCommand__Sequence__fini(auv_msgs__msg__MovementCommand__Sequence * array)
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
      auv_msgs__msg__MovementCommand__fini(&array->data[i]);
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

auv_msgs__msg__MovementCommand__Sequence *
auv_msgs__msg__MovementCommand__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  auv_msgs__msg__MovementCommand__Sequence * array = (auv_msgs__msg__MovementCommand__Sequence *)allocator.allocate(sizeof(auv_msgs__msg__MovementCommand__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = auv_msgs__msg__MovementCommand__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
auv_msgs__msg__MovementCommand__Sequence__destroy(auv_msgs__msg__MovementCommand__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    auv_msgs__msg__MovementCommand__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
auv_msgs__msg__MovementCommand__Sequence__are_equal(const auv_msgs__msg__MovementCommand__Sequence * lhs, const auv_msgs__msg__MovementCommand__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!auv_msgs__msg__MovementCommand__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
auv_msgs__msg__MovementCommand__Sequence__copy(
  const auv_msgs__msg__MovementCommand__Sequence * input,
  auv_msgs__msg__MovementCommand__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(auv_msgs__msg__MovementCommand);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    auv_msgs__msg__MovementCommand * data =
      (auv_msgs__msg__MovementCommand *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!auv_msgs__msg__MovementCommand__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          auv_msgs__msg__MovementCommand__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!auv_msgs__msg__MovementCommand__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
