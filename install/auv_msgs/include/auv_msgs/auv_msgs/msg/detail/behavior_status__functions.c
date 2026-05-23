// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from auv_msgs:msg/BehaviorStatus.idl
// generated code does not contain a copyright notice
#include "auv_msgs/msg/detail/behavior_status__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


// Include directives for member types
// Member `stamp`
#include "builtin_interfaces/msg/detail/time__functions.h"
// Member `action_name`
// Member `status`
// Member `reason`
#include "rosidl_runtime_c/string_functions.h"

bool
auv_msgs__msg__BehaviorStatus__init(auv_msgs__msg__BehaviorStatus * msg)
{
  if (!msg) {
    return false;
  }
  // stamp
  if (!builtin_interfaces__msg__Time__init(&msg->stamp)) {
    auv_msgs__msg__BehaviorStatus__fini(msg);
    return false;
  }
  // action_name
  if (!rosidl_runtime_c__String__init(&msg->action_name)) {
    auv_msgs__msg__BehaviorStatus__fini(msg);
    return false;
  }
  // status
  if (!rosidl_runtime_c__String__init(&msg->status)) {
    auv_msgs__msg__BehaviorStatus__fini(msg);
    return false;
  }
  // reason
  if (!rosidl_runtime_c__String__init(&msg->reason)) {
    auv_msgs__msg__BehaviorStatus__fini(msg);
    return false;
  }
  return true;
}

void
auv_msgs__msg__BehaviorStatus__fini(auv_msgs__msg__BehaviorStatus * msg)
{
  if (!msg) {
    return;
  }
  // stamp
  builtin_interfaces__msg__Time__fini(&msg->stamp);
  // action_name
  rosidl_runtime_c__String__fini(&msg->action_name);
  // status
  rosidl_runtime_c__String__fini(&msg->status);
  // reason
  rosidl_runtime_c__String__fini(&msg->reason);
}

bool
auv_msgs__msg__BehaviorStatus__are_equal(const auv_msgs__msg__BehaviorStatus * lhs, const auv_msgs__msg__BehaviorStatus * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // stamp
  if (!builtin_interfaces__msg__Time__are_equal(
      &(lhs->stamp), &(rhs->stamp)))
  {
    return false;
  }
  // action_name
  if (!rosidl_runtime_c__String__are_equal(
      &(lhs->action_name), &(rhs->action_name)))
  {
    return false;
  }
  // status
  if (!rosidl_runtime_c__String__are_equal(
      &(lhs->status), &(rhs->status)))
  {
    return false;
  }
  // reason
  if (!rosidl_runtime_c__String__are_equal(
      &(lhs->reason), &(rhs->reason)))
  {
    return false;
  }
  return true;
}

bool
auv_msgs__msg__BehaviorStatus__copy(
  const auv_msgs__msg__BehaviorStatus * input,
  auv_msgs__msg__BehaviorStatus * output)
{
  if (!input || !output) {
    return false;
  }
  // stamp
  if (!builtin_interfaces__msg__Time__copy(
      &(input->stamp), &(output->stamp)))
  {
    return false;
  }
  // action_name
  if (!rosidl_runtime_c__String__copy(
      &(input->action_name), &(output->action_name)))
  {
    return false;
  }
  // status
  if (!rosidl_runtime_c__String__copy(
      &(input->status), &(output->status)))
  {
    return false;
  }
  // reason
  if (!rosidl_runtime_c__String__copy(
      &(input->reason), &(output->reason)))
  {
    return false;
  }
  return true;
}

auv_msgs__msg__BehaviorStatus *
auv_msgs__msg__BehaviorStatus__create(void)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  auv_msgs__msg__BehaviorStatus * msg = (auv_msgs__msg__BehaviorStatus *)allocator.allocate(sizeof(auv_msgs__msg__BehaviorStatus), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(auv_msgs__msg__BehaviorStatus));
  bool success = auv_msgs__msg__BehaviorStatus__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
auv_msgs__msg__BehaviorStatus__destroy(auv_msgs__msg__BehaviorStatus * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    auv_msgs__msg__BehaviorStatus__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
auv_msgs__msg__BehaviorStatus__Sequence__init(auv_msgs__msg__BehaviorStatus__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  auv_msgs__msg__BehaviorStatus * data = NULL;

  if (size) {
    data = (auv_msgs__msg__BehaviorStatus *)allocator.zero_allocate(size, sizeof(auv_msgs__msg__BehaviorStatus), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = auv_msgs__msg__BehaviorStatus__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        auv_msgs__msg__BehaviorStatus__fini(&data[i - 1]);
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
auv_msgs__msg__BehaviorStatus__Sequence__fini(auv_msgs__msg__BehaviorStatus__Sequence * array)
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
      auv_msgs__msg__BehaviorStatus__fini(&array->data[i]);
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

auv_msgs__msg__BehaviorStatus__Sequence *
auv_msgs__msg__BehaviorStatus__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  auv_msgs__msg__BehaviorStatus__Sequence * array = (auv_msgs__msg__BehaviorStatus__Sequence *)allocator.allocate(sizeof(auv_msgs__msg__BehaviorStatus__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = auv_msgs__msg__BehaviorStatus__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
auv_msgs__msg__BehaviorStatus__Sequence__destroy(auv_msgs__msg__BehaviorStatus__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    auv_msgs__msg__BehaviorStatus__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
auv_msgs__msg__BehaviorStatus__Sequence__are_equal(const auv_msgs__msg__BehaviorStatus__Sequence * lhs, const auv_msgs__msg__BehaviorStatus__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!auv_msgs__msg__BehaviorStatus__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
auv_msgs__msg__BehaviorStatus__Sequence__copy(
  const auv_msgs__msg__BehaviorStatus__Sequence * input,
  auv_msgs__msg__BehaviorStatus__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(auv_msgs__msg__BehaviorStatus);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    auv_msgs__msg__BehaviorStatus * data =
      (auv_msgs__msg__BehaviorStatus *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!auv_msgs__msg__BehaviorStatus__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          auv_msgs__msg__BehaviorStatus__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!auv_msgs__msg__BehaviorStatus__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
