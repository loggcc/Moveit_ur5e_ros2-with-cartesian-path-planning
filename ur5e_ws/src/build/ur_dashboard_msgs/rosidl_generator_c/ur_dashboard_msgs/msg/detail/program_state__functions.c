// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from ur_dashboard_msgs:msg/ProgramState.idl
// generated code does not contain a copyright notice
#include "ur_dashboard_msgs/msg/detail/program_state__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


// Include directives for member types
// Member `state`
#include "rosidl_runtime_c/string_functions.h"

bool
ur_dashboard_msgs__msg__ProgramState__init(ur_dashboard_msgs__msg__ProgramState * msg)
{
  if (!msg) {
    return false;
  }
  // state
  if (!rosidl_runtime_c__String__init(&msg->state)) {
    ur_dashboard_msgs__msg__ProgramState__fini(msg);
    return false;
  }
  return true;
}

void
ur_dashboard_msgs__msg__ProgramState__fini(ur_dashboard_msgs__msg__ProgramState * msg)
{
  if (!msg) {
    return;
  }
  // state
  rosidl_runtime_c__String__fini(&msg->state);
}

bool
ur_dashboard_msgs__msg__ProgramState__are_equal(const ur_dashboard_msgs__msg__ProgramState * lhs, const ur_dashboard_msgs__msg__ProgramState * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // state
  if (!rosidl_runtime_c__String__are_equal(
      &(lhs->state), &(rhs->state)))
  {
    return false;
  }
  return true;
}

bool
ur_dashboard_msgs__msg__ProgramState__copy(
  const ur_dashboard_msgs__msg__ProgramState * input,
  ur_dashboard_msgs__msg__ProgramState * output)
{
  if (!input || !output) {
    return false;
  }
  // state
  if (!rosidl_runtime_c__String__copy(
      &(input->state), &(output->state)))
  {
    return false;
  }
  return true;
}

ur_dashboard_msgs__msg__ProgramState *
ur_dashboard_msgs__msg__ProgramState__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  ur_dashboard_msgs__msg__ProgramState * msg = (ur_dashboard_msgs__msg__ProgramState *)allocator.allocate(sizeof(ur_dashboard_msgs__msg__ProgramState), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(ur_dashboard_msgs__msg__ProgramState));
  bool success = ur_dashboard_msgs__msg__ProgramState__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
ur_dashboard_msgs__msg__ProgramState__destroy(ur_dashboard_msgs__msg__ProgramState * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    ur_dashboard_msgs__msg__ProgramState__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
ur_dashboard_msgs__msg__ProgramState__Sequence__init(ur_dashboard_msgs__msg__ProgramState__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  ur_dashboard_msgs__msg__ProgramState * data = NULL;

  if (size) {
    data = (ur_dashboard_msgs__msg__ProgramState *)allocator.zero_allocate(size, sizeof(ur_dashboard_msgs__msg__ProgramState), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = ur_dashboard_msgs__msg__ProgramState__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        ur_dashboard_msgs__msg__ProgramState__fini(&data[i - 1]);
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
ur_dashboard_msgs__msg__ProgramState__Sequence__fini(ur_dashboard_msgs__msg__ProgramState__Sequence * array)
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
      ur_dashboard_msgs__msg__ProgramState__fini(&array->data[i]);
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

ur_dashboard_msgs__msg__ProgramState__Sequence *
ur_dashboard_msgs__msg__ProgramState__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  ur_dashboard_msgs__msg__ProgramState__Sequence * array = (ur_dashboard_msgs__msg__ProgramState__Sequence *)allocator.allocate(sizeof(ur_dashboard_msgs__msg__ProgramState__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = ur_dashboard_msgs__msg__ProgramState__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
ur_dashboard_msgs__msg__ProgramState__Sequence__destroy(ur_dashboard_msgs__msg__ProgramState__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    ur_dashboard_msgs__msg__ProgramState__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
ur_dashboard_msgs__msg__ProgramState__Sequence__are_equal(const ur_dashboard_msgs__msg__ProgramState__Sequence * lhs, const ur_dashboard_msgs__msg__ProgramState__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!ur_dashboard_msgs__msg__ProgramState__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
ur_dashboard_msgs__msg__ProgramState__Sequence__copy(
  const ur_dashboard_msgs__msg__ProgramState__Sequence * input,
  ur_dashboard_msgs__msg__ProgramState__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(ur_dashboard_msgs__msg__ProgramState);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    ur_dashboard_msgs__msg__ProgramState * data =
      (ur_dashboard_msgs__msg__ProgramState *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!ur_dashboard_msgs__msg__ProgramState__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          ur_dashboard_msgs__msg__ProgramState__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!ur_dashboard_msgs__msg__ProgramState__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
