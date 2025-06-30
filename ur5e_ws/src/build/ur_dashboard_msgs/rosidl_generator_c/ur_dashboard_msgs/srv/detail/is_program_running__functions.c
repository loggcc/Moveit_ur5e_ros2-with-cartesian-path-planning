// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from ur_dashboard_msgs:srv/IsProgramRunning.idl
// generated code does not contain a copyright notice
#include "ur_dashboard_msgs/srv/detail/is_program_running__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"

bool
ur_dashboard_msgs__srv__IsProgramRunning_Request__init(ur_dashboard_msgs__srv__IsProgramRunning_Request * msg)
{
  if (!msg) {
    return false;
  }
  // structure_needs_at_least_one_member
  return true;
}

void
ur_dashboard_msgs__srv__IsProgramRunning_Request__fini(ur_dashboard_msgs__srv__IsProgramRunning_Request * msg)
{
  if (!msg) {
    return;
  }
  // structure_needs_at_least_one_member
}

bool
ur_dashboard_msgs__srv__IsProgramRunning_Request__are_equal(const ur_dashboard_msgs__srv__IsProgramRunning_Request * lhs, const ur_dashboard_msgs__srv__IsProgramRunning_Request * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // structure_needs_at_least_one_member
  if (lhs->structure_needs_at_least_one_member != rhs->structure_needs_at_least_one_member) {
    return false;
  }
  return true;
}

bool
ur_dashboard_msgs__srv__IsProgramRunning_Request__copy(
  const ur_dashboard_msgs__srv__IsProgramRunning_Request * input,
  ur_dashboard_msgs__srv__IsProgramRunning_Request * output)
{
  if (!input || !output) {
    return false;
  }
  // structure_needs_at_least_one_member
  output->structure_needs_at_least_one_member = input->structure_needs_at_least_one_member;
  return true;
}

ur_dashboard_msgs__srv__IsProgramRunning_Request *
ur_dashboard_msgs__srv__IsProgramRunning_Request__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  ur_dashboard_msgs__srv__IsProgramRunning_Request * msg = (ur_dashboard_msgs__srv__IsProgramRunning_Request *)allocator.allocate(sizeof(ur_dashboard_msgs__srv__IsProgramRunning_Request), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(ur_dashboard_msgs__srv__IsProgramRunning_Request));
  bool success = ur_dashboard_msgs__srv__IsProgramRunning_Request__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
ur_dashboard_msgs__srv__IsProgramRunning_Request__destroy(ur_dashboard_msgs__srv__IsProgramRunning_Request * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    ur_dashboard_msgs__srv__IsProgramRunning_Request__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
ur_dashboard_msgs__srv__IsProgramRunning_Request__Sequence__init(ur_dashboard_msgs__srv__IsProgramRunning_Request__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  ur_dashboard_msgs__srv__IsProgramRunning_Request * data = NULL;

  if (size) {
    data = (ur_dashboard_msgs__srv__IsProgramRunning_Request *)allocator.zero_allocate(size, sizeof(ur_dashboard_msgs__srv__IsProgramRunning_Request), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = ur_dashboard_msgs__srv__IsProgramRunning_Request__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        ur_dashboard_msgs__srv__IsProgramRunning_Request__fini(&data[i - 1]);
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
ur_dashboard_msgs__srv__IsProgramRunning_Request__Sequence__fini(ur_dashboard_msgs__srv__IsProgramRunning_Request__Sequence * array)
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
      ur_dashboard_msgs__srv__IsProgramRunning_Request__fini(&array->data[i]);
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

ur_dashboard_msgs__srv__IsProgramRunning_Request__Sequence *
ur_dashboard_msgs__srv__IsProgramRunning_Request__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  ur_dashboard_msgs__srv__IsProgramRunning_Request__Sequence * array = (ur_dashboard_msgs__srv__IsProgramRunning_Request__Sequence *)allocator.allocate(sizeof(ur_dashboard_msgs__srv__IsProgramRunning_Request__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = ur_dashboard_msgs__srv__IsProgramRunning_Request__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
ur_dashboard_msgs__srv__IsProgramRunning_Request__Sequence__destroy(ur_dashboard_msgs__srv__IsProgramRunning_Request__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    ur_dashboard_msgs__srv__IsProgramRunning_Request__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
ur_dashboard_msgs__srv__IsProgramRunning_Request__Sequence__are_equal(const ur_dashboard_msgs__srv__IsProgramRunning_Request__Sequence * lhs, const ur_dashboard_msgs__srv__IsProgramRunning_Request__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!ur_dashboard_msgs__srv__IsProgramRunning_Request__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
ur_dashboard_msgs__srv__IsProgramRunning_Request__Sequence__copy(
  const ur_dashboard_msgs__srv__IsProgramRunning_Request__Sequence * input,
  ur_dashboard_msgs__srv__IsProgramRunning_Request__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(ur_dashboard_msgs__srv__IsProgramRunning_Request);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    ur_dashboard_msgs__srv__IsProgramRunning_Request * data =
      (ur_dashboard_msgs__srv__IsProgramRunning_Request *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!ur_dashboard_msgs__srv__IsProgramRunning_Request__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          ur_dashboard_msgs__srv__IsProgramRunning_Request__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!ur_dashboard_msgs__srv__IsProgramRunning_Request__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}


// Include directives for member types
// Member `answer`
#include "rosidl_runtime_c/string_functions.h"

bool
ur_dashboard_msgs__srv__IsProgramRunning_Response__init(ur_dashboard_msgs__srv__IsProgramRunning_Response * msg)
{
  if (!msg) {
    return false;
  }
  // answer
  if (!rosidl_runtime_c__String__init(&msg->answer)) {
    ur_dashboard_msgs__srv__IsProgramRunning_Response__fini(msg);
    return false;
  }
  // program_running
  // success
  return true;
}

void
ur_dashboard_msgs__srv__IsProgramRunning_Response__fini(ur_dashboard_msgs__srv__IsProgramRunning_Response * msg)
{
  if (!msg) {
    return;
  }
  // answer
  rosidl_runtime_c__String__fini(&msg->answer);
  // program_running
  // success
}

bool
ur_dashboard_msgs__srv__IsProgramRunning_Response__are_equal(const ur_dashboard_msgs__srv__IsProgramRunning_Response * lhs, const ur_dashboard_msgs__srv__IsProgramRunning_Response * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // answer
  if (!rosidl_runtime_c__String__are_equal(
      &(lhs->answer), &(rhs->answer)))
  {
    return false;
  }
  // program_running
  if (lhs->program_running != rhs->program_running) {
    return false;
  }
  // success
  if (lhs->success != rhs->success) {
    return false;
  }
  return true;
}

bool
ur_dashboard_msgs__srv__IsProgramRunning_Response__copy(
  const ur_dashboard_msgs__srv__IsProgramRunning_Response * input,
  ur_dashboard_msgs__srv__IsProgramRunning_Response * output)
{
  if (!input || !output) {
    return false;
  }
  // answer
  if (!rosidl_runtime_c__String__copy(
      &(input->answer), &(output->answer)))
  {
    return false;
  }
  // program_running
  output->program_running = input->program_running;
  // success
  output->success = input->success;
  return true;
}

ur_dashboard_msgs__srv__IsProgramRunning_Response *
ur_dashboard_msgs__srv__IsProgramRunning_Response__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  ur_dashboard_msgs__srv__IsProgramRunning_Response * msg = (ur_dashboard_msgs__srv__IsProgramRunning_Response *)allocator.allocate(sizeof(ur_dashboard_msgs__srv__IsProgramRunning_Response), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(ur_dashboard_msgs__srv__IsProgramRunning_Response));
  bool success = ur_dashboard_msgs__srv__IsProgramRunning_Response__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
ur_dashboard_msgs__srv__IsProgramRunning_Response__destroy(ur_dashboard_msgs__srv__IsProgramRunning_Response * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    ur_dashboard_msgs__srv__IsProgramRunning_Response__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
ur_dashboard_msgs__srv__IsProgramRunning_Response__Sequence__init(ur_dashboard_msgs__srv__IsProgramRunning_Response__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  ur_dashboard_msgs__srv__IsProgramRunning_Response * data = NULL;

  if (size) {
    data = (ur_dashboard_msgs__srv__IsProgramRunning_Response *)allocator.zero_allocate(size, sizeof(ur_dashboard_msgs__srv__IsProgramRunning_Response), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = ur_dashboard_msgs__srv__IsProgramRunning_Response__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        ur_dashboard_msgs__srv__IsProgramRunning_Response__fini(&data[i - 1]);
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
ur_dashboard_msgs__srv__IsProgramRunning_Response__Sequence__fini(ur_dashboard_msgs__srv__IsProgramRunning_Response__Sequence * array)
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
      ur_dashboard_msgs__srv__IsProgramRunning_Response__fini(&array->data[i]);
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

ur_dashboard_msgs__srv__IsProgramRunning_Response__Sequence *
ur_dashboard_msgs__srv__IsProgramRunning_Response__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  ur_dashboard_msgs__srv__IsProgramRunning_Response__Sequence * array = (ur_dashboard_msgs__srv__IsProgramRunning_Response__Sequence *)allocator.allocate(sizeof(ur_dashboard_msgs__srv__IsProgramRunning_Response__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = ur_dashboard_msgs__srv__IsProgramRunning_Response__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
ur_dashboard_msgs__srv__IsProgramRunning_Response__Sequence__destroy(ur_dashboard_msgs__srv__IsProgramRunning_Response__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    ur_dashboard_msgs__srv__IsProgramRunning_Response__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
ur_dashboard_msgs__srv__IsProgramRunning_Response__Sequence__are_equal(const ur_dashboard_msgs__srv__IsProgramRunning_Response__Sequence * lhs, const ur_dashboard_msgs__srv__IsProgramRunning_Response__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!ur_dashboard_msgs__srv__IsProgramRunning_Response__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
ur_dashboard_msgs__srv__IsProgramRunning_Response__Sequence__copy(
  const ur_dashboard_msgs__srv__IsProgramRunning_Response__Sequence * input,
  ur_dashboard_msgs__srv__IsProgramRunning_Response__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(ur_dashboard_msgs__srv__IsProgramRunning_Response);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    ur_dashboard_msgs__srv__IsProgramRunning_Response * data =
      (ur_dashboard_msgs__srv__IsProgramRunning_Response *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!ur_dashboard_msgs__srv__IsProgramRunning_Response__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          ur_dashboard_msgs__srv__IsProgramRunning_Response__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!ur_dashboard_msgs__srv__IsProgramRunning_Response__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
