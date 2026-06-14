// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from uv_msgs:msg/TaskStatus.idl
// generated code does not contain a copyright notice
#include "uv_msgs/msg/detail/task_status__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


// Include directives for member types
// Member `current_task_name`
// Member `error_message`
#include "rosidl_runtime_c/string_functions.h"

bool
uv_msgs__msg__TaskStatus__init(uv_msgs__msg__TaskStatus * msg)
{
  if (!msg) {
    return false;
  }
  // status
  // current_task_index
  // total_tasks
  // current_task_name
  if (!rosidl_runtime_c__String__init(&msg->current_task_name)) {
    uv_msgs__msg__TaskStatus__fini(msg);
    return false;
  }
  // error_message
  if (!rosidl_runtime_c__String__init(&msg->error_message)) {
    uv_msgs__msg__TaskStatus__fini(msg);
    return false;
  }
  return true;
}

void
uv_msgs__msg__TaskStatus__fini(uv_msgs__msg__TaskStatus * msg)
{
  if (!msg) {
    return;
  }
  // status
  // current_task_index
  // total_tasks
  // current_task_name
  rosidl_runtime_c__String__fini(&msg->current_task_name);
  // error_message
  rosidl_runtime_c__String__fini(&msg->error_message);
}

bool
uv_msgs__msg__TaskStatus__are_equal(const uv_msgs__msg__TaskStatus * lhs, const uv_msgs__msg__TaskStatus * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // status
  if (lhs->status != rhs->status) {
    return false;
  }
  // current_task_index
  if (lhs->current_task_index != rhs->current_task_index) {
    return false;
  }
  // total_tasks
  if (lhs->total_tasks != rhs->total_tasks) {
    return false;
  }
  // current_task_name
  if (!rosidl_runtime_c__String__are_equal(
      &(lhs->current_task_name), &(rhs->current_task_name)))
  {
    return false;
  }
  // error_message
  if (!rosidl_runtime_c__String__are_equal(
      &(lhs->error_message), &(rhs->error_message)))
  {
    return false;
  }
  return true;
}

bool
uv_msgs__msg__TaskStatus__copy(
  const uv_msgs__msg__TaskStatus * input,
  uv_msgs__msg__TaskStatus * output)
{
  if (!input || !output) {
    return false;
  }
  // status
  output->status = input->status;
  // current_task_index
  output->current_task_index = input->current_task_index;
  // total_tasks
  output->total_tasks = input->total_tasks;
  // current_task_name
  if (!rosidl_runtime_c__String__copy(
      &(input->current_task_name), &(output->current_task_name)))
  {
    return false;
  }
  // error_message
  if (!rosidl_runtime_c__String__copy(
      &(input->error_message), &(output->error_message)))
  {
    return false;
  }
  return true;
}

uv_msgs__msg__TaskStatus *
uv_msgs__msg__TaskStatus__create(void)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  uv_msgs__msg__TaskStatus * msg = (uv_msgs__msg__TaskStatus *)allocator.allocate(sizeof(uv_msgs__msg__TaskStatus), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(uv_msgs__msg__TaskStatus));
  bool success = uv_msgs__msg__TaskStatus__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
uv_msgs__msg__TaskStatus__destroy(uv_msgs__msg__TaskStatus * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    uv_msgs__msg__TaskStatus__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
uv_msgs__msg__TaskStatus__Sequence__init(uv_msgs__msg__TaskStatus__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  uv_msgs__msg__TaskStatus * data = NULL;

  if (size) {
    data = (uv_msgs__msg__TaskStatus *)allocator.zero_allocate(size, sizeof(uv_msgs__msg__TaskStatus), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = uv_msgs__msg__TaskStatus__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        uv_msgs__msg__TaskStatus__fini(&data[i - 1]);
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
uv_msgs__msg__TaskStatus__Sequence__fini(uv_msgs__msg__TaskStatus__Sequence * array)
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
      uv_msgs__msg__TaskStatus__fini(&array->data[i]);
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

uv_msgs__msg__TaskStatus__Sequence *
uv_msgs__msg__TaskStatus__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  uv_msgs__msg__TaskStatus__Sequence * array = (uv_msgs__msg__TaskStatus__Sequence *)allocator.allocate(sizeof(uv_msgs__msg__TaskStatus__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = uv_msgs__msg__TaskStatus__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
uv_msgs__msg__TaskStatus__Sequence__destroy(uv_msgs__msg__TaskStatus__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    uv_msgs__msg__TaskStatus__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
uv_msgs__msg__TaskStatus__Sequence__are_equal(const uv_msgs__msg__TaskStatus__Sequence * lhs, const uv_msgs__msg__TaskStatus__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!uv_msgs__msg__TaskStatus__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
uv_msgs__msg__TaskStatus__Sequence__copy(
  const uv_msgs__msg__TaskStatus__Sequence * input,
  uv_msgs__msg__TaskStatus__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(uv_msgs__msg__TaskStatus);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    uv_msgs__msg__TaskStatus * data =
      (uv_msgs__msg__TaskStatus *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!uv_msgs__msg__TaskStatus__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          uv_msgs__msg__TaskStatus__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!uv_msgs__msg__TaskStatus__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
