// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from uv_msgs:msg/PidGains.idl
// generated code does not contain a copyright notice
#include "uv_msgs/msg/detail/pid_gains__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


// Include directives for member types
// Member `name`
#include "rosidl_runtime_c/string_functions.h"

bool
uv_msgs__msg__PidGains__init(uv_msgs__msg__PidGains * msg)
{
  if (!msg) {
    return false;
  }
  // name
  if (!rosidl_runtime_c__String__init(&msg->name)) {
    uv_msgs__msg__PidGains__fini(msg);
    return false;
  }
  // kp
  // ki
  // kd
  // i_limit
  // output_limit
  // feedforward
  return true;
}

void
uv_msgs__msg__PidGains__fini(uv_msgs__msg__PidGains * msg)
{
  if (!msg) {
    return;
  }
  // name
  rosidl_runtime_c__String__fini(&msg->name);
  // kp
  // ki
  // kd
  // i_limit
  // output_limit
  // feedforward
}

bool
uv_msgs__msg__PidGains__are_equal(const uv_msgs__msg__PidGains * lhs, const uv_msgs__msg__PidGains * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // name
  if (!rosidl_runtime_c__String__are_equal(
      &(lhs->name), &(rhs->name)))
  {
    return false;
  }
  // kp
  if (lhs->kp != rhs->kp) {
    return false;
  }
  // ki
  if (lhs->ki != rhs->ki) {
    return false;
  }
  // kd
  if (lhs->kd != rhs->kd) {
    return false;
  }
  // i_limit
  if (lhs->i_limit != rhs->i_limit) {
    return false;
  }
  // output_limit
  if (lhs->output_limit != rhs->output_limit) {
    return false;
  }
  // feedforward
  if (lhs->feedforward != rhs->feedforward) {
    return false;
  }
  return true;
}

bool
uv_msgs__msg__PidGains__copy(
  const uv_msgs__msg__PidGains * input,
  uv_msgs__msg__PidGains * output)
{
  if (!input || !output) {
    return false;
  }
  // name
  if (!rosidl_runtime_c__String__copy(
      &(input->name), &(output->name)))
  {
    return false;
  }
  // kp
  output->kp = input->kp;
  // ki
  output->ki = input->ki;
  // kd
  output->kd = input->kd;
  // i_limit
  output->i_limit = input->i_limit;
  // output_limit
  output->output_limit = input->output_limit;
  // feedforward
  output->feedforward = input->feedforward;
  return true;
}

uv_msgs__msg__PidGains *
uv_msgs__msg__PidGains__create(void)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  uv_msgs__msg__PidGains * msg = (uv_msgs__msg__PidGains *)allocator.allocate(sizeof(uv_msgs__msg__PidGains), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(uv_msgs__msg__PidGains));
  bool success = uv_msgs__msg__PidGains__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
uv_msgs__msg__PidGains__destroy(uv_msgs__msg__PidGains * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    uv_msgs__msg__PidGains__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
uv_msgs__msg__PidGains__Sequence__init(uv_msgs__msg__PidGains__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  uv_msgs__msg__PidGains * data = NULL;

  if (size) {
    data = (uv_msgs__msg__PidGains *)allocator.zero_allocate(size, sizeof(uv_msgs__msg__PidGains), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = uv_msgs__msg__PidGains__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        uv_msgs__msg__PidGains__fini(&data[i - 1]);
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
uv_msgs__msg__PidGains__Sequence__fini(uv_msgs__msg__PidGains__Sequence * array)
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
      uv_msgs__msg__PidGains__fini(&array->data[i]);
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

uv_msgs__msg__PidGains__Sequence *
uv_msgs__msg__PidGains__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  uv_msgs__msg__PidGains__Sequence * array = (uv_msgs__msg__PidGains__Sequence *)allocator.allocate(sizeof(uv_msgs__msg__PidGains__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = uv_msgs__msg__PidGains__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
uv_msgs__msg__PidGains__Sequence__destroy(uv_msgs__msg__PidGains__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    uv_msgs__msg__PidGains__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
uv_msgs__msg__PidGains__Sequence__are_equal(const uv_msgs__msg__PidGains__Sequence * lhs, const uv_msgs__msg__PidGains__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!uv_msgs__msg__PidGains__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
uv_msgs__msg__PidGains__Sequence__copy(
  const uv_msgs__msg__PidGains__Sequence * input,
  uv_msgs__msg__PidGains__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(uv_msgs__msg__PidGains);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    uv_msgs__msg__PidGains * data =
      (uv_msgs__msg__PidGains *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!uv_msgs__msg__PidGains__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          uv_msgs__msg__PidGains__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!uv_msgs__msg__PidGains__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
