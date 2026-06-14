// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from uv_msgs:msg/DeviceCommand.idl
// generated code does not contain a copyright notice
#include "uv_msgs/msg/detail/device_command__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


bool
uv_msgs__msg__DeviceCommand__init(uv_msgs__msg__DeviceCommand * msg)
{
  if (!msg) {
    return false;
  }
  // device_type
  // command
  // value
  return true;
}

void
uv_msgs__msg__DeviceCommand__fini(uv_msgs__msg__DeviceCommand * msg)
{
  if (!msg) {
    return;
  }
  // device_type
  // command
  // value
}

bool
uv_msgs__msg__DeviceCommand__are_equal(const uv_msgs__msg__DeviceCommand * lhs, const uv_msgs__msg__DeviceCommand * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // device_type
  if (lhs->device_type != rhs->device_type) {
    return false;
  }
  // command
  if (lhs->command != rhs->command) {
    return false;
  }
  // value
  if (lhs->value != rhs->value) {
    return false;
  }
  return true;
}

bool
uv_msgs__msg__DeviceCommand__copy(
  const uv_msgs__msg__DeviceCommand * input,
  uv_msgs__msg__DeviceCommand * output)
{
  if (!input || !output) {
    return false;
  }
  // device_type
  output->device_type = input->device_type;
  // command
  output->command = input->command;
  // value
  output->value = input->value;
  return true;
}

uv_msgs__msg__DeviceCommand *
uv_msgs__msg__DeviceCommand__create(void)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  uv_msgs__msg__DeviceCommand * msg = (uv_msgs__msg__DeviceCommand *)allocator.allocate(sizeof(uv_msgs__msg__DeviceCommand), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(uv_msgs__msg__DeviceCommand));
  bool success = uv_msgs__msg__DeviceCommand__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
uv_msgs__msg__DeviceCommand__destroy(uv_msgs__msg__DeviceCommand * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    uv_msgs__msg__DeviceCommand__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
uv_msgs__msg__DeviceCommand__Sequence__init(uv_msgs__msg__DeviceCommand__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  uv_msgs__msg__DeviceCommand * data = NULL;

  if (size) {
    data = (uv_msgs__msg__DeviceCommand *)allocator.zero_allocate(size, sizeof(uv_msgs__msg__DeviceCommand), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = uv_msgs__msg__DeviceCommand__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        uv_msgs__msg__DeviceCommand__fini(&data[i - 1]);
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
uv_msgs__msg__DeviceCommand__Sequence__fini(uv_msgs__msg__DeviceCommand__Sequence * array)
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
      uv_msgs__msg__DeviceCommand__fini(&array->data[i]);
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

uv_msgs__msg__DeviceCommand__Sequence *
uv_msgs__msg__DeviceCommand__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  uv_msgs__msg__DeviceCommand__Sequence * array = (uv_msgs__msg__DeviceCommand__Sequence *)allocator.allocate(sizeof(uv_msgs__msg__DeviceCommand__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = uv_msgs__msg__DeviceCommand__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
uv_msgs__msg__DeviceCommand__Sequence__destroy(uv_msgs__msg__DeviceCommand__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    uv_msgs__msg__DeviceCommand__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
uv_msgs__msg__DeviceCommand__Sequence__are_equal(const uv_msgs__msg__DeviceCommand__Sequence * lhs, const uv_msgs__msg__DeviceCommand__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!uv_msgs__msg__DeviceCommand__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
uv_msgs__msg__DeviceCommand__Sequence__copy(
  const uv_msgs__msg__DeviceCommand__Sequence * input,
  uv_msgs__msg__DeviceCommand__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(uv_msgs__msg__DeviceCommand);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    uv_msgs__msg__DeviceCommand * data =
      (uv_msgs__msg__DeviceCommand *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!uv_msgs__msg__DeviceCommand__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          uv_msgs__msg__DeviceCommand__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!uv_msgs__msg__DeviceCommand__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
