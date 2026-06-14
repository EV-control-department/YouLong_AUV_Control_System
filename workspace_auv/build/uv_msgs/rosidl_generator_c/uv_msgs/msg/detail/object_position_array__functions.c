// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from uv_msgs:msg/ObjectPositionArray.idl
// generated code does not contain a copyright notice
#include "uv_msgs/msg/detail/object_position_array__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


// Include directives for member types
// Member `header`
#include "std_msgs/msg/detail/header__functions.h"
// Member `objects`
#include "uv_msgs/msg/detail/object_position__functions.h"

bool
uv_msgs__msg__ObjectPositionArray__init(uv_msgs__msg__ObjectPositionArray * msg)
{
  if (!msg) {
    return false;
  }
  // header
  if (!std_msgs__msg__Header__init(&msg->header)) {
    uv_msgs__msg__ObjectPositionArray__fini(msg);
    return false;
  }
  // objects
  if (!uv_msgs__msg__ObjectPosition__Sequence__init(&msg->objects, 0)) {
    uv_msgs__msg__ObjectPositionArray__fini(msg);
    return false;
  }
  return true;
}

void
uv_msgs__msg__ObjectPositionArray__fini(uv_msgs__msg__ObjectPositionArray * msg)
{
  if (!msg) {
    return;
  }
  // header
  std_msgs__msg__Header__fini(&msg->header);
  // objects
  uv_msgs__msg__ObjectPosition__Sequence__fini(&msg->objects);
}

bool
uv_msgs__msg__ObjectPositionArray__are_equal(const uv_msgs__msg__ObjectPositionArray * lhs, const uv_msgs__msg__ObjectPositionArray * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // header
  if (!std_msgs__msg__Header__are_equal(
      &(lhs->header), &(rhs->header)))
  {
    return false;
  }
  // objects
  if (!uv_msgs__msg__ObjectPosition__Sequence__are_equal(
      &(lhs->objects), &(rhs->objects)))
  {
    return false;
  }
  return true;
}

bool
uv_msgs__msg__ObjectPositionArray__copy(
  const uv_msgs__msg__ObjectPositionArray * input,
  uv_msgs__msg__ObjectPositionArray * output)
{
  if (!input || !output) {
    return false;
  }
  // header
  if (!std_msgs__msg__Header__copy(
      &(input->header), &(output->header)))
  {
    return false;
  }
  // objects
  if (!uv_msgs__msg__ObjectPosition__Sequence__copy(
      &(input->objects), &(output->objects)))
  {
    return false;
  }
  return true;
}

uv_msgs__msg__ObjectPositionArray *
uv_msgs__msg__ObjectPositionArray__create(void)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  uv_msgs__msg__ObjectPositionArray * msg = (uv_msgs__msg__ObjectPositionArray *)allocator.allocate(sizeof(uv_msgs__msg__ObjectPositionArray), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(uv_msgs__msg__ObjectPositionArray));
  bool success = uv_msgs__msg__ObjectPositionArray__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
uv_msgs__msg__ObjectPositionArray__destroy(uv_msgs__msg__ObjectPositionArray * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    uv_msgs__msg__ObjectPositionArray__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
uv_msgs__msg__ObjectPositionArray__Sequence__init(uv_msgs__msg__ObjectPositionArray__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  uv_msgs__msg__ObjectPositionArray * data = NULL;

  if (size) {
    data = (uv_msgs__msg__ObjectPositionArray *)allocator.zero_allocate(size, sizeof(uv_msgs__msg__ObjectPositionArray), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = uv_msgs__msg__ObjectPositionArray__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        uv_msgs__msg__ObjectPositionArray__fini(&data[i - 1]);
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
uv_msgs__msg__ObjectPositionArray__Sequence__fini(uv_msgs__msg__ObjectPositionArray__Sequence * array)
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
      uv_msgs__msg__ObjectPositionArray__fini(&array->data[i]);
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

uv_msgs__msg__ObjectPositionArray__Sequence *
uv_msgs__msg__ObjectPositionArray__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  uv_msgs__msg__ObjectPositionArray__Sequence * array = (uv_msgs__msg__ObjectPositionArray__Sequence *)allocator.allocate(sizeof(uv_msgs__msg__ObjectPositionArray__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = uv_msgs__msg__ObjectPositionArray__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
uv_msgs__msg__ObjectPositionArray__Sequence__destroy(uv_msgs__msg__ObjectPositionArray__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    uv_msgs__msg__ObjectPositionArray__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
uv_msgs__msg__ObjectPositionArray__Sequence__are_equal(const uv_msgs__msg__ObjectPositionArray__Sequence * lhs, const uv_msgs__msg__ObjectPositionArray__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!uv_msgs__msg__ObjectPositionArray__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
uv_msgs__msg__ObjectPositionArray__Sequence__copy(
  const uv_msgs__msg__ObjectPositionArray__Sequence * input,
  uv_msgs__msg__ObjectPositionArray__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(uv_msgs__msg__ObjectPositionArray);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    uv_msgs__msg__ObjectPositionArray * data =
      (uv_msgs__msg__ObjectPositionArray *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!uv_msgs__msg__ObjectPositionArray__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          uv_msgs__msg__ObjectPositionArray__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!uv_msgs__msg__ObjectPositionArray__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
