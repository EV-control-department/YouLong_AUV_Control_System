// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from uv_msgs:msg/Detection.idl
// generated code does not contain a copyright notice
#include "uv_msgs/msg/detail/detection__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


bool
uv_msgs__msg__Detection__init(uv_msgs__msg__Detection * msg)
{
  if (!msg) {
    return false;
  }
  // class_id
  // confidence
  // pixel_x
  // pixel_y
  // bbox_x1
  // bbox_y1
  // bbox_x2
  // bbox_y2
  return true;
}

void
uv_msgs__msg__Detection__fini(uv_msgs__msg__Detection * msg)
{
  if (!msg) {
    return;
  }
  // class_id
  // confidence
  // pixel_x
  // pixel_y
  // bbox_x1
  // bbox_y1
  // bbox_x2
  // bbox_y2
}

bool
uv_msgs__msg__Detection__are_equal(const uv_msgs__msg__Detection * lhs, const uv_msgs__msg__Detection * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // class_id
  if (lhs->class_id != rhs->class_id) {
    return false;
  }
  // confidence
  if (lhs->confidence != rhs->confidence) {
    return false;
  }
  // pixel_x
  if (lhs->pixel_x != rhs->pixel_x) {
    return false;
  }
  // pixel_y
  if (lhs->pixel_y != rhs->pixel_y) {
    return false;
  }
  // bbox_x1
  if (lhs->bbox_x1 != rhs->bbox_x1) {
    return false;
  }
  // bbox_y1
  if (lhs->bbox_y1 != rhs->bbox_y1) {
    return false;
  }
  // bbox_x2
  if (lhs->bbox_x2 != rhs->bbox_x2) {
    return false;
  }
  // bbox_y2
  if (lhs->bbox_y2 != rhs->bbox_y2) {
    return false;
  }
  return true;
}

bool
uv_msgs__msg__Detection__copy(
  const uv_msgs__msg__Detection * input,
  uv_msgs__msg__Detection * output)
{
  if (!input || !output) {
    return false;
  }
  // class_id
  output->class_id = input->class_id;
  // confidence
  output->confidence = input->confidence;
  // pixel_x
  output->pixel_x = input->pixel_x;
  // pixel_y
  output->pixel_y = input->pixel_y;
  // bbox_x1
  output->bbox_x1 = input->bbox_x1;
  // bbox_y1
  output->bbox_y1 = input->bbox_y1;
  // bbox_x2
  output->bbox_x2 = input->bbox_x2;
  // bbox_y2
  output->bbox_y2 = input->bbox_y2;
  return true;
}

uv_msgs__msg__Detection *
uv_msgs__msg__Detection__create(void)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  uv_msgs__msg__Detection * msg = (uv_msgs__msg__Detection *)allocator.allocate(sizeof(uv_msgs__msg__Detection), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(uv_msgs__msg__Detection));
  bool success = uv_msgs__msg__Detection__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
uv_msgs__msg__Detection__destroy(uv_msgs__msg__Detection * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    uv_msgs__msg__Detection__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
uv_msgs__msg__Detection__Sequence__init(uv_msgs__msg__Detection__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  uv_msgs__msg__Detection * data = NULL;

  if (size) {
    data = (uv_msgs__msg__Detection *)allocator.zero_allocate(size, sizeof(uv_msgs__msg__Detection), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = uv_msgs__msg__Detection__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        uv_msgs__msg__Detection__fini(&data[i - 1]);
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
uv_msgs__msg__Detection__Sequence__fini(uv_msgs__msg__Detection__Sequence * array)
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
      uv_msgs__msg__Detection__fini(&array->data[i]);
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

uv_msgs__msg__Detection__Sequence *
uv_msgs__msg__Detection__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  uv_msgs__msg__Detection__Sequence * array = (uv_msgs__msg__Detection__Sequence *)allocator.allocate(sizeof(uv_msgs__msg__Detection__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = uv_msgs__msg__Detection__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
uv_msgs__msg__Detection__Sequence__destroy(uv_msgs__msg__Detection__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    uv_msgs__msg__Detection__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
uv_msgs__msg__Detection__Sequence__are_equal(const uv_msgs__msg__Detection__Sequence * lhs, const uv_msgs__msg__Detection__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!uv_msgs__msg__Detection__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
uv_msgs__msg__Detection__Sequence__copy(
  const uv_msgs__msg__Detection__Sequence * input,
  uv_msgs__msg__Detection__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(uv_msgs__msg__Detection);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    uv_msgs__msg__Detection * data =
      (uv_msgs__msg__Detection *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!uv_msgs__msg__Detection__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          uv_msgs__msg__Detection__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!uv_msgs__msg__Detection__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
