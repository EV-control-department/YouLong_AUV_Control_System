// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from uv_msgs:msg/ObjectPosition.idl
// generated code does not contain a copyright notice
#include "uv_msgs/msg/detail/object_position__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


// Include directives for member types
// Member `class_name`
#include "rosidl_runtime_c/string_functions.h"

bool
uv_msgs__msg__ObjectPosition__init(uv_msgs__msg__ObjectPosition * msg)
{
  if (!msg) {
    return false;
  }
  // class_id
  // class_name
  if (!rosidl_runtime_c__String__init(&msg->class_name)) {
    uv_msgs__msg__ObjectPosition__fini(msg);
    return false;
  }
  // world_x
  // world_y
  // world_z
  // confidence
  // num_observations
  return true;
}

void
uv_msgs__msg__ObjectPosition__fini(uv_msgs__msg__ObjectPosition * msg)
{
  if (!msg) {
    return;
  }
  // class_id
  // class_name
  rosidl_runtime_c__String__fini(&msg->class_name);
  // world_x
  // world_y
  // world_z
  // confidence
  // num_observations
}

bool
uv_msgs__msg__ObjectPosition__are_equal(const uv_msgs__msg__ObjectPosition * lhs, const uv_msgs__msg__ObjectPosition * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // class_id
  if (lhs->class_id != rhs->class_id) {
    return false;
  }
  // class_name
  if (!rosidl_runtime_c__String__are_equal(
      &(lhs->class_name), &(rhs->class_name)))
  {
    return false;
  }
  // world_x
  if (lhs->world_x != rhs->world_x) {
    return false;
  }
  // world_y
  if (lhs->world_y != rhs->world_y) {
    return false;
  }
  // world_z
  if (lhs->world_z != rhs->world_z) {
    return false;
  }
  // confidence
  if (lhs->confidence != rhs->confidence) {
    return false;
  }
  // num_observations
  if (lhs->num_observations != rhs->num_observations) {
    return false;
  }
  return true;
}

bool
uv_msgs__msg__ObjectPosition__copy(
  const uv_msgs__msg__ObjectPosition * input,
  uv_msgs__msg__ObjectPosition * output)
{
  if (!input || !output) {
    return false;
  }
  // class_id
  output->class_id = input->class_id;
  // class_name
  if (!rosidl_runtime_c__String__copy(
      &(input->class_name), &(output->class_name)))
  {
    return false;
  }
  // world_x
  output->world_x = input->world_x;
  // world_y
  output->world_y = input->world_y;
  // world_z
  output->world_z = input->world_z;
  // confidence
  output->confidence = input->confidence;
  // num_observations
  output->num_observations = input->num_observations;
  return true;
}

uv_msgs__msg__ObjectPosition *
uv_msgs__msg__ObjectPosition__create(void)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  uv_msgs__msg__ObjectPosition * msg = (uv_msgs__msg__ObjectPosition *)allocator.allocate(sizeof(uv_msgs__msg__ObjectPosition), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(uv_msgs__msg__ObjectPosition));
  bool success = uv_msgs__msg__ObjectPosition__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
uv_msgs__msg__ObjectPosition__destroy(uv_msgs__msg__ObjectPosition * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    uv_msgs__msg__ObjectPosition__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
uv_msgs__msg__ObjectPosition__Sequence__init(uv_msgs__msg__ObjectPosition__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  uv_msgs__msg__ObjectPosition * data = NULL;

  if (size) {
    data = (uv_msgs__msg__ObjectPosition *)allocator.zero_allocate(size, sizeof(uv_msgs__msg__ObjectPosition), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = uv_msgs__msg__ObjectPosition__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        uv_msgs__msg__ObjectPosition__fini(&data[i - 1]);
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
uv_msgs__msg__ObjectPosition__Sequence__fini(uv_msgs__msg__ObjectPosition__Sequence * array)
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
      uv_msgs__msg__ObjectPosition__fini(&array->data[i]);
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

uv_msgs__msg__ObjectPosition__Sequence *
uv_msgs__msg__ObjectPosition__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  uv_msgs__msg__ObjectPosition__Sequence * array = (uv_msgs__msg__ObjectPosition__Sequence *)allocator.allocate(sizeof(uv_msgs__msg__ObjectPosition__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = uv_msgs__msg__ObjectPosition__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
uv_msgs__msg__ObjectPosition__Sequence__destroy(uv_msgs__msg__ObjectPosition__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    uv_msgs__msg__ObjectPosition__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
uv_msgs__msg__ObjectPosition__Sequence__are_equal(const uv_msgs__msg__ObjectPosition__Sequence * lhs, const uv_msgs__msg__ObjectPosition__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!uv_msgs__msg__ObjectPosition__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
uv_msgs__msg__ObjectPosition__Sequence__copy(
  const uv_msgs__msg__ObjectPosition__Sequence * input,
  uv_msgs__msg__ObjectPosition__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(uv_msgs__msg__ObjectPosition);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    uv_msgs__msg__ObjectPosition * data =
      (uv_msgs__msg__ObjectPosition *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!uv_msgs__msg__ObjectPosition__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          uv_msgs__msg__ObjectPosition__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!uv_msgs__msg__ObjectPosition__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
