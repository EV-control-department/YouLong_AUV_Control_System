// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from zit6_interfaces:msg/ZitPid.idl
// generated code does not contain a copyright notice
#include "zit6_interfaces/msg/detail/zit_pid__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


bool
zit6_interfaces__msg__ZitPid__init(zit6_interfaces__msg__ZitPid * msg)
{
  if (!msg) {
    return false;
  }
  // axis
  // is_pos_ring
  // kp
  // ki
  // kd
  // i_limit
  // out_limit
  // max_v
  // max_a
  return true;
}

void
zit6_interfaces__msg__ZitPid__fini(zit6_interfaces__msg__ZitPid * msg)
{
  if (!msg) {
    return;
  }
  // axis
  // is_pos_ring
  // kp
  // ki
  // kd
  // i_limit
  // out_limit
  // max_v
  // max_a
}

bool
zit6_interfaces__msg__ZitPid__are_equal(const zit6_interfaces__msg__ZitPid * lhs, const zit6_interfaces__msg__ZitPid * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // axis
  if (lhs->axis != rhs->axis) {
    return false;
  }
  // is_pos_ring
  if (lhs->is_pos_ring != rhs->is_pos_ring) {
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
  // out_limit
  if (lhs->out_limit != rhs->out_limit) {
    return false;
  }
  // max_v
  if (lhs->max_v != rhs->max_v) {
    return false;
  }
  // max_a
  if (lhs->max_a != rhs->max_a) {
    return false;
  }
  return true;
}

bool
zit6_interfaces__msg__ZitPid__copy(
  const zit6_interfaces__msg__ZitPid * input,
  zit6_interfaces__msg__ZitPid * output)
{
  if (!input || !output) {
    return false;
  }
  // axis
  output->axis = input->axis;
  // is_pos_ring
  output->is_pos_ring = input->is_pos_ring;
  // kp
  output->kp = input->kp;
  // ki
  output->ki = input->ki;
  // kd
  output->kd = input->kd;
  // i_limit
  output->i_limit = input->i_limit;
  // out_limit
  output->out_limit = input->out_limit;
  // max_v
  output->max_v = input->max_v;
  // max_a
  output->max_a = input->max_a;
  return true;
}

zit6_interfaces__msg__ZitPid *
zit6_interfaces__msg__ZitPid__create(void)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  zit6_interfaces__msg__ZitPid * msg = (zit6_interfaces__msg__ZitPid *)allocator.allocate(sizeof(zit6_interfaces__msg__ZitPid), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(zit6_interfaces__msg__ZitPid));
  bool success = zit6_interfaces__msg__ZitPid__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
zit6_interfaces__msg__ZitPid__destroy(zit6_interfaces__msg__ZitPid * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    zit6_interfaces__msg__ZitPid__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
zit6_interfaces__msg__ZitPid__Sequence__init(zit6_interfaces__msg__ZitPid__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  zit6_interfaces__msg__ZitPid * data = NULL;

  if (size) {
    data = (zit6_interfaces__msg__ZitPid *)allocator.zero_allocate(size, sizeof(zit6_interfaces__msg__ZitPid), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = zit6_interfaces__msg__ZitPid__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        zit6_interfaces__msg__ZitPid__fini(&data[i - 1]);
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
zit6_interfaces__msg__ZitPid__Sequence__fini(zit6_interfaces__msg__ZitPid__Sequence * array)
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
      zit6_interfaces__msg__ZitPid__fini(&array->data[i]);
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

zit6_interfaces__msg__ZitPid__Sequence *
zit6_interfaces__msg__ZitPid__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  zit6_interfaces__msg__ZitPid__Sequence * array = (zit6_interfaces__msg__ZitPid__Sequence *)allocator.allocate(sizeof(zit6_interfaces__msg__ZitPid__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = zit6_interfaces__msg__ZitPid__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
zit6_interfaces__msg__ZitPid__Sequence__destroy(zit6_interfaces__msg__ZitPid__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    zit6_interfaces__msg__ZitPid__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
zit6_interfaces__msg__ZitPid__Sequence__are_equal(const zit6_interfaces__msg__ZitPid__Sequence * lhs, const zit6_interfaces__msg__ZitPid__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!zit6_interfaces__msg__ZitPid__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
zit6_interfaces__msg__ZitPid__Sequence__copy(
  const zit6_interfaces__msg__ZitPid__Sequence * input,
  zit6_interfaces__msg__ZitPid__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(zit6_interfaces__msg__ZitPid);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    zit6_interfaces__msg__ZitPid * data =
      (zit6_interfaces__msg__ZitPid *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!zit6_interfaces__msg__ZitPid__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          zit6_interfaces__msg__ZitPid__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!zit6_interfaces__msg__ZitPid__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
