// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from zit6_interfaces:msg/ZitPidStatus.idl
// generated code does not contain a copyright notice
#include "zit6_interfaces/msg/detail/zit_pid_status__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


bool
zit6_interfaces__msg__ZitPidStatus__init(zit6_interfaces__msg__ZitPidStatus * msg)
{
  if (!msg) {
    return false;
  }
  // pos_kp
  // pos_max_v
  // pos_max_a
  // pos_out_limit
  // vel_kp
  // vel_ki
  // vel_kd
  // vel_i_limit
  // vel_out_limit
  return true;
}

void
zit6_interfaces__msg__ZitPidStatus__fini(zit6_interfaces__msg__ZitPidStatus * msg)
{
  if (!msg) {
    return;
  }
  // pos_kp
  // pos_max_v
  // pos_max_a
  // pos_out_limit
  // vel_kp
  // vel_ki
  // vel_kd
  // vel_i_limit
  // vel_out_limit
}

bool
zit6_interfaces__msg__ZitPidStatus__are_equal(const zit6_interfaces__msg__ZitPidStatus * lhs, const zit6_interfaces__msg__ZitPidStatus * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // pos_kp
  for (size_t i = 0; i < 4; ++i) {
    if (lhs->pos_kp[i] != rhs->pos_kp[i]) {
      return false;
    }
  }
  // pos_max_v
  for (size_t i = 0; i < 4; ++i) {
    if (lhs->pos_max_v[i] != rhs->pos_max_v[i]) {
      return false;
    }
  }
  // pos_max_a
  for (size_t i = 0; i < 4; ++i) {
    if (lhs->pos_max_a[i] != rhs->pos_max_a[i]) {
      return false;
    }
  }
  // pos_out_limit
  for (size_t i = 0; i < 4; ++i) {
    if (lhs->pos_out_limit[i] != rhs->pos_out_limit[i]) {
      return false;
    }
  }
  // vel_kp
  for (size_t i = 0; i < 4; ++i) {
    if (lhs->vel_kp[i] != rhs->vel_kp[i]) {
      return false;
    }
  }
  // vel_ki
  for (size_t i = 0; i < 4; ++i) {
    if (lhs->vel_ki[i] != rhs->vel_ki[i]) {
      return false;
    }
  }
  // vel_kd
  for (size_t i = 0; i < 4; ++i) {
    if (lhs->vel_kd[i] != rhs->vel_kd[i]) {
      return false;
    }
  }
  // vel_i_limit
  for (size_t i = 0; i < 4; ++i) {
    if (lhs->vel_i_limit[i] != rhs->vel_i_limit[i]) {
      return false;
    }
  }
  // vel_out_limit
  for (size_t i = 0; i < 4; ++i) {
    if (lhs->vel_out_limit[i] != rhs->vel_out_limit[i]) {
      return false;
    }
  }
  return true;
}

bool
zit6_interfaces__msg__ZitPidStatus__copy(
  const zit6_interfaces__msg__ZitPidStatus * input,
  zit6_interfaces__msg__ZitPidStatus * output)
{
  if (!input || !output) {
    return false;
  }
  // pos_kp
  for (size_t i = 0; i < 4; ++i) {
    output->pos_kp[i] = input->pos_kp[i];
  }
  // pos_max_v
  for (size_t i = 0; i < 4; ++i) {
    output->pos_max_v[i] = input->pos_max_v[i];
  }
  // pos_max_a
  for (size_t i = 0; i < 4; ++i) {
    output->pos_max_a[i] = input->pos_max_a[i];
  }
  // pos_out_limit
  for (size_t i = 0; i < 4; ++i) {
    output->pos_out_limit[i] = input->pos_out_limit[i];
  }
  // vel_kp
  for (size_t i = 0; i < 4; ++i) {
    output->vel_kp[i] = input->vel_kp[i];
  }
  // vel_ki
  for (size_t i = 0; i < 4; ++i) {
    output->vel_ki[i] = input->vel_ki[i];
  }
  // vel_kd
  for (size_t i = 0; i < 4; ++i) {
    output->vel_kd[i] = input->vel_kd[i];
  }
  // vel_i_limit
  for (size_t i = 0; i < 4; ++i) {
    output->vel_i_limit[i] = input->vel_i_limit[i];
  }
  // vel_out_limit
  for (size_t i = 0; i < 4; ++i) {
    output->vel_out_limit[i] = input->vel_out_limit[i];
  }
  return true;
}

zit6_interfaces__msg__ZitPidStatus *
zit6_interfaces__msg__ZitPidStatus__create(void)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  zit6_interfaces__msg__ZitPidStatus * msg = (zit6_interfaces__msg__ZitPidStatus *)allocator.allocate(sizeof(zit6_interfaces__msg__ZitPidStatus), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(zit6_interfaces__msg__ZitPidStatus));
  bool success = zit6_interfaces__msg__ZitPidStatus__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
zit6_interfaces__msg__ZitPidStatus__destroy(zit6_interfaces__msg__ZitPidStatus * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    zit6_interfaces__msg__ZitPidStatus__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
zit6_interfaces__msg__ZitPidStatus__Sequence__init(zit6_interfaces__msg__ZitPidStatus__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  zit6_interfaces__msg__ZitPidStatus * data = NULL;

  if (size) {
    data = (zit6_interfaces__msg__ZitPidStatus *)allocator.zero_allocate(size, sizeof(zit6_interfaces__msg__ZitPidStatus), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = zit6_interfaces__msg__ZitPidStatus__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        zit6_interfaces__msg__ZitPidStatus__fini(&data[i - 1]);
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
zit6_interfaces__msg__ZitPidStatus__Sequence__fini(zit6_interfaces__msg__ZitPidStatus__Sequence * array)
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
      zit6_interfaces__msg__ZitPidStatus__fini(&array->data[i]);
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

zit6_interfaces__msg__ZitPidStatus__Sequence *
zit6_interfaces__msg__ZitPidStatus__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  zit6_interfaces__msg__ZitPidStatus__Sequence * array = (zit6_interfaces__msg__ZitPidStatus__Sequence *)allocator.allocate(sizeof(zit6_interfaces__msg__ZitPidStatus__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = zit6_interfaces__msg__ZitPidStatus__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
zit6_interfaces__msg__ZitPidStatus__Sequence__destroy(zit6_interfaces__msg__ZitPidStatus__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    zit6_interfaces__msg__ZitPidStatus__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
zit6_interfaces__msg__ZitPidStatus__Sequence__are_equal(const zit6_interfaces__msg__ZitPidStatus__Sequence * lhs, const zit6_interfaces__msg__ZitPidStatus__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!zit6_interfaces__msg__ZitPidStatus__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
zit6_interfaces__msg__ZitPidStatus__Sequence__copy(
  const zit6_interfaces__msg__ZitPidStatus__Sequence * input,
  zit6_interfaces__msg__ZitPidStatus__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(zit6_interfaces__msg__ZitPidStatus);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    zit6_interfaces__msg__ZitPidStatus * data =
      (zit6_interfaces__msg__ZitPidStatus *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!zit6_interfaces__msg__ZitPidStatus__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          zit6_interfaces__msg__ZitPidStatus__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!zit6_interfaces__msg__ZitPidStatus__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
