// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from uv_msgs:msg/PidState.idl
// generated code does not contain a copyright notice
#include "uv_msgs/msg/detail/pid_state__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


// Include directives for member types
// Member `speed_mag`
// Member `vel_x`
// Member `vel_y`
// Member `pos_z`
// Member `yaw`
// Member `yaw_rate`
#include "uv_msgs/msg/detail/pid_gains__functions.h"

bool
uv_msgs__msg__PidState__init(uv_msgs__msg__PidState * msg)
{
  if (!msg) {
    return false;
  }
  // speed_mag
  if (!uv_msgs__msg__PidGains__init(&msg->speed_mag)) {
    uv_msgs__msg__PidState__fini(msg);
    return false;
  }
  // vel_x
  if (!uv_msgs__msg__PidGains__init(&msg->vel_x)) {
    uv_msgs__msg__PidState__fini(msg);
    return false;
  }
  // vel_y
  if (!uv_msgs__msg__PidGains__init(&msg->vel_y)) {
    uv_msgs__msg__PidState__fini(msg);
    return false;
  }
  // pos_z
  if (!uv_msgs__msg__PidGains__init(&msg->pos_z)) {
    uv_msgs__msg__PidState__fini(msg);
    return false;
  }
  // yaw
  if (!uv_msgs__msg__PidGains__init(&msg->yaw)) {
    uv_msgs__msg__PidState__fini(msg);
    return false;
  }
  // yaw_rate
  if (!uv_msgs__msg__PidGains__init(&msg->yaw_rate)) {
    uv_msgs__msg__PidState__fini(msg);
    return false;
  }
  // error_length
  // error_angle_deg
  // speed_cmd
  // vx_body_cmd
  // vy_body_cmd
  return true;
}

void
uv_msgs__msg__PidState__fini(uv_msgs__msg__PidState * msg)
{
  if (!msg) {
    return;
  }
  // speed_mag
  uv_msgs__msg__PidGains__fini(&msg->speed_mag);
  // vel_x
  uv_msgs__msg__PidGains__fini(&msg->vel_x);
  // vel_y
  uv_msgs__msg__PidGains__fini(&msg->vel_y);
  // pos_z
  uv_msgs__msg__PidGains__fini(&msg->pos_z);
  // yaw
  uv_msgs__msg__PidGains__fini(&msg->yaw);
  // yaw_rate
  uv_msgs__msg__PidGains__fini(&msg->yaw_rate);
  // error_length
  // error_angle_deg
  // speed_cmd
  // vx_body_cmd
  // vy_body_cmd
}

bool
uv_msgs__msg__PidState__are_equal(const uv_msgs__msg__PidState * lhs, const uv_msgs__msg__PidState * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // speed_mag
  if (!uv_msgs__msg__PidGains__are_equal(
      &(lhs->speed_mag), &(rhs->speed_mag)))
  {
    return false;
  }
  // vel_x
  if (!uv_msgs__msg__PidGains__are_equal(
      &(lhs->vel_x), &(rhs->vel_x)))
  {
    return false;
  }
  // vel_y
  if (!uv_msgs__msg__PidGains__are_equal(
      &(lhs->vel_y), &(rhs->vel_y)))
  {
    return false;
  }
  // pos_z
  if (!uv_msgs__msg__PidGains__are_equal(
      &(lhs->pos_z), &(rhs->pos_z)))
  {
    return false;
  }
  // yaw
  if (!uv_msgs__msg__PidGains__are_equal(
      &(lhs->yaw), &(rhs->yaw)))
  {
    return false;
  }
  // yaw_rate
  if (!uv_msgs__msg__PidGains__are_equal(
      &(lhs->yaw_rate), &(rhs->yaw_rate)))
  {
    return false;
  }
  // error_length
  if (lhs->error_length != rhs->error_length) {
    return false;
  }
  // error_angle_deg
  if (lhs->error_angle_deg != rhs->error_angle_deg) {
    return false;
  }
  // speed_cmd
  if (lhs->speed_cmd != rhs->speed_cmd) {
    return false;
  }
  // vx_body_cmd
  if (lhs->vx_body_cmd != rhs->vx_body_cmd) {
    return false;
  }
  // vy_body_cmd
  if (lhs->vy_body_cmd != rhs->vy_body_cmd) {
    return false;
  }
  return true;
}

bool
uv_msgs__msg__PidState__copy(
  const uv_msgs__msg__PidState * input,
  uv_msgs__msg__PidState * output)
{
  if (!input || !output) {
    return false;
  }
  // speed_mag
  if (!uv_msgs__msg__PidGains__copy(
      &(input->speed_mag), &(output->speed_mag)))
  {
    return false;
  }
  // vel_x
  if (!uv_msgs__msg__PidGains__copy(
      &(input->vel_x), &(output->vel_x)))
  {
    return false;
  }
  // vel_y
  if (!uv_msgs__msg__PidGains__copy(
      &(input->vel_y), &(output->vel_y)))
  {
    return false;
  }
  // pos_z
  if (!uv_msgs__msg__PidGains__copy(
      &(input->pos_z), &(output->pos_z)))
  {
    return false;
  }
  // yaw
  if (!uv_msgs__msg__PidGains__copy(
      &(input->yaw), &(output->yaw)))
  {
    return false;
  }
  // yaw_rate
  if (!uv_msgs__msg__PidGains__copy(
      &(input->yaw_rate), &(output->yaw_rate)))
  {
    return false;
  }
  // error_length
  output->error_length = input->error_length;
  // error_angle_deg
  output->error_angle_deg = input->error_angle_deg;
  // speed_cmd
  output->speed_cmd = input->speed_cmd;
  // vx_body_cmd
  output->vx_body_cmd = input->vx_body_cmd;
  // vy_body_cmd
  output->vy_body_cmd = input->vy_body_cmd;
  return true;
}

uv_msgs__msg__PidState *
uv_msgs__msg__PidState__create(void)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  uv_msgs__msg__PidState * msg = (uv_msgs__msg__PidState *)allocator.allocate(sizeof(uv_msgs__msg__PidState), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(uv_msgs__msg__PidState));
  bool success = uv_msgs__msg__PidState__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
uv_msgs__msg__PidState__destroy(uv_msgs__msg__PidState * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    uv_msgs__msg__PidState__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
uv_msgs__msg__PidState__Sequence__init(uv_msgs__msg__PidState__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  uv_msgs__msg__PidState * data = NULL;

  if (size) {
    data = (uv_msgs__msg__PidState *)allocator.zero_allocate(size, sizeof(uv_msgs__msg__PidState), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = uv_msgs__msg__PidState__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        uv_msgs__msg__PidState__fini(&data[i - 1]);
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
uv_msgs__msg__PidState__Sequence__fini(uv_msgs__msg__PidState__Sequence * array)
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
      uv_msgs__msg__PidState__fini(&array->data[i]);
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

uv_msgs__msg__PidState__Sequence *
uv_msgs__msg__PidState__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  uv_msgs__msg__PidState__Sequence * array = (uv_msgs__msg__PidState__Sequence *)allocator.allocate(sizeof(uv_msgs__msg__PidState__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = uv_msgs__msg__PidState__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
uv_msgs__msg__PidState__Sequence__destroy(uv_msgs__msg__PidState__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    uv_msgs__msg__PidState__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
uv_msgs__msg__PidState__Sequence__are_equal(const uv_msgs__msg__PidState__Sequence * lhs, const uv_msgs__msg__PidState__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!uv_msgs__msg__PidState__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
uv_msgs__msg__PidState__Sequence__copy(
  const uv_msgs__msg__PidState__Sequence * input,
  uv_msgs__msg__PidState__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(uv_msgs__msg__PidState);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    uv_msgs__msg__PidState * data =
      (uv_msgs__msg__PidState *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!uv_msgs__msg__PidState__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          uv_msgs__msg__PidState__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!uv_msgs__msg__PidState__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
