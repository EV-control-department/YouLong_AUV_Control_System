// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from uv_msgs:msg/AuvState.idl
// generated code does not contain a copyright notice
#include "uv_msgs/msg/detail/auv_state__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


bool
uv_msgs__msg__AuvState__init(uv_msgs__msg__AuvState * msg)
{
  if (!msg) {
    return false;
  }
  // pos_x
  // pos_y
  // pos_z
  // yaw
  // vel_x
  // vel_y
  // vel_z
  // yaw_rate
  // vel_world_x
  // vel_world_y
  // armed
  // control_mode
  // battery_voltage
  // error_flags
  // cycle_time_ms
  // target_x
  // target_y
  // target_z
  // target_yaw
  return true;
}

void
uv_msgs__msg__AuvState__fini(uv_msgs__msg__AuvState * msg)
{
  if (!msg) {
    return;
  }
  // pos_x
  // pos_y
  // pos_z
  // yaw
  // vel_x
  // vel_y
  // vel_z
  // yaw_rate
  // vel_world_x
  // vel_world_y
  // armed
  // control_mode
  // battery_voltage
  // error_flags
  // cycle_time_ms
  // target_x
  // target_y
  // target_z
  // target_yaw
}

bool
uv_msgs__msg__AuvState__are_equal(const uv_msgs__msg__AuvState * lhs, const uv_msgs__msg__AuvState * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // pos_x
  if (lhs->pos_x != rhs->pos_x) {
    return false;
  }
  // pos_y
  if (lhs->pos_y != rhs->pos_y) {
    return false;
  }
  // pos_z
  if (lhs->pos_z != rhs->pos_z) {
    return false;
  }
  // yaw
  if (lhs->yaw != rhs->yaw) {
    return false;
  }
  // vel_x
  if (lhs->vel_x != rhs->vel_x) {
    return false;
  }
  // vel_y
  if (lhs->vel_y != rhs->vel_y) {
    return false;
  }
  // vel_z
  if (lhs->vel_z != rhs->vel_z) {
    return false;
  }
  // yaw_rate
  if (lhs->yaw_rate != rhs->yaw_rate) {
    return false;
  }
  // vel_world_x
  if (lhs->vel_world_x != rhs->vel_world_x) {
    return false;
  }
  // vel_world_y
  if (lhs->vel_world_y != rhs->vel_world_y) {
    return false;
  }
  // armed
  if (lhs->armed != rhs->armed) {
    return false;
  }
  // control_mode
  if (lhs->control_mode != rhs->control_mode) {
    return false;
  }
  // battery_voltage
  if (lhs->battery_voltage != rhs->battery_voltage) {
    return false;
  }
  // error_flags
  if (lhs->error_flags != rhs->error_flags) {
    return false;
  }
  // cycle_time_ms
  if (lhs->cycle_time_ms != rhs->cycle_time_ms) {
    return false;
  }
  // target_x
  if (lhs->target_x != rhs->target_x) {
    return false;
  }
  // target_y
  if (lhs->target_y != rhs->target_y) {
    return false;
  }
  // target_z
  if (lhs->target_z != rhs->target_z) {
    return false;
  }
  // target_yaw
  if (lhs->target_yaw != rhs->target_yaw) {
    return false;
  }
  return true;
}

bool
uv_msgs__msg__AuvState__copy(
  const uv_msgs__msg__AuvState * input,
  uv_msgs__msg__AuvState * output)
{
  if (!input || !output) {
    return false;
  }
  // pos_x
  output->pos_x = input->pos_x;
  // pos_y
  output->pos_y = input->pos_y;
  // pos_z
  output->pos_z = input->pos_z;
  // yaw
  output->yaw = input->yaw;
  // vel_x
  output->vel_x = input->vel_x;
  // vel_y
  output->vel_y = input->vel_y;
  // vel_z
  output->vel_z = input->vel_z;
  // yaw_rate
  output->yaw_rate = input->yaw_rate;
  // vel_world_x
  output->vel_world_x = input->vel_world_x;
  // vel_world_y
  output->vel_world_y = input->vel_world_y;
  // armed
  output->armed = input->armed;
  // control_mode
  output->control_mode = input->control_mode;
  // battery_voltage
  output->battery_voltage = input->battery_voltage;
  // error_flags
  output->error_flags = input->error_flags;
  // cycle_time_ms
  output->cycle_time_ms = input->cycle_time_ms;
  // target_x
  output->target_x = input->target_x;
  // target_y
  output->target_y = input->target_y;
  // target_z
  output->target_z = input->target_z;
  // target_yaw
  output->target_yaw = input->target_yaw;
  return true;
}

uv_msgs__msg__AuvState *
uv_msgs__msg__AuvState__create(void)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  uv_msgs__msg__AuvState * msg = (uv_msgs__msg__AuvState *)allocator.allocate(sizeof(uv_msgs__msg__AuvState), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(uv_msgs__msg__AuvState));
  bool success = uv_msgs__msg__AuvState__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
uv_msgs__msg__AuvState__destroy(uv_msgs__msg__AuvState * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    uv_msgs__msg__AuvState__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
uv_msgs__msg__AuvState__Sequence__init(uv_msgs__msg__AuvState__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  uv_msgs__msg__AuvState * data = NULL;

  if (size) {
    data = (uv_msgs__msg__AuvState *)allocator.zero_allocate(size, sizeof(uv_msgs__msg__AuvState), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = uv_msgs__msg__AuvState__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        uv_msgs__msg__AuvState__fini(&data[i - 1]);
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
uv_msgs__msg__AuvState__Sequence__fini(uv_msgs__msg__AuvState__Sequence * array)
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
      uv_msgs__msg__AuvState__fini(&array->data[i]);
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

uv_msgs__msg__AuvState__Sequence *
uv_msgs__msg__AuvState__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  uv_msgs__msg__AuvState__Sequence * array = (uv_msgs__msg__AuvState__Sequence *)allocator.allocate(sizeof(uv_msgs__msg__AuvState__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = uv_msgs__msg__AuvState__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
uv_msgs__msg__AuvState__Sequence__destroy(uv_msgs__msg__AuvState__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    uv_msgs__msg__AuvState__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
uv_msgs__msg__AuvState__Sequence__are_equal(const uv_msgs__msg__AuvState__Sequence * lhs, const uv_msgs__msg__AuvState__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!uv_msgs__msg__AuvState__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
uv_msgs__msg__AuvState__Sequence__copy(
  const uv_msgs__msg__AuvState__Sequence * input,
  uv_msgs__msg__AuvState__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(uv_msgs__msg__AuvState);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    uv_msgs__msg__AuvState * data =
      (uv_msgs__msg__AuvState *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!uv_msgs__msg__AuvState__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          uv_msgs__msg__AuvState__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!uv_msgs__msg__AuvState__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
