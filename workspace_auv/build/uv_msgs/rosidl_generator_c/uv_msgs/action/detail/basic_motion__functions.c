// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from uv_msgs:action/BasicMotion.idl
// generated code does not contain a copyright notice
#include "uv_msgs/action/detail/basic_motion__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


// Include directives for member types
// Member `axes`
#include "rosidl_runtime_c/string_functions.h"
// Member `target`
#include "rosidl_runtime_c/primitives_sequence_functions.h"

bool
uv_msgs__action__BasicMotion_Goal__init(uv_msgs__action__BasicMotion_Goal * msg)
{
  if (!msg) {
    return false;
  }
  // cmd_type
  // axes
  if (!rosidl_runtime_c__String__init(&msg->axes)) {
    uv_msgs__action__BasicMotion_Goal__fini(msg);
    return false;
  }
  // target
  if (!rosidl_runtime_c__float__Sequence__init(&msg->target, 0)) {
    uv_msgs__action__BasicMotion_Goal__fini(msg);
    return false;
  }
  // timeout
  return true;
}

void
uv_msgs__action__BasicMotion_Goal__fini(uv_msgs__action__BasicMotion_Goal * msg)
{
  if (!msg) {
    return;
  }
  // cmd_type
  // axes
  rosidl_runtime_c__String__fini(&msg->axes);
  // target
  rosidl_runtime_c__float__Sequence__fini(&msg->target);
  // timeout
}

bool
uv_msgs__action__BasicMotion_Goal__are_equal(const uv_msgs__action__BasicMotion_Goal * lhs, const uv_msgs__action__BasicMotion_Goal * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // cmd_type
  if (lhs->cmd_type != rhs->cmd_type) {
    return false;
  }
  // axes
  if (!rosidl_runtime_c__String__are_equal(
      &(lhs->axes), &(rhs->axes)))
  {
    return false;
  }
  // target
  if (!rosidl_runtime_c__float__Sequence__are_equal(
      &(lhs->target), &(rhs->target)))
  {
    return false;
  }
  // timeout
  if (lhs->timeout != rhs->timeout) {
    return false;
  }
  return true;
}

bool
uv_msgs__action__BasicMotion_Goal__copy(
  const uv_msgs__action__BasicMotion_Goal * input,
  uv_msgs__action__BasicMotion_Goal * output)
{
  if (!input || !output) {
    return false;
  }
  // cmd_type
  output->cmd_type = input->cmd_type;
  // axes
  if (!rosidl_runtime_c__String__copy(
      &(input->axes), &(output->axes)))
  {
    return false;
  }
  // target
  if (!rosidl_runtime_c__float__Sequence__copy(
      &(input->target), &(output->target)))
  {
    return false;
  }
  // timeout
  output->timeout = input->timeout;
  return true;
}

uv_msgs__action__BasicMotion_Goal *
uv_msgs__action__BasicMotion_Goal__create(void)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  uv_msgs__action__BasicMotion_Goal * msg = (uv_msgs__action__BasicMotion_Goal *)allocator.allocate(sizeof(uv_msgs__action__BasicMotion_Goal), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(uv_msgs__action__BasicMotion_Goal));
  bool success = uv_msgs__action__BasicMotion_Goal__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
uv_msgs__action__BasicMotion_Goal__destroy(uv_msgs__action__BasicMotion_Goal * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    uv_msgs__action__BasicMotion_Goal__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
uv_msgs__action__BasicMotion_Goal__Sequence__init(uv_msgs__action__BasicMotion_Goal__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  uv_msgs__action__BasicMotion_Goal * data = NULL;

  if (size) {
    data = (uv_msgs__action__BasicMotion_Goal *)allocator.zero_allocate(size, sizeof(uv_msgs__action__BasicMotion_Goal), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = uv_msgs__action__BasicMotion_Goal__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        uv_msgs__action__BasicMotion_Goal__fini(&data[i - 1]);
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
uv_msgs__action__BasicMotion_Goal__Sequence__fini(uv_msgs__action__BasicMotion_Goal__Sequence * array)
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
      uv_msgs__action__BasicMotion_Goal__fini(&array->data[i]);
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

uv_msgs__action__BasicMotion_Goal__Sequence *
uv_msgs__action__BasicMotion_Goal__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  uv_msgs__action__BasicMotion_Goal__Sequence * array = (uv_msgs__action__BasicMotion_Goal__Sequence *)allocator.allocate(sizeof(uv_msgs__action__BasicMotion_Goal__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = uv_msgs__action__BasicMotion_Goal__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
uv_msgs__action__BasicMotion_Goal__Sequence__destroy(uv_msgs__action__BasicMotion_Goal__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    uv_msgs__action__BasicMotion_Goal__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
uv_msgs__action__BasicMotion_Goal__Sequence__are_equal(const uv_msgs__action__BasicMotion_Goal__Sequence * lhs, const uv_msgs__action__BasicMotion_Goal__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!uv_msgs__action__BasicMotion_Goal__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
uv_msgs__action__BasicMotion_Goal__Sequence__copy(
  const uv_msgs__action__BasicMotion_Goal__Sequence * input,
  uv_msgs__action__BasicMotion_Goal__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(uv_msgs__action__BasicMotion_Goal);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    uv_msgs__action__BasicMotion_Goal * data =
      (uv_msgs__action__BasicMotion_Goal *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!uv_msgs__action__BasicMotion_Goal__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          uv_msgs__action__BasicMotion_Goal__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!uv_msgs__action__BasicMotion_Goal__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}


// Include directives for member types
// Member `message`
// already included above
// #include "rosidl_runtime_c/string_functions.h"

bool
uv_msgs__action__BasicMotion_Result__init(uv_msgs__action__BasicMotion_Result * msg)
{
  if (!msg) {
    return false;
  }
  // success
  // message
  if (!rosidl_runtime_c__String__init(&msg->message)) {
    uv_msgs__action__BasicMotion_Result__fini(msg);
    return false;
  }
  return true;
}

void
uv_msgs__action__BasicMotion_Result__fini(uv_msgs__action__BasicMotion_Result * msg)
{
  if (!msg) {
    return;
  }
  // success
  // message
  rosidl_runtime_c__String__fini(&msg->message);
}

bool
uv_msgs__action__BasicMotion_Result__are_equal(const uv_msgs__action__BasicMotion_Result * lhs, const uv_msgs__action__BasicMotion_Result * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // success
  if (lhs->success != rhs->success) {
    return false;
  }
  // message
  if (!rosidl_runtime_c__String__are_equal(
      &(lhs->message), &(rhs->message)))
  {
    return false;
  }
  return true;
}

bool
uv_msgs__action__BasicMotion_Result__copy(
  const uv_msgs__action__BasicMotion_Result * input,
  uv_msgs__action__BasicMotion_Result * output)
{
  if (!input || !output) {
    return false;
  }
  // success
  output->success = input->success;
  // message
  if (!rosidl_runtime_c__String__copy(
      &(input->message), &(output->message)))
  {
    return false;
  }
  return true;
}

uv_msgs__action__BasicMotion_Result *
uv_msgs__action__BasicMotion_Result__create(void)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  uv_msgs__action__BasicMotion_Result * msg = (uv_msgs__action__BasicMotion_Result *)allocator.allocate(sizeof(uv_msgs__action__BasicMotion_Result), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(uv_msgs__action__BasicMotion_Result));
  bool success = uv_msgs__action__BasicMotion_Result__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
uv_msgs__action__BasicMotion_Result__destroy(uv_msgs__action__BasicMotion_Result * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    uv_msgs__action__BasicMotion_Result__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
uv_msgs__action__BasicMotion_Result__Sequence__init(uv_msgs__action__BasicMotion_Result__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  uv_msgs__action__BasicMotion_Result * data = NULL;

  if (size) {
    data = (uv_msgs__action__BasicMotion_Result *)allocator.zero_allocate(size, sizeof(uv_msgs__action__BasicMotion_Result), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = uv_msgs__action__BasicMotion_Result__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        uv_msgs__action__BasicMotion_Result__fini(&data[i - 1]);
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
uv_msgs__action__BasicMotion_Result__Sequence__fini(uv_msgs__action__BasicMotion_Result__Sequence * array)
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
      uv_msgs__action__BasicMotion_Result__fini(&array->data[i]);
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

uv_msgs__action__BasicMotion_Result__Sequence *
uv_msgs__action__BasicMotion_Result__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  uv_msgs__action__BasicMotion_Result__Sequence * array = (uv_msgs__action__BasicMotion_Result__Sequence *)allocator.allocate(sizeof(uv_msgs__action__BasicMotion_Result__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = uv_msgs__action__BasicMotion_Result__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
uv_msgs__action__BasicMotion_Result__Sequence__destroy(uv_msgs__action__BasicMotion_Result__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    uv_msgs__action__BasicMotion_Result__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
uv_msgs__action__BasicMotion_Result__Sequence__are_equal(const uv_msgs__action__BasicMotion_Result__Sequence * lhs, const uv_msgs__action__BasicMotion_Result__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!uv_msgs__action__BasicMotion_Result__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
uv_msgs__action__BasicMotion_Result__Sequence__copy(
  const uv_msgs__action__BasicMotion_Result__Sequence * input,
  uv_msgs__action__BasicMotion_Result__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(uv_msgs__action__BasicMotion_Result);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    uv_msgs__action__BasicMotion_Result * data =
      (uv_msgs__action__BasicMotion_Result *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!uv_msgs__action__BasicMotion_Result__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          uv_msgs__action__BasicMotion_Result__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!uv_msgs__action__BasicMotion_Result__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}


bool
uv_msgs__action__BasicMotion_Feedback__init(uv_msgs__action__BasicMotion_Feedback * msg)
{
  if (!msg) {
    return false;
  }
  // distance_remaining
  return true;
}

void
uv_msgs__action__BasicMotion_Feedback__fini(uv_msgs__action__BasicMotion_Feedback * msg)
{
  if (!msg) {
    return;
  }
  // distance_remaining
}

bool
uv_msgs__action__BasicMotion_Feedback__are_equal(const uv_msgs__action__BasicMotion_Feedback * lhs, const uv_msgs__action__BasicMotion_Feedback * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // distance_remaining
  if (lhs->distance_remaining != rhs->distance_remaining) {
    return false;
  }
  return true;
}

bool
uv_msgs__action__BasicMotion_Feedback__copy(
  const uv_msgs__action__BasicMotion_Feedback * input,
  uv_msgs__action__BasicMotion_Feedback * output)
{
  if (!input || !output) {
    return false;
  }
  // distance_remaining
  output->distance_remaining = input->distance_remaining;
  return true;
}

uv_msgs__action__BasicMotion_Feedback *
uv_msgs__action__BasicMotion_Feedback__create(void)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  uv_msgs__action__BasicMotion_Feedback * msg = (uv_msgs__action__BasicMotion_Feedback *)allocator.allocate(sizeof(uv_msgs__action__BasicMotion_Feedback), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(uv_msgs__action__BasicMotion_Feedback));
  bool success = uv_msgs__action__BasicMotion_Feedback__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
uv_msgs__action__BasicMotion_Feedback__destroy(uv_msgs__action__BasicMotion_Feedback * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    uv_msgs__action__BasicMotion_Feedback__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
uv_msgs__action__BasicMotion_Feedback__Sequence__init(uv_msgs__action__BasicMotion_Feedback__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  uv_msgs__action__BasicMotion_Feedback * data = NULL;

  if (size) {
    data = (uv_msgs__action__BasicMotion_Feedback *)allocator.zero_allocate(size, sizeof(uv_msgs__action__BasicMotion_Feedback), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = uv_msgs__action__BasicMotion_Feedback__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        uv_msgs__action__BasicMotion_Feedback__fini(&data[i - 1]);
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
uv_msgs__action__BasicMotion_Feedback__Sequence__fini(uv_msgs__action__BasicMotion_Feedback__Sequence * array)
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
      uv_msgs__action__BasicMotion_Feedback__fini(&array->data[i]);
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

uv_msgs__action__BasicMotion_Feedback__Sequence *
uv_msgs__action__BasicMotion_Feedback__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  uv_msgs__action__BasicMotion_Feedback__Sequence * array = (uv_msgs__action__BasicMotion_Feedback__Sequence *)allocator.allocate(sizeof(uv_msgs__action__BasicMotion_Feedback__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = uv_msgs__action__BasicMotion_Feedback__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
uv_msgs__action__BasicMotion_Feedback__Sequence__destroy(uv_msgs__action__BasicMotion_Feedback__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    uv_msgs__action__BasicMotion_Feedback__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
uv_msgs__action__BasicMotion_Feedback__Sequence__are_equal(const uv_msgs__action__BasicMotion_Feedback__Sequence * lhs, const uv_msgs__action__BasicMotion_Feedback__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!uv_msgs__action__BasicMotion_Feedback__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
uv_msgs__action__BasicMotion_Feedback__Sequence__copy(
  const uv_msgs__action__BasicMotion_Feedback__Sequence * input,
  uv_msgs__action__BasicMotion_Feedback__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(uv_msgs__action__BasicMotion_Feedback);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    uv_msgs__action__BasicMotion_Feedback * data =
      (uv_msgs__action__BasicMotion_Feedback *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!uv_msgs__action__BasicMotion_Feedback__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          uv_msgs__action__BasicMotion_Feedback__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!uv_msgs__action__BasicMotion_Feedback__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}


// Include directives for member types
// Member `goal_id`
#include "unique_identifier_msgs/msg/detail/uuid__functions.h"
// Member `goal`
// already included above
// #include "uv_msgs/action/detail/basic_motion__functions.h"

bool
uv_msgs__action__BasicMotion_SendGoal_Request__init(uv_msgs__action__BasicMotion_SendGoal_Request * msg)
{
  if (!msg) {
    return false;
  }
  // goal_id
  if (!unique_identifier_msgs__msg__UUID__init(&msg->goal_id)) {
    uv_msgs__action__BasicMotion_SendGoal_Request__fini(msg);
    return false;
  }
  // goal
  if (!uv_msgs__action__BasicMotion_Goal__init(&msg->goal)) {
    uv_msgs__action__BasicMotion_SendGoal_Request__fini(msg);
    return false;
  }
  return true;
}

void
uv_msgs__action__BasicMotion_SendGoal_Request__fini(uv_msgs__action__BasicMotion_SendGoal_Request * msg)
{
  if (!msg) {
    return;
  }
  // goal_id
  unique_identifier_msgs__msg__UUID__fini(&msg->goal_id);
  // goal
  uv_msgs__action__BasicMotion_Goal__fini(&msg->goal);
}

bool
uv_msgs__action__BasicMotion_SendGoal_Request__are_equal(const uv_msgs__action__BasicMotion_SendGoal_Request * lhs, const uv_msgs__action__BasicMotion_SendGoal_Request * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // goal_id
  if (!unique_identifier_msgs__msg__UUID__are_equal(
      &(lhs->goal_id), &(rhs->goal_id)))
  {
    return false;
  }
  // goal
  if (!uv_msgs__action__BasicMotion_Goal__are_equal(
      &(lhs->goal), &(rhs->goal)))
  {
    return false;
  }
  return true;
}

bool
uv_msgs__action__BasicMotion_SendGoal_Request__copy(
  const uv_msgs__action__BasicMotion_SendGoal_Request * input,
  uv_msgs__action__BasicMotion_SendGoal_Request * output)
{
  if (!input || !output) {
    return false;
  }
  // goal_id
  if (!unique_identifier_msgs__msg__UUID__copy(
      &(input->goal_id), &(output->goal_id)))
  {
    return false;
  }
  // goal
  if (!uv_msgs__action__BasicMotion_Goal__copy(
      &(input->goal), &(output->goal)))
  {
    return false;
  }
  return true;
}

uv_msgs__action__BasicMotion_SendGoal_Request *
uv_msgs__action__BasicMotion_SendGoal_Request__create(void)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  uv_msgs__action__BasicMotion_SendGoal_Request * msg = (uv_msgs__action__BasicMotion_SendGoal_Request *)allocator.allocate(sizeof(uv_msgs__action__BasicMotion_SendGoal_Request), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(uv_msgs__action__BasicMotion_SendGoal_Request));
  bool success = uv_msgs__action__BasicMotion_SendGoal_Request__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
uv_msgs__action__BasicMotion_SendGoal_Request__destroy(uv_msgs__action__BasicMotion_SendGoal_Request * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    uv_msgs__action__BasicMotion_SendGoal_Request__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
uv_msgs__action__BasicMotion_SendGoal_Request__Sequence__init(uv_msgs__action__BasicMotion_SendGoal_Request__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  uv_msgs__action__BasicMotion_SendGoal_Request * data = NULL;

  if (size) {
    data = (uv_msgs__action__BasicMotion_SendGoal_Request *)allocator.zero_allocate(size, sizeof(uv_msgs__action__BasicMotion_SendGoal_Request), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = uv_msgs__action__BasicMotion_SendGoal_Request__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        uv_msgs__action__BasicMotion_SendGoal_Request__fini(&data[i - 1]);
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
uv_msgs__action__BasicMotion_SendGoal_Request__Sequence__fini(uv_msgs__action__BasicMotion_SendGoal_Request__Sequence * array)
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
      uv_msgs__action__BasicMotion_SendGoal_Request__fini(&array->data[i]);
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

uv_msgs__action__BasicMotion_SendGoal_Request__Sequence *
uv_msgs__action__BasicMotion_SendGoal_Request__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  uv_msgs__action__BasicMotion_SendGoal_Request__Sequence * array = (uv_msgs__action__BasicMotion_SendGoal_Request__Sequence *)allocator.allocate(sizeof(uv_msgs__action__BasicMotion_SendGoal_Request__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = uv_msgs__action__BasicMotion_SendGoal_Request__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
uv_msgs__action__BasicMotion_SendGoal_Request__Sequence__destroy(uv_msgs__action__BasicMotion_SendGoal_Request__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    uv_msgs__action__BasicMotion_SendGoal_Request__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
uv_msgs__action__BasicMotion_SendGoal_Request__Sequence__are_equal(const uv_msgs__action__BasicMotion_SendGoal_Request__Sequence * lhs, const uv_msgs__action__BasicMotion_SendGoal_Request__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!uv_msgs__action__BasicMotion_SendGoal_Request__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
uv_msgs__action__BasicMotion_SendGoal_Request__Sequence__copy(
  const uv_msgs__action__BasicMotion_SendGoal_Request__Sequence * input,
  uv_msgs__action__BasicMotion_SendGoal_Request__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(uv_msgs__action__BasicMotion_SendGoal_Request);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    uv_msgs__action__BasicMotion_SendGoal_Request * data =
      (uv_msgs__action__BasicMotion_SendGoal_Request *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!uv_msgs__action__BasicMotion_SendGoal_Request__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          uv_msgs__action__BasicMotion_SendGoal_Request__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!uv_msgs__action__BasicMotion_SendGoal_Request__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}


// Include directives for member types
// Member `stamp`
#include "builtin_interfaces/msg/detail/time__functions.h"

bool
uv_msgs__action__BasicMotion_SendGoal_Response__init(uv_msgs__action__BasicMotion_SendGoal_Response * msg)
{
  if (!msg) {
    return false;
  }
  // accepted
  // stamp
  if (!builtin_interfaces__msg__Time__init(&msg->stamp)) {
    uv_msgs__action__BasicMotion_SendGoal_Response__fini(msg);
    return false;
  }
  return true;
}

void
uv_msgs__action__BasicMotion_SendGoal_Response__fini(uv_msgs__action__BasicMotion_SendGoal_Response * msg)
{
  if (!msg) {
    return;
  }
  // accepted
  // stamp
  builtin_interfaces__msg__Time__fini(&msg->stamp);
}

bool
uv_msgs__action__BasicMotion_SendGoal_Response__are_equal(const uv_msgs__action__BasicMotion_SendGoal_Response * lhs, const uv_msgs__action__BasicMotion_SendGoal_Response * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // accepted
  if (lhs->accepted != rhs->accepted) {
    return false;
  }
  // stamp
  if (!builtin_interfaces__msg__Time__are_equal(
      &(lhs->stamp), &(rhs->stamp)))
  {
    return false;
  }
  return true;
}

bool
uv_msgs__action__BasicMotion_SendGoal_Response__copy(
  const uv_msgs__action__BasicMotion_SendGoal_Response * input,
  uv_msgs__action__BasicMotion_SendGoal_Response * output)
{
  if (!input || !output) {
    return false;
  }
  // accepted
  output->accepted = input->accepted;
  // stamp
  if (!builtin_interfaces__msg__Time__copy(
      &(input->stamp), &(output->stamp)))
  {
    return false;
  }
  return true;
}

uv_msgs__action__BasicMotion_SendGoal_Response *
uv_msgs__action__BasicMotion_SendGoal_Response__create(void)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  uv_msgs__action__BasicMotion_SendGoal_Response * msg = (uv_msgs__action__BasicMotion_SendGoal_Response *)allocator.allocate(sizeof(uv_msgs__action__BasicMotion_SendGoal_Response), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(uv_msgs__action__BasicMotion_SendGoal_Response));
  bool success = uv_msgs__action__BasicMotion_SendGoal_Response__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
uv_msgs__action__BasicMotion_SendGoal_Response__destroy(uv_msgs__action__BasicMotion_SendGoal_Response * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    uv_msgs__action__BasicMotion_SendGoal_Response__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
uv_msgs__action__BasicMotion_SendGoal_Response__Sequence__init(uv_msgs__action__BasicMotion_SendGoal_Response__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  uv_msgs__action__BasicMotion_SendGoal_Response * data = NULL;

  if (size) {
    data = (uv_msgs__action__BasicMotion_SendGoal_Response *)allocator.zero_allocate(size, sizeof(uv_msgs__action__BasicMotion_SendGoal_Response), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = uv_msgs__action__BasicMotion_SendGoal_Response__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        uv_msgs__action__BasicMotion_SendGoal_Response__fini(&data[i - 1]);
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
uv_msgs__action__BasicMotion_SendGoal_Response__Sequence__fini(uv_msgs__action__BasicMotion_SendGoal_Response__Sequence * array)
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
      uv_msgs__action__BasicMotion_SendGoal_Response__fini(&array->data[i]);
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

uv_msgs__action__BasicMotion_SendGoal_Response__Sequence *
uv_msgs__action__BasicMotion_SendGoal_Response__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  uv_msgs__action__BasicMotion_SendGoal_Response__Sequence * array = (uv_msgs__action__BasicMotion_SendGoal_Response__Sequence *)allocator.allocate(sizeof(uv_msgs__action__BasicMotion_SendGoal_Response__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = uv_msgs__action__BasicMotion_SendGoal_Response__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
uv_msgs__action__BasicMotion_SendGoal_Response__Sequence__destroy(uv_msgs__action__BasicMotion_SendGoal_Response__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    uv_msgs__action__BasicMotion_SendGoal_Response__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
uv_msgs__action__BasicMotion_SendGoal_Response__Sequence__are_equal(const uv_msgs__action__BasicMotion_SendGoal_Response__Sequence * lhs, const uv_msgs__action__BasicMotion_SendGoal_Response__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!uv_msgs__action__BasicMotion_SendGoal_Response__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
uv_msgs__action__BasicMotion_SendGoal_Response__Sequence__copy(
  const uv_msgs__action__BasicMotion_SendGoal_Response__Sequence * input,
  uv_msgs__action__BasicMotion_SendGoal_Response__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(uv_msgs__action__BasicMotion_SendGoal_Response);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    uv_msgs__action__BasicMotion_SendGoal_Response * data =
      (uv_msgs__action__BasicMotion_SendGoal_Response *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!uv_msgs__action__BasicMotion_SendGoal_Response__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          uv_msgs__action__BasicMotion_SendGoal_Response__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!uv_msgs__action__BasicMotion_SendGoal_Response__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}


// Include directives for member types
// Member `info`
#include "service_msgs/msg/detail/service_event_info__functions.h"
// Member `request`
// Member `response`
// already included above
// #include "uv_msgs/action/detail/basic_motion__functions.h"

bool
uv_msgs__action__BasicMotion_SendGoal_Event__init(uv_msgs__action__BasicMotion_SendGoal_Event * msg)
{
  if (!msg) {
    return false;
  }
  // info
  if (!service_msgs__msg__ServiceEventInfo__init(&msg->info)) {
    uv_msgs__action__BasicMotion_SendGoal_Event__fini(msg);
    return false;
  }
  // request
  if (!uv_msgs__action__BasicMotion_SendGoal_Request__Sequence__init(&msg->request, 0)) {
    uv_msgs__action__BasicMotion_SendGoal_Event__fini(msg);
    return false;
  }
  // response
  if (!uv_msgs__action__BasicMotion_SendGoal_Response__Sequence__init(&msg->response, 0)) {
    uv_msgs__action__BasicMotion_SendGoal_Event__fini(msg);
    return false;
  }
  return true;
}

void
uv_msgs__action__BasicMotion_SendGoal_Event__fini(uv_msgs__action__BasicMotion_SendGoal_Event * msg)
{
  if (!msg) {
    return;
  }
  // info
  service_msgs__msg__ServiceEventInfo__fini(&msg->info);
  // request
  uv_msgs__action__BasicMotion_SendGoal_Request__Sequence__fini(&msg->request);
  // response
  uv_msgs__action__BasicMotion_SendGoal_Response__Sequence__fini(&msg->response);
}

bool
uv_msgs__action__BasicMotion_SendGoal_Event__are_equal(const uv_msgs__action__BasicMotion_SendGoal_Event * lhs, const uv_msgs__action__BasicMotion_SendGoal_Event * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // info
  if (!service_msgs__msg__ServiceEventInfo__are_equal(
      &(lhs->info), &(rhs->info)))
  {
    return false;
  }
  // request
  if (!uv_msgs__action__BasicMotion_SendGoal_Request__Sequence__are_equal(
      &(lhs->request), &(rhs->request)))
  {
    return false;
  }
  // response
  if (!uv_msgs__action__BasicMotion_SendGoal_Response__Sequence__are_equal(
      &(lhs->response), &(rhs->response)))
  {
    return false;
  }
  return true;
}

bool
uv_msgs__action__BasicMotion_SendGoal_Event__copy(
  const uv_msgs__action__BasicMotion_SendGoal_Event * input,
  uv_msgs__action__BasicMotion_SendGoal_Event * output)
{
  if (!input || !output) {
    return false;
  }
  // info
  if (!service_msgs__msg__ServiceEventInfo__copy(
      &(input->info), &(output->info)))
  {
    return false;
  }
  // request
  if (!uv_msgs__action__BasicMotion_SendGoal_Request__Sequence__copy(
      &(input->request), &(output->request)))
  {
    return false;
  }
  // response
  if (!uv_msgs__action__BasicMotion_SendGoal_Response__Sequence__copy(
      &(input->response), &(output->response)))
  {
    return false;
  }
  return true;
}

uv_msgs__action__BasicMotion_SendGoal_Event *
uv_msgs__action__BasicMotion_SendGoal_Event__create(void)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  uv_msgs__action__BasicMotion_SendGoal_Event * msg = (uv_msgs__action__BasicMotion_SendGoal_Event *)allocator.allocate(sizeof(uv_msgs__action__BasicMotion_SendGoal_Event), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(uv_msgs__action__BasicMotion_SendGoal_Event));
  bool success = uv_msgs__action__BasicMotion_SendGoal_Event__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
uv_msgs__action__BasicMotion_SendGoal_Event__destroy(uv_msgs__action__BasicMotion_SendGoal_Event * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    uv_msgs__action__BasicMotion_SendGoal_Event__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
uv_msgs__action__BasicMotion_SendGoal_Event__Sequence__init(uv_msgs__action__BasicMotion_SendGoal_Event__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  uv_msgs__action__BasicMotion_SendGoal_Event * data = NULL;

  if (size) {
    data = (uv_msgs__action__BasicMotion_SendGoal_Event *)allocator.zero_allocate(size, sizeof(uv_msgs__action__BasicMotion_SendGoal_Event), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = uv_msgs__action__BasicMotion_SendGoal_Event__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        uv_msgs__action__BasicMotion_SendGoal_Event__fini(&data[i - 1]);
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
uv_msgs__action__BasicMotion_SendGoal_Event__Sequence__fini(uv_msgs__action__BasicMotion_SendGoal_Event__Sequence * array)
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
      uv_msgs__action__BasicMotion_SendGoal_Event__fini(&array->data[i]);
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

uv_msgs__action__BasicMotion_SendGoal_Event__Sequence *
uv_msgs__action__BasicMotion_SendGoal_Event__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  uv_msgs__action__BasicMotion_SendGoal_Event__Sequence * array = (uv_msgs__action__BasicMotion_SendGoal_Event__Sequence *)allocator.allocate(sizeof(uv_msgs__action__BasicMotion_SendGoal_Event__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = uv_msgs__action__BasicMotion_SendGoal_Event__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
uv_msgs__action__BasicMotion_SendGoal_Event__Sequence__destroy(uv_msgs__action__BasicMotion_SendGoal_Event__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    uv_msgs__action__BasicMotion_SendGoal_Event__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
uv_msgs__action__BasicMotion_SendGoal_Event__Sequence__are_equal(const uv_msgs__action__BasicMotion_SendGoal_Event__Sequence * lhs, const uv_msgs__action__BasicMotion_SendGoal_Event__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!uv_msgs__action__BasicMotion_SendGoal_Event__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
uv_msgs__action__BasicMotion_SendGoal_Event__Sequence__copy(
  const uv_msgs__action__BasicMotion_SendGoal_Event__Sequence * input,
  uv_msgs__action__BasicMotion_SendGoal_Event__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(uv_msgs__action__BasicMotion_SendGoal_Event);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    uv_msgs__action__BasicMotion_SendGoal_Event * data =
      (uv_msgs__action__BasicMotion_SendGoal_Event *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!uv_msgs__action__BasicMotion_SendGoal_Event__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          uv_msgs__action__BasicMotion_SendGoal_Event__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!uv_msgs__action__BasicMotion_SendGoal_Event__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}


// Include directives for member types
// Member `goal_id`
// already included above
// #include "unique_identifier_msgs/msg/detail/uuid__functions.h"

bool
uv_msgs__action__BasicMotion_GetResult_Request__init(uv_msgs__action__BasicMotion_GetResult_Request * msg)
{
  if (!msg) {
    return false;
  }
  // goal_id
  if (!unique_identifier_msgs__msg__UUID__init(&msg->goal_id)) {
    uv_msgs__action__BasicMotion_GetResult_Request__fini(msg);
    return false;
  }
  return true;
}

void
uv_msgs__action__BasicMotion_GetResult_Request__fini(uv_msgs__action__BasicMotion_GetResult_Request * msg)
{
  if (!msg) {
    return;
  }
  // goal_id
  unique_identifier_msgs__msg__UUID__fini(&msg->goal_id);
}

bool
uv_msgs__action__BasicMotion_GetResult_Request__are_equal(const uv_msgs__action__BasicMotion_GetResult_Request * lhs, const uv_msgs__action__BasicMotion_GetResult_Request * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // goal_id
  if (!unique_identifier_msgs__msg__UUID__are_equal(
      &(lhs->goal_id), &(rhs->goal_id)))
  {
    return false;
  }
  return true;
}

bool
uv_msgs__action__BasicMotion_GetResult_Request__copy(
  const uv_msgs__action__BasicMotion_GetResult_Request * input,
  uv_msgs__action__BasicMotion_GetResult_Request * output)
{
  if (!input || !output) {
    return false;
  }
  // goal_id
  if (!unique_identifier_msgs__msg__UUID__copy(
      &(input->goal_id), &(output->goal_id)))
  {
    return false;
  }
  return true;
}

uv_msgs__action__BasicMotion_GetResult_Request *
uv_msgs__action__BasicMotion_GetResult_Request__create(void)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  uv_msgs__action__BasicMotion_GetResult_Request * msg = (uv_msgs__action__BasicMotion_GetResult_Request *)allocator.allocate(sizeof(uv_msgs__action__BasicMotion_GetResult_Request), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(uv_msgs__action__BasicMotion_GetResult_Request));
  bool success = uv_msgs__action__BasicMotion_GetResult_Request__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
uv_msgs__action__BasicMotion_GetResult_Request__destroy(uv_msgs__action__BasicMotion_GetResult_Request * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    uv_msgs__action__BasicMotion_GetResult_Request__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
uv_msgs__action__BasicMotion_GetResult_Request__Sequence__init(uv_msgs__action__BasicMotion_GetResult_Request__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  uv_msgs__action__BasicMotion_GetResult_Request * data = NULL;

  if (size) {
    data = (uv_msgs__action__BasicMotion_GetResult_Request *)allocator.zero_allocate(size, sizeof(uv_msgs__action__BasicMotion_GetResult_Request), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = uv_msgs__action__BasicMotion_GetResult_Request__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        uv_msgs__action__BasicMotion_GetResult_Request__fini(&data[i - 1]);
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
uv_msgs__action__BasicMotion_GetResult_Request__Sequence__fini(uv_msgs__action__BasicMotion_GetResult_Request__Sequence * array)
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
      uv_msgs__action__BasicMotion_GetResult_Request__fini(&array->data[i]);
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

uv_msgs__action__BasicMotion_GetResult_Request__Sequence *
uv_msgs__action__BasicMotion_GetResult_Request__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  uv_msgs__action__BasicMotion_GetResult_Request__Sequence * array = (uv_msgs__action__BasicMotion_GetResult_Request__Sequence *)allocator.allocate(sizeof(uv_msgs__action__BasicMotion_GetResult_Request__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = uv_msgs__action__BasicMotion_GetResult_Request__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
uv_msgs__action__BasicMotion_GetResult_Request__Sequence__destroy(uv_msgs__action__BasicMotion_GetResult_Request__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    uv_msgs__action__BasicMotion_GetResult_Request__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
uv_msgs__action__BasicMotion_GetResult_Request__Sequence__are_equal(const uv_msgs__action__BasicMotion_GetResult_Request__Sequence * lhs, const uv_msgs__action__BasicMotion_GetResult_Request__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!uv_msgs__action__BasicMotion_GetResult_Request__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
uv_msgs__action__BasicMotion_GetResult_Request__Sequence__copy(
  const uv_msgs__action__BasicMotion_GetResult_Request__Sequence * input,
  uv_msgs__action__BasicMotion_GetResult_Request__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(uv_msgs__action__BasicMotion_GetResult_Request);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    uv_msgs__action__BasicMotion_GetResult_Request * data =
      (uv_msgs__action__BasicMotion_GetResult_Request *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!uv_msgs__action__BasicMotion_GetResult_Request__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          uv_msgs__action__BasicMotion_GetResult_Request__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!uv_msgs__action__BasicMotion_GetResult_Request__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}


// Include directives for member types
// Member `result`
// already included above
// #include "uv_msgs/action/detail/basic_motion__functions.h"

bool
uv_msgs__action__BasicMotion_GetResult_Response__init(uv_msgs__action__BasicMotion_GetResult_Response * msg)
{
  if (!msg) {
    return false;
  }
  // status
  // result
  if (!uv_msgs__action__BasicMotion_Result__init(&msg->result)) {
    uv_msgs__action__BasicMotion_GetResult_Response__fini(msg);
    return false;
  }
  return true;
}

void
uv_msgs__action__BasicMotion_GetResult_Response__fini(uv_msgs__action__BasicMotion_GetResult_Response * msg)
{
  if (!msg) {
    return;
  }
  // status
  // result
  uv_msgs__action__BasicMotion_Result__fini(&msg->result);
}

bool
uv_msgs__action__BasicMotion_GetResult_Response__are_equal(const uv_msgs__action__BasicMotion_GetResult_Response * lhs, const uv_msgs__action__BasicMotion_GetResult_Response * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // status
  if (lhs->status != rhs->status) {
    return false;
  }
  // result
  if (!uv_msgs__action__BasicMotion_Result__are_equal(
      &(lhs->result), &(rhs->result)))
  {
    return false;
  }
  return true;
}

bool
uv_msgs__action__BasicMotion_GetResult_Response__copy(
  const uv_msgs__action__BasicMotion_GetResult_Response * input,
  uv_msgs__action__BasicMotion_GetResult_Response * output)
{
  if (!input || !output) {
    return false;
  }
  // status
  output->status = input->status;
  // result
  if (!uv_msgs__action__BasicMotion_Result__copy(
      &(input->result), &(output->result)))
  {
    return false;
  }
  return true;
}

uv_msgs__action__BasicMotion_GetResult_Response *
uv_msgs__action__BasicMotion_GetResult_Response__create(void)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  uv_msgs__action__BasicMotion_GetResult_Response * msg = (uv_msgs__action__BasicMotion_GetResult_Response *)allocator.allocate(sizeof(uv_msgs__action__BasicMotion_GetResult_Response), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(uv_msgs__action__BasicMotion_GetResult_Response));
  bool success = uv_msgs__action__BasicMotion_GetResult_Response__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
uv_msgs__action__BasicMotion_GetResult_Response__destroy(uv_msgs__action__BasicMotion_GetResult_Response * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    uv_msgs__action__BasicMotion_GetResult_Response__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
uv_msgs__action__BasicMotion_GetResult_Response__Sequence__init(uv_msgs__action__BasicMotion_GetResult_Response__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  uv_msgs__action__BasicMotion_GetResult_Response * data = NULL;

  if (size) {
    data = (uv_msgs__action__BasicMotion_GetResult_Response *)allocator.zero_allocate(size, sizeof(uv_msgs__action__BasicMotion_GetResult_Response), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = uv_msgs__action__BasicMotion_GetResult_Response__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        uv_msgs__action__BasicMotion_GetResult_Response__fini(&data[i - 1]);
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
uv_msgs__action__BasicMotion_GetResult_Response__Sequence__fini(uv_msgs__action__BasicMotion_GetResult_Response__Sequence * array)
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
      uv_msgs__action__BasicMotion_GetResult_Response__fini(&array->data[i]);
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

uv_msgs__action__BasicMotion_GetResult_Response__Sequence *
uv_msgs__action__BasicMotion_GetResult_Response__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  uv_msgs__action__BasicMotion_GetResult_Response__Sequence * array = (uv_msgs__action__BasicMotion_GetResult_Response__Sequence *)allocator.allocate(sizeof(uv_msgs__action__BasicMotion_GetResult_Response__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = uv_msgs__action__BasicMotion_GetResult_Response__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
uv_msgs__action__BasicMotion_GetResult_Response__Sequence__destroy(uv_msgs__action__BasicMotion_GetResult_Response__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    uv_msgs__action__BasicMotion_GetResult_Response__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
uv_msgs__action__BasicMotion_GetResult_Response__Sequence__are_equal(const uv_msgs__action__BasicMotion_GetResult_Response__Sequence * lhs, const uv_msgs__action__BasicMotion_GetResult_Response__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!uv_msgs__action__BasicMotion_GetResult_Response__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
uv_msgs__action__BasicMotion_GetResult_Response__Sequence__copy(
  const uv_msgs__action__BasicMotion_GetResult_Response__Sequence * input,
  uv_msgs__action__BasicMotion_GetResult_Response__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(uv_msgs__action__BasicMotion_GetResult_Response);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    uv_msgs__action__BasicMotion_GetResult_Response * data =
      (uv_msgs__action__BasicMotion_GetResult_Response *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!uv_msgs__action__BasicMotion_GetResult_Response__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          uv_msgs__action__BasicMotion_GetResult_Response__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!uv_msgs__action__BasicMotion_GetResult_Response__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}


// Include directives for member types
// Member `info`
// already included above
// #include "service_msgs/msg/detail/service_event_info__functions.h"
// Member `request`
// Member `response`
// already included above
// #include "uv_msgs/action/detail/basic_motion__functions.h"

bool
uv_msgs__action__BasicMotion_GetResult_Event__init(uv_msgs__action__BasicMotion_GetResult_Event * msg)
{
  if (!msg) {
    return false;
  }
  // info
  if (!service_msgs__msg__ServiceEventInfo__init(&msg->info)) {
    uv_msgs__action__BasicMotion_GetResult_Event__fini(msg);
    return false;
  }
  // request
  if (!uv_msgs__action__BasicMotion_GetResult_Request__Sequence__init(&msg->request, 0)) {
    uv_msgs__action__BasicMotion_GetResult_Event__fini(msg);
    return false;
  }
  // response
  if (!uv_msgs__action__BasicMotion_GetResult_Response__Sequence__init(&msg->response, 0)) {
    uv_msgs__action__BasicMotion_GetResult_Event__fini(msg);
    return false;
  }
  return true;
}

void
uv_msgs__action__BasicMotion_GetResult_Event__fini(uv_msgs__action__BasicMotion_GetResult_Event * msg)
{
  if (!msg) {
    return;
  }
  // info
  service_msgs__msg__ServiceEventInfo__fini(&msg->info);
  // request
  uv_msgs__action__BasicMotion_GetResult_Request__Sequence__fini(&msg->request);
  // response
  uv_msgs__action__BasicMotion_GetResult_Response__Sequence__fini(&msg->response);
}

bool
uv_msgs__action__BasicMotion_GetResult_Event__are_equal(const uv_msgs__action__BasicMotion_GetResult_Event * lhs, const uv_msgs__action__BasicMotion_GetResult_Event * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // info
  if (!service_msgs__msg__ServiceEventInfo__are_equal(
      &(lhs->info), &(rhs->info)))
  {
    return false;
  }
  // request
  if (!uv_msgs__action__BasicMotion_GetResult_Request__Sequence__are_equal(
      &(lhs->request), &(rhs->request)))
  {
    return false;
  }
  // response
  if (!uv_msgs__action__BasicMotion_GetResult_Response__Sequence__are_equal(
      &(lhs->response), &(rhs->response)))
  {
    return false;
  }
  return true;
}

bool
uv_msgs__action__BasicMotion_GetResult_Event__copy(
  const uv_msgs__action__BasicMotion_GetResult_Event * input,
  uv_msgs__action__BasicMotion_GetResult_Event * output)
{
  if (!input || !output) {
    return false;
  }
  // info
  if (!service_msgs__msg__ServiceEventInfo__copy(
      &(input->info), &(output->info)))
  {
    return false;
  }
  // request
  if (!uv_msgs__action__BasicMotion_GetResult_Request__Sequence__copy(
      &(input->request), &(output->request)))
  {
    return false;
  }
  // response
  if (!uv_msgs__action__BasicMotion_GetResult_Response__Sequence__copy(
      &(input->response), &(output->response)))
  {
    return false;
  }
  return true;
}

uv_msgs__action__BasicMotion_GetResult_Event *
uv_msgs__action__BasicMotion_GetResult_Event__create(void)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  uv_msgs__action__BasicMotion_GetResult_Event * msg = (uv_msgs__action__BasicMotion_GetResult_Event *)allocator.allocate(sizeof(uv_msgs__action__BasicMotion_GetResult_Event), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(uv_msgs__action__BasicMotion_GetResult_Event));
  bool success = uv_msgs__action__BasicMotion_GetResult_Event__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
uv_msgs__action__BasicMotion_GetResult_Event__destroy(uv_msgs__action__BasicMotion_GetResult_Event * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    uv_msgs__action__BasicMotion_GetResult_Event__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
uv_msgs__action__BasicMotion_GetResult_Event__Sequence__init(uv_msgs__action__BasicMotion_GetResult_Event__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  uv_msgs__action__BasicMotion_GetResult_Event * data = NULL;

  if (size) {
    data = (uv_msgs__action__BasicMotion_GetResult_Event *)allocator.zero_allocate(size, sizeof(uv_msgs__action__BasicMotion_GetResult_Event), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = uv_msgs__action__BasicMotion_GetResult_Event__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        uv_msgs__action__BasicMotion_GetResult_Event__fini(&data[i - 1]);
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
uv_msgs__action__BasicMotion_GetResult_Event__Sequence__fini(uv_msgs__action__BasicMotion_GetResult_Event__Sequence * array)
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
      uv_msgs__action__BasicMotion_GetResult_Event__fini(&array->data[i]);
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

uv_msgs__action__BasicMotion_GetResult_Event__Sequence *
uv_msgs__action__BasicMotion_GetResult_Event__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  uv_msgs__action__BasicMotion_GetResult_Event__Sequence * array = (uv_msgs__action__BasicMotion_GetResult_Event__Sequence *)allocator.allocate(sizeof(uv_msgs__action__BasicMotion_GetResult_Event__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = uv_msgs__action__BasicMotion_GetResult_Event__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
uv_msgs__action__BasicMotion_GetResult_Event__Sequence__destroy(uv_msgs__action__BasicMotion_GetResult_Event__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    uv_msgs__action__BasicMotion_GetResult_Event__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
uv_msgs__action__BasicMotion_GetResult_Event__Sequence__are_equal(const uv_msgs__action__BasicMotion_GetResult_Event__Sequence * lhs, const uv_msgs__action__BasicMotion_GetResult_Event__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!uv_msgs__action__BasicMotion_GetResult_Event__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
uv_msgs__action__BasicMotion_GetResult_Event__Sequence__copy(
  const uv_msgs__action__BasicMotion_GetResult_Event__Sequence * input,
  uv_msgs__action__BasicMotion_GetResult_Event__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(uv_msgs__action__BasicMotion_GetResult_Event);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    uv_msgs__action__BasicMotion_GetResult_Event * data =
      (uv_msgs__action__BasicMotion_GetResult_Event *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!uv_msgs__action__BasicMotion_GetResult_Event__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          uv_msgs__action__BasicMotion_GetResult_Event__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!uv_msgs__action__BasicMotion_GetResult_Event__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}


// Include directives for member types
// Member `goal_id`
// already included above
// #include "unique_identifier_msgs/msg/detail/uuid__functions.h"
// Member `feedback`
// already included above
// #include "uv_msgs/action/detail/basic_motion__functions.h"

bool
uv_msgs__action__BasicMotion_FeedbackMessage__init(uv_msgs__action__BasicMotion_FeedbackMessage * msg)
{
  if (!msg) {
    return false;
  }
  // goal_id
  if (!unique_identifier_msgs__msg__UUID__init(&msg->goal_id)) {
    uv_msgs__action__BasicMotion_FeedbackMessage__fini(msg);
    return false;
  }
  // feedback
  if (!uv_msgs__action__BasicMotion_Feedback__init(&msg->feedback)) {
    uv_msgs__action__BasicMotion_FeedbackMessage__fini(msg);
    return false;
  }
  return true;
}

void
uv_msgs__action__BasicMotion_FeedbackMessage__fini(uv_msgs__action__BasicMotion_FeedbackMessage * msg)
{
  if (!msg) {
    return;
  }
  // goal_id
  unique_identifier_msgs__msg__UUID__fini(&msg->goal_id);
  // feedback
  uv_msgs__action__BasicMotion_Feedback__fini(&msg->feedback);
}

bool
uv_msgs__action__BasicMotion_FeedbackMessage__are_equal(const uv_msgs__action__BasicMotion_FeedbackMessage * lhs, const uv_msgs__action__BasicMotion_FeedbackMessage * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // goal_id
  if (!unique_identifier_msgs__msg__UUID__are_equal(
      &(lhs->goal_id), &(rhs->goal_id)))
  {
    return false;
  }
  // feedback
  if (!uv_msgs__action__BasicMotion_Feedback__are_equal(
      &(lhs->feedback), &(rhs->feedback)))
  {
    return false;
  }
  return true;
}

bool
uv_msgs__action__BasicMotion_FeedbackMessage__copy(
  const uv_msgs__action__BasicMotion_FeedbackMessage * input,
  uv_msgs__action__BasicMotion_FeedbackMessage * output)
{
  if (!input || !output) {
    return false;
  }
  // goal_id
  if (!unique_identifier_msgs__msg__UUID__copy(
      &(input->goal_id), &(output->goal_id)))
  {
    return false;
  }
  // feedback
  if (!uv_msgs__action__BasicMotion_Feedback__copy(
      &(input->feedback), &(output->feedback)))
  {
    return false;
  }
  return true;
}

uv_msgs__action__BasicMotion_FeedbackMessage *
uv_msgs__action__BasicMotion_FeedbackMessage__create(void)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  uv_msgs__action__BasicMotion_FeedbackMessage * msg = (uv_msgs__action__BasicMotion_FeedbackMessage *)allocator.allocate(sizeof(uv_msgs__action__BasicMotion_FeedbackMessage), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(uv_msgs__action__BasicMotion_FeedbackMessage));
  bool success = uv_msgs__action__BasicMotion_FeedbackMessage__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
uv_msgs__action__BasicMotion_FeedbackMessage__destroy(uv_msgs__action__BasicMotion_FeedbackMessage * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    uv_msgs__action__BasicMotion_FeedbackMessage__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
uv_msgs__action__BasicMotion_FeedbackMessage__Sequence__init(uv_msgs__action__BasicMotion_FeedbackMessage__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  uv_msgs__action__BasicMotion_FeedbackMessage * data = NULL;

  if (size) {
    data = (uv_msgs__action__BasicMotion_FeedbackMessage *)allocator.zero_allocate(size, sizeof(uv_msgs__action__BasicMotion_FeedbackMessage), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = uv_msgs__action__BasicMotion_FeedbackMessage__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        uv_msgs__action__BasicMotion_FeedbackMessage__fini(&data[i - 1]);
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
uv_msgs__action__BasicMotion_FeedbackMessage__Sequence__fini(uv_msgs__action__BasicMotion_FeedbackMessage__Sequence * array)
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
      uv_msgs__action__BasicMotion_FeedbackMessage__fini(&array->data[i]);
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

uv_msgs__action__BasicMotion_FeedbackMessage__Sequence *
uv_msgs__action__BasicMotion_FeedbackMessage__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  uv_msgs__action__BasicMotion_FeedbackMessage__Sequence * array = (uv_msgs__action__BasicMotion_FeedbackMessage__Sequence *)allocator.allocate(sizeof(uv_msgs__action__BasicMotion_FeedbackMessage__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = uv_msgs__action__BasicMotion_FeedbackMessage__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
uv_msgs__action__BasicMotion_FeedbackMessage__Sequence__destroy(uv_msgs__action__BasicMotion_FeedbackMessage__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    uv_msgs__action__BasicMotion_FeedbackMessage__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
uv_msgs__action__BasicMotion_FeedbackMessage__Sequence__are_equal(const uv_msgs__action__BasicMotion_FeedbackMessage__Sequence * lhs, const uv_msgs__action__BasicMotion_FeedbackMessage__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!uv_msgs__action__BasicMotion_FeedbackMessage__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
uv_msgs__action__BasicMotion_FeedbackMessage__Sequence__copy(
  const uv_msgs__action__BasicMotion_FeedbackMessage__Sequence * input,
  uv_msgs__action__BasicMotion_FeedbackMessage__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(uv_msgs__action__BasicMotion_FeedbackMessage);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    uv_msgs__action__BasicMotion_FeedbackMessage * data =
      (uv_msgs__action__BasicMotion_FeedbackMessage *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!uv_msgs__action__BasicMotion_FeedbackMessage__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          uv_msgs__action__BasicMotion_FeedbackMessage__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!uv_msgs__action__BasicMotion_FeedbackMessage__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
