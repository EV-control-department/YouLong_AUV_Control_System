// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from uv_msgs:srv/GetState.idl
// generated code does not contain a copyright notice
#include "uv_msgs/srv/detail/get_state__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"

bool
uv_msgs__srv__GetState_Request__init(uv_msgs__srv__GetState_Request * msg)
{
  if (!msg) {
    return false;
  }
  // structure_needs_at_least_one_member
  return true;
}

void
uv_msgs__srv__GetState_Request__fini(uv_msgs__srv__GetState_Request * msg)
{
  if (!msg) {
    return;
  }
  // structure_needs_at_least_one_member
}

bool
uv_msgs__srv__GetState_Request__are_equal(const uv_msgs__srv__GetState_Request * lhs, const uv_msgs__srv__GetState_Request * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // structure_needs_at_least_one_member
  if (lhs->structure_needs_at_least_one_member != rhs->structure_needs_at_least_one_member) {
    return false;
  }
  return true;
}

bool
uv_msgs__srv__GetState_Request__copy(
  const uv_msgs__srv__GetState_Request * input,
  uv_msgs__srv__GetState_Request * output)
{
  if (!input || !output) {
    return false;
  }
  // structure_needs_at_least_one_member
  output->structure_needs_at_least_one_member = input->structure_needs_at_least_one_member;
  return true;
}

uv_msgs__srv__GetState_Request *
uv_msgs__srv__GetState_Request__create(void)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  uv_msgs__srv__GetState_Request * msg = (uv_msgs__srv__GetState_Request *)allocator.allocate(sizeof(uv_msgs__srv__GetState_Request), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(uv_msgs__srv__GetState_Request));
  bool success = uv_msgs__srv__GetState_Request__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
uv_msgs__srv__GetState_Request__destroy(uv_msgs__srv__GetState_Request * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    uv_msgs__srv__GetState_Request__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
uv_msgs__srv__GetState_Request__Sequence__init(uv_msgs__srv__GetState_Request__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  uv_msgs__srv__GetState_Request * data = NULL;

  if (size) {
    data = (uv_msgs__srv__GetState_Request *)allocator.zero_allocate(size, sizeof(uv_msgs__srv__GetState_Request), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = uv_msgs__srv__GetState_Request__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        uv_msgs__srv__GetState_Request__fini(&data[i - 1]);
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
uv_msgs__srv__GetState_Request__Sequence__fini(uv_msgs__srv__GetState_Request__Sequence * array)
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
      uv_msgs__srv__GetState_Request__fini(&array->data[i]);
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

uv_msgs__srv__GetState_Request__Sequence *
uv_msgs__srv__GetState_Request__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  uv_msgs__srv__GetState_Request__Sequence * array = (uv_msgs__srv__GetState_Request__Sequence *)allocator.allocate(sizeof(uv_msgs__srv__GetState_Request__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = uv_msgs__srv__GetState_Request__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
uv_msgs__srv__GetState_Request__Sequence__destroy(uv_msgs__srv__GetState_Request__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    uv_msgs__srv__GetState_Request__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
uv_msgs__srv__GetState_Request__Sequence__are_equal(const uv_msgs__srv__GetState_Request__Sequence * lhs, const uv_msgs__srv__GetState_Request__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!uv_msgs__srv__GetState_Request__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
uv_msgs__srv__GetState_Request__Sequence__copy(
  const uv_msgs__srv__GetState_Request__Sequence * input,
  uv_msgs__srv__GetState_Request__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(uv_msgs__srv__GetState_Request);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    uv_msgs__srv__GetState_Request * data =
      (uv_msgs__srv__GetState_Request *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!uv_msgs__srv__GetState_Request__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          uv_msgs__srv__GetState_Request__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!uv_msgs__srv__GetState_Request__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}


// Include directives for member types
// Member `state`
#include "uv_msgs/msg/detail/auv_state__functions.h"

bool
uv_msgs__srv__GetState_Response__init(uv_msgs__srv__GetState_Response * msg)
{
  if (!msg) {
    return false;
  }
  // state
  if (!uv_msgs__msg__AuvState__init(&msg->state)) {
    uv_msgs__srv__GetState_Response__fini(msg);
    return false;
  }
  return true;
}

void
uv_msgs__srv__GetState_Response__fini(uv_msgs__srv__GetState_Response * msg)
{
  if (!msg) {
    return;
  }
  // state
  uv_msgs__msg__AuvState__fini(&msg->state);
}

bool
uv_msgs__srv__GetState_Response__are_equal(const uv_msgs__srv__GetState_Response * lhs, const uv_msgs__srv__GetState_Response * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // state
  if (!uv_msgs__msg__AuvState__are_equal(
      &(lhs->state), &(rhs->state)))
  {
    return false;
  }
  return true;
}

bool
uv_msgs__srv__GetState_Response__copy(
  const uv_msgs__srv__GetState_Response * input,
  uv_msgs__srv__GetState_Response * output)
{
  if (!input || !output) {
    return false;
  }
  // state
  if (!uv_msgs__msg__AuvState__copy(
      &(input->state), &(output->state)))
  {
    return false;
  }
  return true;
}

uv_msgs__srv__GetState_Response *
uv_msgs__srv__GetState_Response__create(void)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  uv_msgs__srv__GetState_Response * msg = (uv_msgs__srv__GetState_Response *)allocator.allocate(sizeof(uv_msgs__srv__GetState_Response), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(uv_msgs__srv__GetState_Response));
  bool success = uv_msgs__srv__GetState_Response__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
uv_msgs__srv__GetState_Response__destroy(uv_msgs__srv__GetState_Response * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    uv_msgs__srv__GetState_Response__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
uv_msgs__srv__GetState_Response__Sequence__init(uv_msgs__srv__GetState_Response__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  uv_msgs__srv__GetState_Response * data = NULL;

  if (size) {
    data = (uv_msgs__srv__GetState_Response *)allocator.zero_allocate(size, sizeof(uv_msgs__srv__GetState_Response), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = uv_msgs__srv__GetState_Response__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        uv_msgs__srv__GetState_Response__fini(&data[i - 1]);
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
uv_msgs__srv__GetState_Response__Sequence__fini(uv_msgs__srv__GetState_Response__Sequence * array)
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
      uv_msgs__srv__GetState_Response__fini(&array->data[i]);
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

uv_msgs__srv__GetState_Response__Sequence *
uv_msgs__srv__GetState_Response__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  uv_msgs__srv__GetState_Response__Sequence * array = (uv_msgs__srv__GetState_Response__Sequence *)allocator.allocate(sizeof(uv_msgs__srv__GetState_Response__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = uv_msgs__srv__GetState_Response__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
uv_msgs__srv__GetState_Response__Sequence__destroy(uv_msgs__srv__GetState_Response__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    uv_msgs__srv__GetState_Response__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
uv_msgs__srv__GetState_Response__Sequence__are_equal(const uv_msgs__srv__GetState_Response__Sequence * lhs, const uv_msgs__srv__GetState_Response__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!uv_msgs__srv__GetState_Response__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
uv_msgs__srv__GetState_Response__Sequence__copy(
  const uv_msgs__srv__GetState_Response__Sequence * input,
  uv_msgs__srv__GetState_Response__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(uv_msgs__srv__GetState_Response);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    uv_msgs__srv__GetState_Response * data =
      (uv_msgs__srv__GetState_Response *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!uv_msgs__srv__GetState_Response__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          uv_msgs__srv__GetState_Response__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!uv_msgs__srv__GetState_Response__copy(
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
// #include "uv_msgs/srv/detail/get_state__functions.h"

bool
uv_msgs__srv__GetState_Event__init(uv_msgs__srv__GetState_Event * msg)
{
  if (!msg) {
    return false;
  }
  // info
  if (!service_msgs__msg__ServiceEventInfo__init(&msg->info)) {
    uv_msgs__srv__GetState_Event__fini(msg);
    return false;
  }
  // request
  if (!uv_msgs__srv__GetState_Request__Sequence__init(&msg->request, 0)) {
    uv_msgs__srv__GetState_Event__fini(msg);
    return false;
  }
  // response
  if (!uv_msgs__srv__GetState_Response__Sequence__init(&msg->response, 0)) {
    uv_msgs__srv__GetState_Event__fini(msg);
    return false;
  }
  return true;
}

void
uv_msgs__srv__GetState_Event__fini(uv_msgs__srv__GetState_Event * msg)
{
  if (!msg) {
    return;
  }
  // info
  service_msgs__msg__ServiceEventInfo__fini(&msg->info);
  // request
  uv_msgs__srv__GetState_Request__Sequence__fini(&msg->request);
  // response
  uv_msgs__srv__GetState_Response__Sequence__fini(&msg->response);
}

bool
uv_msgs__srv__GetState_Event__are_equal(const uv_msgs__srv__GetState_Event * lhs, const uv_msgs__srv__GetState_Event * rhs)
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
  if (!uv_msgs__srv__GetState_Request__Sequence__are_equal(
      &(lhs->request), &(rhs->request)))
  {
    return false;
  }
  // response
  if (!uv_msgs__srv__GetState_Response__Sequence__are_equal(
      &(lhs->response), &(rhs->response)))
  {
    return false;
  }
  return true;
}

bool
uv_msgs__srv__GetState_Event__copy(
  const uv_msgs__srv__GetState_Event * input,
  uv_msgs__srv__GetState_Event * output)
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
  if (!uv_msgs__srv__GetState_Request__Sequence__copy(
      &(input->request), &(output->request)))
  {
    return false;
  }
  // response
  if (!uv_msgs__srv__GetState_Response__Sequence__copy(
      &(input->response), &(output->response)))
  {
    return false;
  }
  return true;
}

uv_msgs__srv__GetState_Event *
uv_msgs__srv__GetState_Event__create(void)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  uv_msgs__srv__GetState_Event * msg = (uv_msgs__srv__GetState_Event *)allocator.allocate(sizeof(uv_msgs__srv__GetState_Event), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(uv_msgs__srv__GetState_Event));
  bool success = uv_msgs__srv__GetState_Event__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
uv_msgs__srv__GetState_Event__destroy(uv_msgs__srv__GetState_Event * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    uv_msgs__srv__GetState_Event__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
uv_msgs__srv__GetState_Event__Sequence__init(uv_msgs__srv__GetState_Event__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  uv_msgs__srv__GetState_Event * data = NULL;

  if (size) {
    data = (uv_msgs__srv__GetState_Event *)allocator.zero_allocate(size, sizeof(uv_msgs__srv__GetState_Event), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = uv_msgs__srv__GetState_Event__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        uv_msgs__srv__GetState_Event__fini(&data[i - 1]);
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
uv_msgs__srv__GetState_Event__Sequence__fini(uv_msgs__srv__GetState_Event__Sequence * array)
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
      uv_msgs__srv__GetState_Event__fini(&array->data[i]);
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

uv_msgs__srv__GetState_Event__Sequence *
uv_msgs__srv__GetState_Event__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  uv_msgs__srv__GetState_Event__Sequence * array = (uv_msgs__srv__GetState_Event__Sequence *)allocator.allocate(sizeof(uv_msgs__srv__GetState_Event__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = uv_msgs__srv__GetState_Event__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
uv_msgs__srv__GetState_Event__Sequence__destroy(uv_msgs__srv__GetState_Event__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    uv_msgs__srv__GetState_Event__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
uv_msgs__srv__GetState_Event__Sequence__are_equal(const uv_msgs__srv__GetState_Event__Sequence * lhs, const uv_msgs__srv__GetState_Event__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!uv_msgs__srv__GetState_Event__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
uv_msgs__srv__GetState_Event__Sequence__copy(
  const uv_msgs__srv__GetState_Event__Sequence * input,
  uv_msgs__srv__GetState_Event__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(uv_msgs__srv__GetState_Event);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    uv_msgs__srv__GetState_Event * data =
      (uv_msgs__srv__GetState_Event *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!uv_msgs__srv__GetState_Event__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          uv_msgs__srv__GetState_Event__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!uv_msgs__srv__GetState_Event__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
