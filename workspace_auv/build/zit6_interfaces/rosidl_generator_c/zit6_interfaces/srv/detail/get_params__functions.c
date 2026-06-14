// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from zit6_interfaces:srv/GetParams.idl
// generated code does not contain a copyright notice
#include "zit6_interfaces/srv/detail/get_params__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"

// Include directives for member types
// Member `paths`
#include "rosidl_runtime_c/string_functions.h"

bool
zit6_interfaces__srv__GetParams_Request__init(zit6_interfaces__srv__GetParams_Request * msg)
{
  if (!msg) {
    return false;
  }
  // paths
  if (!rosidl_runtime_c__String__Sequence__init(&msg->paths, 0)) {
    zit6_interfaces__srv__GetParams_Request__fini(msg);
    return false;
  }
  return true;
}

void
zit6_interfaces__srv__GetParams_Request__fini(zit6_interfaces__srv__GetParams_Request * msg)
{
  if (!msg) {
    return;
  }
  // paths
  rosidl_runtime_c__String__Sequence__fini(&msg->paths);
}

bool
zit6_interfaces__srv__GetParams_Request__are_equal(const zit6_interfaces__srv__GetParams_Request * lhs, const zit6_interfaces__srv__GetParams_Request * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // paths
  if (!rosidl_runtime_c__String__Sequence__are_equal(
      &(lhs->paths), &(rhs->paths)))
  {
    return false;
  }
  return true;
}

bool
zit6_interfaces__srv__GetParams_Request__copy(
  const zit6_interfaces__srv__GetParams_Request * input,
  zit6_interfaces__srv__GetParams_Request * output)
{
  if (!input || !output) {
    return false;
  }
  // paths
  if (!rosidl_runtime_c__String__Sequence__copy(
      &(input->paths), &(output->paths)))
  {
    return false;
  }
  return true;
}

zit6_interfaces__srv__GetParams_Request *
zit6_interfaces__srv__GetParams_Request__create(void)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  zit6_interfaces__srv__GetParams_Request * msg = (zit6_interfaces__srv__GetParams_Request *)allocator.allocate(sizeof(zit6_interfaces__srv__GetParams_Request), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(zit6_interfaces__srv__GetParams_Request));
  bool success = zit6_interfaces__srv__GetParams_Request__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
zit6_interfaces__srv__GetParams_Request__destroy(zit6_interfaces__srv__GetParams_Request * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    zit6_interfaces__srv__GetParams_Request__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
zit6_interfaces__srv__GetParams_Request__Sequence__init(zit6_interfaces__srv__GetParams_Request__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  zit6_interfaces__srv__GetParams_Request * data = NULL;

  if (size) {
    data = (zit6_interfaces__srv__GetParams_Request *)allocator.zero_allocate(size, sizeof(zit6_interfaces__srv__GetParams_Request), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = zit6_interfaces__srv__GetParams_Request__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        zit6_interfaces__srv__GetParams_Request__fini(&data[i - 1]);
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
zit6_interfaces__srv__GetParams_Request__Sequence__fini(zit6_interfaces__srv__GetParams_Request__Sequence * array)
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
      zit6_interfaces__srv__GetParams_Request__fini(&array->data[i]);
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

zit6_interfaces__srv__GetParams_Request__Sequence *
zit6_interfaces__srv__GetParams_Request__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  zit6_interfaces__srv__GetParams_Request__Sequence * array = (zit6_interfaces__srv__GetParams_Request__Sequence *)allocator.allocate(sizeof(zit6_interfaces__srv__GetParams_Request__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = zit6_interfaces__srv__GetParams_Request__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
zit6_interfaces__srv__GetParams_Request__Sequence__destroy(zit6_interfaces__srv__GetParams_Request__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    zit6_interfaces__srv__GetParams_Request__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
zit6_interfaces__srv__GetParams_Request__Sequence__are_equal(const zit6_interfaces__srv__GetParams_Request__Sequence * lhs, const zit6_interfaces__srv__GetParams_Request__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!zit6_interfaces__srv__GetParams_Request__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
zit6_interfaces__srv__GetParams_Request__Sequence__copy(
  const zit6_interfaces__srv__GetParams_Request__Sequence * input,
  zit6_interfaces__srv__GetParams_Request__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(zit6_interfaces__srv__GetParams_Request);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    zit6_interfaces__srv__GetParams_Request * data =
      (zit6_interfaces__srv__GetParams_Request *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!zit6_interfaces__srv__GetParams_Request__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          zit6_interfaces__srv__GetParams_Request__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!zit6_interfaces__srv__GetParams_Request__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}


// Include directives for member types
// Member `message`
// Member `config_json`
// already included above
// #include "rosidl_runtime_c/string_functions.h"

bool
zit6_interfaces__srv__GetParams_Response__init(zit6_interfaces__srv__GetParams_Response * msg)
{
  if (!msg) {
    return false;
  }
  // success
  // message
  if (!rosidl_runtime_c__String__init(&msg->message)) {
    zit6_interfaces__srv__GetParams_Response__fini(msg);
    return false;
  }
  // config_json
  if (!rosidl_runtime_c__String__init(&msg->config_json)) {
    zit6_interfaces__srv__GetParams_Response__fini(msg);
    return false;
  }
  return true;
}

void
zit6_interfaces__srv__GetParams_Response__fini(zit6_interfaces__srv__GetParams_Response * msg)
{
  if (!msg) {
    return;
  }
  // success
  // message
  rosidl_runtime_c__String__fini(&msg->message);
  // config_json
  rosidl_runtime_c__String__fini(&msg->config_json);
}

bool
zit6_interfaces__srv__GetParams_Response__are_equal(const zit6_interfaces__srv__GetParams_Response * lhs, const zit6_interfaces__srv__GetParams_Response * rhs)
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
  // config_json
  if (!rosidl_runtime_c__String__are_equal(
      &(lhs->config_json), &(rhs->config_json)))
  {
    return false;
  }
  return true;
}

bool
zit6_interfaces__srv__GetParams_Response__copy(
  const zit6_interfaces__srv__GetParams_Response * input,
  zit6_interfaces__srv__GetParams_Response * output)
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
  // config_json
  if (!rosidl_runtime_c__String__copy(
      &(input->config_json), &(output->config_json)))
  {
    return false;
  }
  return true;
}

zit6_interfaces__srv__GetParams_Response *
zit6_interfaces__srv__GetParams_Response__create(void)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  zit6_interfaces__srv__GetParams_Response * msg = (zit6_interfaces__srv__GetParams_Response *)allocator.allocate(sizeof(zit6_interfaces__srv__GetParams_Response), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(zit6_interfaces__srv__GetParams_Response));
  bool success = zit6_interfaces__srv__GetParams_Response__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
zit6_interfaces__srv__GetParams_Response__destroy(zit6_interfaces__srv__GetParams_Response * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    zit6_interfaces__srv__GetParams_Response__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
zit6_interfaces__srv__GetParams_Response__Sequence__init(zit6_interfaces__srv__GetParams_Response__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  zit6_interfaces__srv__GetParams_Response * data = NULL;

  if (size) {
    data = (zit6_interfaces__srv__GetParams_Response *)allocator.zero_allocate(size, sizeof(zit6_interfaces__srv__GetParams_Response), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = zit6_interfaces__srv__GetParams_Response__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        zit6_interfaces__srv__GetParams_Response__fini(&data[i - 1]);
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
zit6_interfaces__srv__GetParams_Response__Sequence__fini(zit6_interfaces__srv__GetParams_Response__Sequence * array)
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
      zit6_interfaces__srv__GetParams_Response__fini(&array->data[i]);
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

zit6_interfaces__srv__GetParams_Response__Sequence *
zit6_interfaces__srv__GetParams_Response__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  zit6_interfaces__srv__GetParams_Response__Sequence * array = (zit6_interfaces__srv__GetParams_Response__Sequence *)allocator.allocate(sizeof(zit6_interfaces__srv__GetParams_Response__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = zit6_interfaces__srv__GetParams_Response__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
zit6_interfaces__srv__GetParams_Response__Sequence__destroy(zit6_interfaces__srv__GetParams_Response__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    zit6_interfaces__srv__GetParams_Response__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
zit6_interfaces__srv__GetParams_Response__Sequence__are_equal(const zit6_interfaces__srv__GetParams_Response__Sequence * lhs, const zit6_interfaces__srv__GetParams_Response__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!zit6_interfaces__srv__GetParams_Response__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
zit6_interfaces__srv__GetParams_Response__Sequence__copy(
  const zit6_interfaces__srv__GetParams_Response__Sequence * input,
  zit6_interfaces__srv__GetParams_Response__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(zit6_interfaces__srv__GetParams_Response);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    zit6_interfaces__srv__GetParams_Response * data =
      (zit6_interfaces__srv__GetParams_Response *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!zit6_interfaces__srv__GetParams_Response__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          zit6_interfaces__srv__GetParams_Response__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!zit6_interfaces__srv__GetParams_Response__copy(
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
// #include "zit6_interfaces/srv/detail/get_params__functions.h"

bool
zit6_interfaces__srv__GetParams_Event__init(zit6_interfaces__srv__GetParams_Event * msg)
{
  if (!msg) {
    return false;
  }
  // info
  if (!service_msgs__msg__ServiceEventInfo__init(&msg->info)) {
    zit6_interfaces__srv__GetParams_Event__fini(msg);
    return false;
  }
  // request
  if (!zit6_interfaces__srv__GetParams_Request__Sequence__init(&msg->request, 0)) {
    zit6_interfaces__srv__GetParams_Event__fini(msg);
    return false;
  }
  // response
  if (!zit6_interfaces__srv__GetParams_Response__Sequence__init(&msg->response, 0)) {
    zit6_interfaces__srv__GetParams_Event__fini(msg);
    return false;
  }
  return true;
}

void
zit6_interfaces__srv__GetParams_Event__fini(zit6_interfaces__srv__GetParams_Event * msg)
{
  if (!msg) {
    return;
  }
  // info
  service_msgs__msg__ServiceEventInfo__fini(&msg->info);
  // request
  zit6_interfaces__srv__GetParams_Request__Sequence__fini(&msg->request);
  // response
  zit6_interfaces__srv__GetParams_Response__Sequence__fini(&msg->response);
}

bool
zit6_interfaces__srv__GetParams_Event__are_equal(const zit6_interfaces__srv__GetParams_Event * lhs, const zit6_interfaces__srv__GetParams_Event * rhs)
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
  if (!zit6_interfaces__srv__GetParams_Request__Sequence__are_equal(
      &(lhs->request), &(rhs->request)))
  {
    return false;
  }
  // response
  if (!zit6_interfaces__srv__GetParams_Response__Sequence__are_equal(
      &(lhs->response), &(rhs->response)))
  {
    return false;
  }
  return true;
}

bool
zit6_interfaces__srv__GetParams_Event__copy(
  const zit6_interfaces__srv__GetParams_Event * input,
  zit6_interfaces__srv__GetParams_Event * output)
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
  if (!zit6_interfaces__srv__GetParams_Request__Sequence__copy(
      &(input->request), &(output->request)))
  {
    return false;
  }
  // response
  if (!zit6_interfaces__srv__GetParams_Response__Sequence__copy(
      &(input->response), &(output->response)))
  {
    return false;
  }
  return true;
}

zit6_interfaces__srv__GetParams_Event *
zit6_interfaces__srv__GetParams_Event__create(void)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  zit6_interfaces__srv__GetParams_Event * msg = (zit6_interfaces__srv__GetParams_Event *)allocator.allocate(sizeof(zit6_interfaces__srv__GetParams_Event), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(zit6_interfaces__srv__GetParams_Event));
  bool success = zit6_interfaces__srv__GetParams_Event__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
zit6_interfaces__srv__GetParams_Event__destroy(zit6_interfaces__srv__GetParams_Event * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    zit6_interfaces__srv__GetParams_Event__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
zit6_interfaces__srv__GetParams_Event__Sequence__init(zit6_interfaces__srv__GetParams_Event__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  zit6_interfaces__srv__GetParams_Event * data = NULL;

  if (size) {
    data = (zit6_interfaces__srv__GetParams_Event *)allocator.zero_allocate(size, sizeof(zit6_interfaces__srv__GetParams_Event), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = zit6_interfaces__srv__GetParams_Event__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        zit6_interfaces__srv__GetParams_Event__fini(&data[i - 1]);
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
zit6_interfaces__srv__GetParams_Event__Sequence__fini(zit6_interfaces__srv__GetParams_Event__Sequence * array)
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
      zit6_interfaces__srv__GetParams_Event__fini(&array->data[i]);
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

zit6_interfaces__srv__GetParams_Event__Sequence *
zit6_interfaces__srv__GetParams_Event__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  zit6_interfaces__srv__GetParams_Event__Sequence * array = (zit6_interfaces__srv__GetParams_Event__Sequence *)allocator.allocate(sizeof(zit6_interfaces__srv__GetParams_Event__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = zit6_interfaces__srv__GetParams_Event__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
zit6_interfaces__srv__GetParams_Event__Sequence__destroy(zit6_interfaces__srv__GetParams_Event__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    zit6_interfaces__srv__GetParams_Event__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
zit6_interfaces__srv__GetParams_Event__Sequence__are_equal(const zit6_interfaces__srv__GetParams_Event__Sequence * lhs, const zit6_interfaces__srv__GetParams_Event__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!zit6_interfaces__srv__GetParams_Event__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
zit6_interfaces__srv__GetParams_Event__Sequence__copy(
  const zit6_interfaces__srv__GetParams_Event__Sequence * input,
  zit6_interfaces__srv__GetParams_Event__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(zit6_interfaces__srv__GetParams_Event);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    zit6_interfaces__srv__GetParams_Event * data =
      (zit6_interfaces__srv__GetParams_Event *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!zit6_interfaces__srv__GetParams_Event__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          zit6_interfaces__srv__GetParams_Event__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!zit6_interfaces__srv__GetParams_Event__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
