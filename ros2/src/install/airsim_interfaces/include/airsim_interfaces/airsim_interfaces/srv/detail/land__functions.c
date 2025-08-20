// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from airsim_interfaces:srv/Land.idl
// generated code does not contain a copyright notice
#include "airsim_interfaces/srv/detail/land__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"

bool
airsim_interfaces__srv__Land_Request__init(airsim_interfaces__srv__Land_Request * msg)
{
  if (!msg) {
    return false;
  }
  // wait_on_last_task
  return true;
}

void
airsim_interfaces__srv__Land_Request__fini(airsim_interfaces__srv__Land_Request * msg)
{
  if (!msg) {
    return;
  }
  // wait_on_last_task
}

bool
airsim_interfaces__srv__Land_Request__are_equal(const airsim_interfaces__srv__Land_Request * lhs, const airsim_interfaces__srv__Land_Request * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // wait_on_last_task
  if (lhs->wait_on_last_task != rhs->wait_on_last_task) {
    return false;
  }
  return true;
}

bool
airsim_interfaces__srv__Land_Request__copy(
  const airsim_interfaces__srv__Land_Request * input,
  airsim_interfaces__srv__Land_Request * output)
{
  if (!input || !output) {
    return false;
  }
  // wait_on_last_task
  output->wait_on_last_task = input->wait_on_last_task;
  return true;
}

airsim_interfaces__srv__Land_Request *
airsim_interfaces__srv__Land_Request__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  airsim_interfaces__srv__Land_Request * msg = (airsim_interfaces__srv__Land_Request *)allocator.allocate(sizeof(airsim_interfaces__srv__Land_Request), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(airsim_interfaces__srv__Land_Request));
  bool success = airsim_interfaces__srv__Land_Request__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
airsim_interfaces__srv__Land_Request__destroy(airsim_interfaces__srv__Land_Request * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    airsim_interfaces__srv__Land_Request__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
airsim_interfaces__srv__Land_Request__Sequence__init(airsim_interfaces__srv__Land_Request__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  airsim_interfaces__srv__Land_Request * data = NULL;

  if (size) {
    data = (airsim_interfaces__srv__Land_Request *)allocator.zero_allocate(size, sizeof(airsim_interfaces__srv__Land_Request), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = airsim_interfaces__srv__Land_Request__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        airsim_interfaces__srv__Land_Request__fini(&data[i - 1]);
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
airsim_interfaces__srv__Land_Request__Sequence__fini(airsim_interfaces__srv__Land_Request__Sequence * array)
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
      airsim_interfaces__srv__Land_Request__fini(&array->data[i]);
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

airsim_interfaces__srv__Land_Request__Sequence *
airsim_interfaces__srv__Land_Request__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  airsim_interfaces__srv__Land_Request__Sequence * array = (airsim_interfaces__srv__Land_Request__Sequence *)allocator.allocate(sizeof(airsim_interfaces__srv__Land_Request__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = airsim_interfaces__srv__Land_Request__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
airsim_interfaces__srv__Land_Request__Sequence__destroy(airsim_interfaces__srv__Land_Request__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    airsim_interfaces__srv__Land_Request__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
airsim_interfaces__srv__Land_Request__Sequence__are_equal(const airsim_interfaces__srv__Land_Request__Sequence * lhs, const airsim_interfaces__srv__Land_Request__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!airsim_interfaces__srv__Land_Request__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
airsim_interfaces__srv__Land_Request__Sequence__copy(
  const airsim_interfaces__srv__Land_Request__Sequence * input,
  airsim_interfaces__srv__Land_Request__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(airsim_interfaces__srv__Land_Request);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    airsim_interfaces__srv__Land_Request * data =
      (airsim_interfaces__srv__Land_Request *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!airsim_interfaces__srv__Land_Request__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          airsim_interfaces__srv__Land_Request__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!airsim_interfaces__srv__Land_Request__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}


bool
airsim_interfaces__srv__Land_Response__init(airsim_interfaces__srv__Land_Response * msg)
{
  if (!msg) {
    return false;
  }
  // success
  return true;
}

void
airsim_interfaces__srv__Land_Response__fini(airsim_interfaces__srv__Land_Response * msg)
{
  if (!msg) {
    return;
  }
  // success
}

bool
airsim_interfaces__srv__Land_Response__are_equal(const airsim_interfaces__srv__Land_Response * lhs, const airsim_interfaces__srv__Land_Response * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // success
  if (lhs->success != rhs->success) {
    return false;
  }
  return true;
}

bool
airsim_interfaces__srv__Land_Response__copy(
  const airsim_interfaces__srv__Land_Response * input,
  airsim_interfaces__srv__Land_Response * output)
{
  if (!input || !output) {
    return false;
  }
  // success
  output->success = input->success;
  return true;
}

airsim_interfaces__srv__Land_Response *
airsim_interfaces__srv__Land_Response__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  airsim_interfaces__srv__Land_Response * msg = (airsim_interfaces__srv__Land_Response *)allocator.allocate(sizeof(airsim_interfaces__srv__Land_Response), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(airsim_interfaces__srv__Land_Response));
  bool success = airsim_interfaces__srv__Land_Response__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
airsim_interfaces__srv__Land_Response__destroy(airsim_interfaces__srv__Land_Response * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    airsim_interfaces__srv__Land_Response__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
airsim_interfaces__srv__Land_Response__Sequence__init(airsim_interfaces__srv__Land_Response__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  airsim_interfaces__srv__Land_Response * data = NULL;

  if (size) {
    data = (airsim_interfaces__srv__Land_Response *)allocator.zero_allocate(size, sizeof(airsim_interfaces__srv__Land_Response), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = airsim_interfaces__srv__Land_Response__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        airsim_interfaces__srv__Land_Response__fini(&data[i - 1]);
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
airsim_interfaces__srv__Land_Response__Sequence__fini(airsim_interfaces__srv__Land_Response__Sequence * array)
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
      airsim_interfaces__srv__Land_Response__fini(&array->data[i]);
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

airsim_interfaces__srv__Land_Response__Sequence *
airsim_interfaces__srv__Land_Response__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  airsim_interfaces__srv__Land_Response__Sequence * array = (airsim_interfaces__srv__Land_Response__Sequence *)allocator.allocate(sizeof(airsim_interfaces__srv__Land_Response__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = airsim_interfaces__srv__Land_Response__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
airsim_interfaces__srv__Land_Response__Sequence__destroy(airsim_interfaces__srv__Land_Response__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    airsim_interfaces__srv__Land_Response__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
airsim_interfaces__srv__Land_Response__Sequence__are_equal(const airsim_interfaces__srv__Land_Response__Sequence * lhs, const airsim_interfaces__srv__Land_Response__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!airsim_interfaces__srv__Land_Response__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
airsim_interfaces__srv__Land_Response__Sequence__copy(
  const airsim_interfaces__srv__Land_Response__Sequence * input,
  airsim_interfaces__srv__Land_Response__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(airsim_interfaces__srv__Land_Response);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    airsim_interfaces__srv__Land_Response * data =
      (airsim_interfaces__srv__Land_Response *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!airsim_interfaces__srv__Land_Response__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          airsim_interfaces__srv__Land_Response__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!airsim_interfaces__srv__Land_Response__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
