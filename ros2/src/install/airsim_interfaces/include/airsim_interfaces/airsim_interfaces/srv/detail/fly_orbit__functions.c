// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from airsim_interfaces:srv/FlyOrbit.idl
// generated code does not contain a copyright notice
#include "airsim_interfaces/srv/detail/fly_orbit__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"

// Include directives for member types
// Member `center`
#include "geometry_msgs/msg/detail/point__functions.h"

bool
airsim_interfaces__srv__FlyOrbit_Request__init(airsim_interfaces__srv__FlyOrbit_Request * msg)
{
  if (!msg) {
    return false;
  }
  // center
  if (!geometry_msgs__msg__Point__init(&msg->center)) {
    airsim_interfaces__srv__FlyOrbit_Request__fini(msg);
    return false;
  }
  // radius
  // velocity
  // yaw_mode
  // duration
  return true;
}

void
airsim_interfaces__srv__FlyOrbit_Request__fini(airsim_interfaces__srv__FlyOrbit_Request * msg)
{
  if (!msg) {
    return;
  }
  // center
  geometry_msgs__msg__Point__fini(&msg->center);
  // radius
  // velocity
  // yaw_mode
  // duration
}

bool
airsim_interfaces__srv__FlyOrbit_Request__are_equal(const airsim_interfaces__srv__FlyOrbit_Request * lhs, const airsim_interfaces__srv__FlyOrbit_Request * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // center
  if (!geometry_msgs__msg__Point__are_equal(
      &(lhs->center), &(rhs->center)))
  {
    return false;
  }
  // radius
  if (lhs->radius != rhs->radius) {
    return false;
  }
  // velocity
  if (lhs->velocity != rhs->velocity) {
    return false;
  }
  // yaw_mode
  if (lhs->yaw_mode != rhs->yaw_mode) {
    return false;
  }
  // duration
  if (lhs->duration != rhs->duration) {
    return false;
  }
  return true;
}

bool
airsim_interfaces__srv__FlyOrbit_Request__copy(
  const airsim_interfaces__srv__FlyOrbit_Request * input,
  airsim_interfaces__srv__FlyOrbit_Request * output)
{
  if (!input || !output) {
    return false;
  }
  // center
  if (!geometry_msgs__msg__Point__copy(
      &(input->center), &(output->center)))
  {
    return false;
  }
  // radius
  output->radius = input->radius;
  // velocity
  output->velocity = input->velocity;
  // yaw_mode
  output->yaw_mode = input->yaw_mode;
  // duration
  output->duration = input->duration;
  return true;
}

airsim_interfaces__srv__FlyOrbit_Request *
airsim_interfaces__srv__FlyOrbit_Request__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  airsim_interfaces__srv__FlyOrbit_Request * msg = (airsim_interfaces__srv__FlyOrbit_Request *)allocator.allocate(sizeof(airsim_interfaces__srv__FlyOrbit_Request), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(airsim_interfaces__srv__FlyOrbit_Request));
  bool success = airsim_interfaces__srv__FlyOrbit_Request__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
airsim_interfaces__srv__FlyOrbit_Request__destroy(airsim_interfaces__srv__FlyOrbit_Request * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    airsim_interfaces__srv__FlyOrbit_Request__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
airsim_interfaces__srv__FlyOrbit_Request__Sequence__init(airsim_interfaces__srv__FlyOrbit_Request__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  airsim_interfaces__srv__FlyOrbit_Request * data = NULL;

  if (size) {
    data = (airsim_interfaces__srv__FlyOrbit_Request *)allocator.zero_allocate(size, sizeof(airsim_interfaces__srv__FlyOrbit_Request), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = airsim_interfaces__srv__FlyOrbit_Request__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        airsim_interfaces__srv__FlyOrbit_Request__fini(&data[i - 1]);
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
airsim_interfaces__srv__FlyOrbit_Request__Sequence__fini(airsim_interfaces__srv__FlyOrbit_Request__Sequence * array)
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
      airsim_interfaces__srv__FlyOrbit_Request__fini(&array->data[i]);
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

airsim_interfaces__srv__FlyOrbit_Request__Sequence *
airsim_interfaces__srv__FlyOrbit_Request__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  airsim_interfaces__srv__FlyOrbit_Request__Sequence * array = (airsim_interfaces__srv__FlyOrbit_Request__Sequence *)allocator.allocate(sizeof(airsim_interfaces__srv__FlyOrbit_Request__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = airsim_interfaces__srv__FlyOrbit_Request__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
airsim_interfaces__srv__FlyOrbit_Request__Sequence__destroy(airsim_interfaces__srv__FlyOrbit_Request__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    airsim_interfaces__srv__FlyOrbit_Request__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
airsim_interfaces__srv__FlyOrbit_Request__Sequence__are_equal(const airsim_interfaces__srv__FlyOrbit_Request__Sequence * lhs, const airsim_interfaces__srv__FlyOrbit_Request__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!airsim_interfaces__srv__FlyOrbit_Request__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
airsim_interfaces__srv__FlyOrbit_Request__Sequence__copy(
  const airsim_interfaces__srv__FlyOrbit_Request__Sequence * input,
  airsim_interfaces__srv__FlyOrbit_Request__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(airsim_interfaces__srv__FlyOrbit_Request);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    airsim_interfaces__srv__FlyOrbit_Request * data =
      (airsim_interfaces__srv__FlyOrbit_Request *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!airsim_interfaces__srv__FlyOrbit_Request__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          airsim_interfaces__srv__FlyOrbit_Request__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!airsim_interfaces__srv__FlyOrbit_Request__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}


// Include directives for member types
// Member `message`
#include "rosidl_runtime_c/string_functions.h"

bool
airsim_interfaces__srv__FlyOrbit_Response__init(airsim_interfaces__srv__FlyOrbit_Response * msg)
{
  if (!msg) {
    return false;
  }
  // success
  // message
  if (!rosidl_runtime_c__String__init(&msg->message)) {
    airsim_interfaces__srv__FlyOrbit_Response__fini(msg);
    return false;
  }
  return true;
}

void
airsim_interfaces__srv__FlyOrbit_Response__fini(airsim_interfaces__srv__FlyOrbit_Response * msg)
{
  if (!msg) {
    return;
  }
  // success
  // message
  rosidl_runtime_c__String__fini(&msg->message);
}

bool
airsim_interfaces__srv__FlyOrbit_Response__are_equal(const airsim_interfaces__srv__FlyOrbit_Response * lhs, const airsim_interfaces__srv__FlyOrbit_Response * rhs)
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
airsim_interfaces__srv__FlyOrbit_Response__copy(
  const airsim_interfaces__srv__FlyOrbit_Response * input,
  airsim_interfaces__srv__FlyOrbit_Response * output)
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

airsim_interfaces__srv__FlyOrbit_Response *
airsim_interfaces__srv__FlyOrbit_Response__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  airsim_interfaces__srv__FlyOrbit_Response * msg = (airsim_interfaces__srv__FlyOrbit_Response *)allocator.allocate(sizeof(airsim_interfaces__srv__FlyOrbit_Response), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(airsim_interfaces__srv__FlyOrbit_Response));
  bool success = airsim_interfaces__srv__FlyOrbit_Response__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
airsim_interfaces__srv__FlyOrbit_Response__destroy(airsim_interfaces__srv__FlyOrbit_Response * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    airsim_interfaces__srv__FlyOrbit_Response__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
airsim_interfaces__srv__FlyOrbit_Response__Sequence__init(airsim_interfaces__srv__FlyOrbit_Response__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  airsim_interfaces__srv__FlyOrbit_Response * data = NULL;

  if (size) {
    data = (airsim_interfaces__srv__FlyOrbit_Response *)allocator.zero_allocate(size, sizeof(airsim_interfaces__srv__FlyOrbit_Response), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = airsim_interfaces__srv__FlyOrbit_Response__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        airsim_interfaces__srv__FlyOrbit_Response__fini(&data[i - 1]);
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
airsim_interfaces__srv__FlyOrbit_Response__Sequence__fini(airsim_interfaces__srv__FlyOrbit_Response__Sequence * array)
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
      airsim_interfaces__srv__FlyOrbit_Response__fini(&array->data[i]);
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

airsim_interfaces__srv__FlyOrbit_Response__Sequence *
airsim_interfaces__srv__FlyOrbit_Response__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  airsim_interfaces__srv__FlyOrbit_Response__Sequence * array = (airsim_interfaces__srv__FlyOrbit_Response__Sequence *)allocator.allocate(sizeof(airsim_interfaces__srv__FlyOrbit_Response__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = airsim_interfaces__srv__FlyOrbit_Response__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
airsim_interfaces__srv__FlyOrbit_Response__Sequence__destroy(airsim_interfaces__srv__FlyOrbit_Response__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    airsim_interfaces__srv__FlyOrbit_Response__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
airsim_interfaces__srv__FlyOrbit_Response__Sequence__are_equal(const airsim_interfaces__srv__FlyOrbit_Response__Sequence * lhs, const airsim_interfaces__srv__FlyOrbit_Response__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!airsim_interfaces__srv__FlyOrbit_Response__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
airsim_interfaces__srv__FlyOrbit_Response__Sequence__copy(
  const airsim_interfaces__srv__FlyOrbit_Response__Sequence * input,
  airsim_interfaces__srv__FlyOrbit_Response__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(airsim_interfaces__srv__FlyOrbit_Response);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    airsim_interfaces__srv__FlyOrbit_Response * data =
      (airsim_interfaces__srv__FlyOrbit_Response *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!airsim_interfaces__srv__FlyOrbit_Response__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          airsim_interfaces__srv__FlyOrbit_Response__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!airsim_interfaces__srv__FlyOrbit_Response__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
