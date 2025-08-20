// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from airsim_interfaces:msg/VelCmdGroup.idl
// generated code does not contain a copyright notice
#include "airsim_interfaces/msg/detail/vel_cmd_group__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


// Include directives for member types
// Member `vel_cmd`
#include "airsim_interfaces/msg/detail/vel_cmd__functions.h"
// Member `vehicle_names`
#include "rosidl_runtime_c/string_functions.h"

bool
airsim_interfaces__msg__VelCmdGroup__init(airsim_interfaces__msg__VelCmdGroup * msg)
{
  if (!msg) {
    return false;
  }
  // vel_cmd
  if (!airsim_interfaces__msg__VelCmd__init(&msg->vel_cmd)) {
    airsim_interfaces__msg__VelCmdGroup__fini(msg);
    return false;
  }
  // vehicle_names
  if (!rosidl_runtime_c__String__Sequence__init(&msg->vehicle_names, 0)) {
    airsim_interfaces__msg__VelCmdGroup__fini(msg);
    return false;
  }
  return true;
}

void
airsim_interfaces__msg__VelCmdGroup__fini(airsim_interfaces__msg__VelCmdGroup * msg)
{
  if (!msg) {
    return;
  }
  // vel_cmd
  airsim_interfaces__msg__VelCmd__fini(&msg->vel_cmd);
  // vehicle_names
  rosidl_runtime_c__String__Sequence__fini(&msg->vehicle_names);
}

bool
airsim_interfaces__msg__VelCmdGroup__are_equal(const airsim_interfaces__msg__VelCmdGroup * lhs, const airsim_interfaces__msg__VelCmdGroup * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // vel_cmd
  if (!airsim_interfaces__msg__VelCmd__are_equal(
      &(lhs->vel_cmd), &(rhs->vel_cmd)))
  {
    return false;
  }
  // vehicle_names
  if (!rosidl_runtime_c__String__Sequence__are_equal(
      &(lhs->vehicle_names), &(rhs->vehicle_names)))
  {
    return false;
  }
  return true;
}

bool
airsim_interfaces__msg__VelCmdGroup__copy(
  const airsim_interfaces__msg__VelCmdGroup * input,
  airsim_interfaces__msg__VelCmdGroup * output)
{
  if (!input || !output) {
    return false;
  }
  // vel_cmd
  if (!airsim_interfaces__msg__VelCmd__copy(
      &(input->vel_cmd), &(output->vel_cmd)))
  {
    return false;
  }
  // vehicle_names
  if (!rosidl_runtime_c__String__Sequence__copy(
      &(input->vehicle_names), &(output->vehicle_names)))
  {
    return false;
  }
  return true;
}

airsim_interfaces__msg__VelCmdGroup *
airsim_interfaces__msg__VelCmdGroup__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  airsim_interfaces__msg__VelCmdGroup * msg = (airsim_interfaces__msg__VelCmdGroup *)allocator.allocate(sizeof(airsim_interfaces__msg__VelCmdGroup), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(airsim_interfaces__msg__VelCmdGroup));
  bool success = airsim_interfaces__msg__VelCmdGroup__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
airsim_interfaces__msg__VelCmdGroup__destroy(airsim_interfaces__msg__VelCmdGroup * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    airsim_interfaces__msg__VelCmdGroup__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
airsim_interfaces__msg__VelCmdGroup__Sequence__init(airsim_interfaces__msg__VelCmdGroup__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  airsim_interfaces__msg__VelCmdGroup * data = NULL;

  if (size) {
    data = (airsim_interfaces__msg__VelCmdGroup *)allocator.zero_allocate(size, sizeof(airsim_interfaces__msg__VelCmdGroup), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = airsim_interfaces__msg__VelCmdGroup__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        airsim_interfaces__msg__VelCmdGroup__fini(&data[i - 1]);
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
airsim_interfaces__msg__VelCmdGroup__Sequence__fini(airsim_interfaces__msg__VelCmdGroup__Sequence * array)
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
      airsim_interfaces__msg__VelCmdGroup__fini(&array->data[i]);
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

airsim_interfaces__msg__VelCmdGroup__Sequence *
airsim_interfaces__msg__VelCmdGroup__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  airsim_interfaces__msg__VelCmdGroup__Sequence * array = (airsim_interfaces__msg__VelCmdGroup__Sequence *)allocator.allocate(sizeof(airsim_interfaces__msg__VelCmdGroup__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = airsim_interfaces__msg__VelCmdGroup__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
airsim_interfaces__msg__VelCmdGroup__Sequence__destroy(airsim_interfaces__msg__VelCmdGroup__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    airsim_interfaces__msg__VelCmdGroup__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
airsim_interfaces__msg__VelCmdGroup__Sequence__are_equal(const airsim_interfaces__msg__VelCmdGroup__Sequence * lhs, const airsim_interfaces__msg__VelCmdGroup__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!airsim_interfaces__msg__VelCmdGroup__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
airsim_interfaces__msg__VelCmdGroup__Sequence__copy(
  const airsim_interfaces__msg__VelCmdGroup__Sequence * input,
  airsim_interfaces__msg__VelCmdGroup__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(airsim_interfaces__msg__VelCmdGroup);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    airsim_interfaces__msg__VelCmdGroup * data =
      (airsim_interfaces__msg__VelCmdGroup *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!airsim_interfaces__msg__VelCmdGroup__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          airsim_interfaces__msg__VelCmdGroup__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!airsim_interfaces__msg__VelCmdGroup__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
