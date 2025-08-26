// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from airsim_interfaces:msg/TargetDetection.idl
// generated code does not contain a copyright notice
#include "airsim_interfaces/msg/detail/target_detection__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


// Include directives for member types
// Member `header`
#include "std_msgs/msg/detail/header__functions.h"
// Member `vehicle_name`
#include "rosidl_runtime_c/string_functions.h"

bool
airsim_interfaces__msg__TargetDetection__init(airsim_interfaces__msg__TargetDetection * msg)
{
  if (!msg) {
    return false;
  }
  // header
  if (!std_msgs__msg__Header__init(&msg->header)) {
    airsim_interfaces__msg__TargetDetection__fini(msg);
    return false;
  }
  // vehicle_name
  if (!rosidl_runtime_c__String__init(&msg->vehicle_name)) {
    airsim_interfaces__msg__TargetDetection__fini(msg);
    return false;
  }
  // target_x
  // target_y
  // target_z
  // confidence
  return true;
}

void
airsim_interfaces__msg__TargetDetection__fini(airsim_interfaces__msg__TargetDetection * msg)
{
  if (!msg) {
    return;
  }
  // header
  std_msgs__msg__Header__fini(&msg->header);
  // vehicle_name
  rosidl_runtime_c__String__fini(&msg->vehicle_name);
  // target_x
  // target_y
  // target_z
  // confidence
}

bool
airsim_interfaces__msg__TargetDetection__are_equal(const airsim_interfaces__msg__TargetDetection * lhs, const airsim_interfaces__msg__TargetDetection * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // header
  if (!std_msgs__msg__Header__are_equal(
      &(lhs->header), &(rhs->header)))
  {
    return false;
  }
  // vehicle_name
  if (!rosidl_runtime_c__String__are_equal(
      &(lhs->vehicle_name), &(rhs->vehicle_name)))
  {
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
  // confidence
  if (lhs->confidence != rhs->confidence) {
    return false;
  }
  return true;
}

bool
airsim_interfaces__msg__TargetDetection__copy(
  const airsim_interfaces__msg__TargetDetection * input,
  airsim_interfaces__msg__TargetDetection * output)
{
  if (!input || !output) {
    return false;
  }
  // header
  if (!std_msgs__msg__Header__copy(
      &(input->header), &(output->header)))
  {
    return false;
  }
  // vehicle_name
  if (!rosidl_runtime_c__String__copy(
      &(input->vehicle_name), &(output->vehicle_name)))
  {
    return false;
  }
  // target_x
  output->target_x = input->target_x;
  // target_y
  output->target_y = input->target_y;
  // target_z
  output->target_z = input->target_z;
  // confidence
  output->confidence = input->confidence;
  return true;
}

airsim_interfaces__msg__TargetDetection *
airsim_interfaces__msg__TargetDetection__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  airsim_interfaces__msg__TargetDetection * msg = (airsim_interfaces__msg__TargetDetection *)allocator.allocate(sizeof(airsim_interfaces__msg__TargetDetection), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(airsim_interfaces__msg__TargetDetection));
  bool success = airsim_interfaces__msg__TargetDetection__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
airsim_interfaces__msg__TargetDetection__destroy(airsim_interfaces__msg__TargetDetection * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    airsim_interfaces__msg__TargetDetection__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
airsim_interfaces__msg__TargetDetection__Sequence__init(airsim_interfaces__msg__TargetDetection__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  airsim_interfaces__msg__TargetDetection * data = NULL;

  if (size) {
    data = (airsim_interfaces__msg__TargetDetection *)allocator.zero_allocate(size, sizeof(airsim_interfaces__msg__TargetDetection), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = airsim_interfaces__msg__TargetDetection__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        airsim_interfaces__msg__TargetDetection__fini(&data[i - 1]);
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
airsim_interfaces__msg__TargetDetection__Sequence__fini(airsim_interfaces__msg__TargetDetection__Sequence * array)
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
      airsim_interfaces__msg__TargetDetection__fini(&array->data[i]);
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

airsim_interfaces__msg__TargetDetection__Sequence *
airsim_interfaces__msg__TargetDetection__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  airsim_interfaces__msg__TargetDetection__Sequence * array = (airsim_interfaces__msg__TargetDetection__Sequence *)allocator.allocate(sizeof(airsim_interfaces__msg__TargetDetection__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = airsim_interfaces__msg__TargetDetection__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
airsim_interfaces__msg__TargetDetection__Sequence__destroy(airsim_interfaces__msg__TargetDetection__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    airsim_interfaces__msg__TargetDetection__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
airsim_interfaces__msg__TargetDetection__Sequence__are_equal(const airsim_interfaces__msg__TargetDetection__Sequence * lhs, const airsim_interfaces__msg__TargetDetection__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!airsim_interfaces__msg__TargetDetection__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
airsim_interfaces__msg__TargetDetection__Sequence__copy(
  const airsim_interfaces__msg__TargetDetection__Sequence * input,
  airsim_interfaces__msg__TargetDetection__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(airsim_interfaces__msg__TargetDetection);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    airsim_interfaces__msg__TargetDetection * data =
      (airsim_interfaces__msg__TargetDetection *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!airsim_interfaces__msg__TargetDetection__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          airsim_interfaces__msg__TargetDetection__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!airsim_interfaces__msg__TargetDetection__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
