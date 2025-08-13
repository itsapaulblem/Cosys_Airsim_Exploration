// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from airsim_interfaces:msg/InstanceSegmentationLabel.idl
// generated code does not contain a copyright notice
#include "airsim_interfaces/msg/detail/instance_segmentation_label__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


// Include directives for member types
// Member `name`
#include "rosidl_runtime_c/string_functions.h"

bool
airsim_interfaces__msg__InstanceSegmentationLabel__init(airsim_interfaces__msg__InstanceSegmentationLabel * msg)
{
  if (!msg) {
    return false;
  }
  // name
  if (!rosidl_runtime_c__String__init(&msg->name)) {
    airsim_interfaces__msg__InstanceSegmentationLabel__fini(msg);
    return false;
  }
  // index
  // r
  // g
  // b
  return true;
}

void
airsim_interfaces__msg__InstanceSegmentationLabel__fini(airsim_interfaces__msg__InstanceSegmentationLabel * msg)
{
  if (!msg) {
    return;
  }
  // name
  rosidl_runtime_c__String__fini(&msg->name);
  // index
  // r
  // g
  // b
}

bool
airsim_interfaces__msg__InstanceSegmentationLabel__are_equal(const airsim_interfaces__msg__InstanceSegmentationLabel * lhs, const airsim_interfaces__msg__InstanceSegmentationLabel * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // name
  if (!rosidl_runtime_c__String__are_equal(
      &(lhs->name), &(rhs->name)))
  {
    return false;
  }
  // index
  if (lhs->index != rhs->index) {
    return false;
  }
  // r
  if (lhs->r != rhs->r) {
    return false;
  }
  // g
  if (lhs->g != rhs->g) {
    return false;
  }
  // b
  if (lhs->b != rhs->b) {
    return false;
  }
  return true;
}

bool
airsim_interfaces__msg__InstanceSegmentationLabel__copy(
  const airsim_interfaces__msg__InstanceSegmentationLabel * input,
  airsim_interfaces__msg__InstanceSegmentationLabel * output)
{
  if (!input || !output) {
    return false;
  }
  // name
  if (!rosidl_runtime_c__String__copy(
      &(input->name), &(output->name)))
  {
    return false;
  }
  // index
  output->index = input->index;
  // r
  output->r = input->r;
  // g
  output->g = input->g;
  // b
  output->b = input->b;
  return true;
}

airsim_interfaces__msg__InstanceSegmentationLabel *
airsim_interfaces__msg__InstanceSegmentationLabel__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  airsim_interfaces__msg__InstanceSegmentationLabel * msg = (airsim_interfaces__msg__InstanceSegmentationLabel *)allocator.allocate(sizeof(airsim_interfaces__msg__InstanceSegmentationLabel), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(airsim_interfaces__msg__InstanceSegmentationLabel));
  bool success = airsim_interfaces__msg__InstanceSegmentationLabel__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
airsim_interfaces__msg__InstanceSegmentationLabel__destroy(airsim_interfaces__msg__InstanceSegmentationLabel * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    airsim_interfaces__msg__InstanceSegmentationLabel__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
airsim_interfaces__msg__InstanceSegmentationLabel__Sequence__init(airsim_interfaces__msg__InstanceSegmentationLabel__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  airsim_interfaces__msg__InstanceSegmentationLabel * data = NULL;

  if (size) {
    data = (airsim_interfaces__msg__InstanceSegmentationLabel *)allocator.zero_allocate(size, sizeof(airsim_interfaces__msg__InstanceSegmentationLabel), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = airsim_interfaces__msg__InstanceSegmentationLabel__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        airsim_interfaces__msg__InstanceSegmentationLabel__fini(&data[i - 1]);
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
airsim_interfaces__msg__InstanceSegmentationLabel__Sequence__fini(airsim_interfaces__msg__InstanceSegmentationLabel__Sequence * array)
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
      airsim_interfaces__msg__InstanceSegmentationLabel__fini(&array->data[i]);
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

airsim_interfaces__msg__InstanceSegmentationLabel__Sequence *
airsim_interfaces__msg__InstanceSegmentationLabel__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  airsim_interfaces__msg__InstanceSegmentationLabel__Sequence * array = (airsim_interfaces__msg__InstanceSegmentationLabel__Sequence *)allocator.allocate(sizeof(airsim_interfaces__msg__InstanceSegmentationLabel__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = airsim_interfaces__msg__InstanceSegmentationLabel__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
airsim_interfaces__msg__InstanceSegmentationLabel__Sequence__destroy(airsim_interfaces__msg__InstanceSegmentationLabel__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    airsim_interfaces__msg__InstanceSegmentationLabel__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
airsim_interfaces__msg__InstanceSegmentationLabel__Sequence__are_equal(const airsim_interfaces__msg__InstanceSegmentationLabel__Sequence * lhs, const airsim_interfaces__msg__InstanceSegmentationLabel__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!airsim_interfaces__msg__InstanceSegmentationLabel__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
airsim_interfaces__msg__InstanceSegmentationLabel__Sequence__copy(
  const airsim_interfaces__msg__InstanceSegmentationLabel__Sequence * input,
  airsim_interfaces__msg__InstanceSegmentationLabel__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(airsim_interfaces__msg__InstanceSegmentationLabel);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    airsim_interfaces__msg__InstanceSegmentationLabel * data =
      (airsim_interfaces__msg__InstanceSegmentationLabel *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!airsim_interfaces__msg__InstanceSegmentationLabel__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          airsim_interfaces__msg__InstanceSegmentationLabel__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!airsim_interfaces__msg__InstanceSegmentationLabel__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
