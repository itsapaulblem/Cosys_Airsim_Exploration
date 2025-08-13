// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from airsim_interfaces:msg/InstanceSegmentationList.idl
// generated code does not contain a copyright notice
#include "airsim_interfaces/msg/detail/instance_segmentation_list__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


// Include directives for member types
// Member `header`
#include "std_msgs/msg/detail/header__functions.h"
// Member `labels`
#include "airsim_interfaces/msg/detail/instance_segmentation_label__functions.h"

bool
airsim_interfaces__msg__InstanceSegmentationList__init(airsim_interfaces__msg__InstanceSegmentationList * msg)
{
  if (!msg) {
    return false;
  }
  // header
  if (!std_msgs__msg__Header__init(&msg->header)) {
    airsim_interfaces__msg__InstanceSegmentationList__fini(msg);
    return false;
  }
  // labels
  if (!airsim_interfaces__msg__InstanceSegmentationLabel__Sequence__init(&msg->labels, 0)) {
    airsim_interfaces__msg__InstanceSegmentationList__fini(msg);
    return false;
  }
  return true;
}

void
airsim_interfaces__msg__InstanceSegmentationList__fini(airsim_interfaces__msg__InstanceSegmentationList * msg)
{
  if (!msg) {
    return;
  }
  // header
  std_msgs__msg__Header__fini(&msg->header);
  // labels
  airsim_interfaces__msg__InstanceSegmentationLabel__Sequence__fini(&msg->labels);
}

bool
airsim_interfaces__msg__InstanceSegmentationList__are_equal(const airsim_interfaces__msg__InstanceSegmentationList * lhs, const airsim_interfaces__msg__InstanceSegmentationList * rhs)
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
  // labels
  if (!airsim_interfaces__msg__InstanceSegmentationLabel__Sequence__are_equal(
      &(lhs->labels), &(rhs->labels)))
  {
    return false;
  }
  return true;
}

bool
airsim_interfaces__msg__InstanceSegmentationList__copy(
  const airsim_interfaces__msg__InstanceSegmentationList * input,
  airsim_interfaces__msg__InstanceSegmentationList * output)
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
  // labels
  if (!airsim_interfaces__msg__InstanceSegmentationLabel__Sequence__copy(
      &(input->labels), &(output->labels)))
  {
    return false;
  }
  return true;
}

airsim_interfaces__msg__InstanceSegmentationList *
airsim_interfaces__msg__InstanceSegmentationList__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  airsim_interfaces__msg__InstanceSegmentationList * msg = (airsim_interfaces__msg__InstanceSegmentationList *)allocator.allocate(sizeof(airsim_interfaces__msg__InstanceSegmentationList), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(airsim_interfaces__msg__InstanceSegmentationList));
  bool success = airsim_interfaces__msg__InstanceSegmentationList__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
airsim_interfaces__msg__InstanceSegmentationList__destroy(airsim_interfaces__msg__InstanceSegmentationList * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    airsim_interfaces__msg__InstanceSegmentationList__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
airsim_interfaces__msg__InstanceSegmentationList__Sequence__init(airsim_interfaces__msg__InstanceSegmentationList__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  airsim_interfaces__msg__InstanceSegmentationList * data = NULL;

  if (size) {
    data = (airsim_interfaces__msg__InstanceSegmentationList *)allocator.zero_allocate(size, sizeof(airsim_interfaces__msg__InstanceSegmentationList), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = airsim_interfaces__msg__InstanceSegmentationList__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        airsim_interfaces__msg__InstanceSegmentationList__fini(&data[i - 1]);
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
airsim_interfaces__msg__InstanceSegmentationList__Sequence__fini(airsim_interfaces__msg__InstanceSegmentationList__Sequence * array)
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
      airsim_interfaces__msg__InstanceSegmentationList__fini(&array->data[i]);
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

airsim_interfaces__msg__InstanceSegmentationList__Sequence *
airsim_interfaces__msg__InstanceSegmentationList__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  airsim_interfaces__msg__InstanceSegmentationList__Sequence * array = (airsim_interfaces__msg__InstanceSegmentationList__Sequence *)allocator.allocate(sizeof(airsim_interfaces__msg__InstanceSegmentationList__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = airsim_interfaces__msg__InstanceSegmentationList__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
airsim_interfaces__msg__InstanceSegmentationList__Sequence__destroy(airsim_interfaces__msg__InstanceSegmentationList__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    airsim_interfaces__msg__InstanceSegmentationList__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
airsim_interfaces__msg__InstanceSegmentationList__Sequence__are_equal(const airsim_interfaces__msg__InstanceSegmentationList__Sequence * lhs, const airsim_interfaces__msg__InstanceSegmentationList__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!airsim_interfaces__msg__InstanceSegmentationList__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
airsim_interfaces__msg__InstanceSegmentationList__Sequence__copy(
  const airsim_interfaces__msg__InstanceSegmentationList__Sequence * input,
  airsim_interfaces__msg__InstanceSegmentationList__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(airsim_interfaces__msg__InstanceSegmentationList);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    airsim_interfaces__msg__InstanceSegmentationList * data =
      (airsim_interfaces__msg__InstanceSegmentationList *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!airsim_interfaces__msg__InstanceSegmentationList__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          airsim_interfaces__msg__InstanceSegmentationList__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!airsim_interfaces__msg__InstanceSegmentationList__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
