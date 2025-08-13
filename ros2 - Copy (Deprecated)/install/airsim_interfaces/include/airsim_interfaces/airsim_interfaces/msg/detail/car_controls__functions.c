// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from airsim_interfaces:msg/CarControls.idl
// generated code does not contain a copyright notice
#include "airsim_interfaces/msg/detail/car_controls__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


// Include directives for member types
// Member `header`
#include "std_msgs/msg/detail/header__functions.h"

bool
airsim_interfaces__msg__CarControls__init(airsim_interfaces__msg__CarControls * msg)
{
  if (!msg) {
    return false;
  }
  // header
  if (!std_msgs__msg__Header__init(&msg->header)) {
    airsim_interfaces__msg__CarControls__fini(msg);
    return false;
  }
  // throttle
  // brake
  // steering
  // handbrake
  // manual
  // manual_gear
  // gear_immediate
  return true;
}

void
airsim_interfaces__msg__CarControls__fini(airsim_interfaces__msg__CarControls * msg)
{
  if (!msg) {
    return;
  }
  // header
  std_msgs__msg__Header__fini(&msg->header);
  // throttle
  // brake
  // steering
  // handbrake
  // manual
  // manual_gear
  // gear_immediate
}

bool
airsim_interfaces__msg__CarControls__are_equal(const airsim_interfaces__msg__CarControls * lhs, const airsim_interfaces__msg__CarControls * rhs)
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
  // throttle
  if (lhs->throttle != rhs->throttle) {
    return false;
  }
  // brake
  if (lhs->brake != rhs->brake) {
    return false;
  }
  // steering
  if (lhs->steering != rhs->steering) {
    return false;
  }
  // handbrake
  if (lhs->handbrake != rhs->handbrake) {
    return false;
  }
  // manual
  if (lhs->manual != rhs->manual) {
    return false;
  }
  // manual_gear
  if (lhs->manual_gear != rhs->manual_gear) {
    return false;
  }
  // gear_immediate
  if (lhs->gear_immediate != rhs->gear_immediate) {
    return false;
  }
  return true;
}

bool
airsim_interfaces__msg__CarControls__copy(
  const airsim_interfaces__msg__CarControls * input,
  airsim_interfaces__msg__CarControls * output)
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
  // throttle
  output->throttle = input->throttle;
  // brake
  output->brake = input->brake;
  // steering
  output->steering = input->steering;
  // handbrake
  output->handbrake = input->handbrake;
  // manual
  output->manual = input->manual;
  // manual_gear
  output->manual_gear = input->manual_gear;
  // gear_immediate
  output->gear_immediate = input->gear_immediate;
  return true;
}

airsim_interfaces__msg__CarControls *
airsim_interfaces__msg__CarControls__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  airsim_interfaces__msg__CarControls * msg = (airsim_interfaces__msg__CarControls *)allocator.allocate(sizeof(airsim_interfaces__msg__CarControls), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(airsim_interfaces__msg__CarControls));
  bool success = airsim_interfaces__msg__CarControls__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
airsim_interfaces__msg__CarControls__destroy(airsim_interfaces__msg__CarControls * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    airsim_interfaces__msg__CarControls__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
airsim_interfaces__msg__CarControls__Sequence__init(airsim_interfaces__msg__CarControls__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  airsim_interfaces__msg__CarControls * data = NULL;

  if (size) {
    data = (airsim_interfaces__msg__CarControls *)allocator.zero_allocate(size, sizeof(airsim_interfaces__msg__CarControls), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = airsim_interfaces__msg__CarControls__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        airsim_interfaces__msg__CarControls__fini(&data[i - 1]);
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
airsim_interfaces__msg__CarControls__Sequence__fini(airsim_interfaces__msg__CarControls__Sequence * array)
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
      airsim_interfaces__msg__CarControls__fini(&array->data[i]);
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

airsim_interfaces__msg__CarControls__Sequence *
airsim_interfaces__msg__CarControls__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  airsim_interfaces__msg__CarControls__Sequence * array = (airsim_interfaces__msg__CarControls__Sequence *)allocator.allocate(sizeof(airsim_interfaces__msg__CarControls__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = airsim_interfaces__msg__CarControls__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
airsim_interfaces__msg__CarControls__Sequence__destroy(airsim_interfaces__msg__CarControls__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    airsim_interfaces__msg__CarControls__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
airsim_interfaces__msg__CarControls__Sequence__are_equal(const airsim_interfaces__msg__CarControls__Sequence * lhs, const airsim_interfaces__msg__CarControls__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!airsim_interfaces__msg__CarControls__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
airsim_interfaces__msg__CarControls__Sequence__copy(
  const airsim_interfaces__msg__CarControls__Sequence * input,
  airsim_interfaces__msg__CarControls__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(airsim_interfaces__msg__CarControls);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    airsim_interfaces__msg__CarControls * data =
      (airsim_interfaces__msg__CarControls *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!airsim_interfaces__msg__CarControls__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          airsim_interfaces__msg__CarControls__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!airsim_interfaces__msg__CarControls__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
