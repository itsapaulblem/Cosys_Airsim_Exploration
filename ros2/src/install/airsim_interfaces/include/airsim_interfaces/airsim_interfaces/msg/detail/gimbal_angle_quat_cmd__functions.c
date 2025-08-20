// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from airsim_interfaces:msg/GimbalAngleQuatCmd.idl
// generated code does not contain a copyright notice
#include "airsim_interfaces/msg/detail/gimbal_angle_quat_cmd__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


// Include directives for member types
// Member `header`
#include "std_msgs/msg/detail/header__functions.h"
// Member `camera_name`
// Member `vehicle_name`
#include "rosidl_runtime_c/string_functions.h"
// Member `orientation`
#include "geometry_msgs/msg/detail/quaternion__functions.h"

bool
airsim_interfaces__msg__GimbalAngleQuatCmd__init(airsim_interfaces__msg__GimbalAngleQuatCmd * msg)
{
  if (!msg) {
    return false;
  }
  // header
  if (!std_msgs__msg__Header__init(&msg->header)) {
    airsim_interfaces__msg__GimbalAngleQuatCmd__fini(msg);
    return false;
  }
  // camera_name
  if (!rosidl_runtime_c__String__init(&msg->camera_name)) {
    airsim_interfaces__msg__GimbalAngleQuatCmd__fini(msg);
    return false;
  }
  // vehicle_name
  if (!rosidl_runtime_c__String__init(&msg->vehicle_name)) {
    airsim_interfaces__msg__GimbalAngleQuatCmd__fini(msg);
    return false;
  }
  // orientation
  if (!geometry_msgs__msg__Quaternion__init(&msg->orientation)) {
    airsim_interfaces__msg__GimbalAngleQuatCmd__fini(msg);
    return false;
  }
  return true;
}

void
airsim_interfaces__msg__GimbalAngleQuatCmd__fini(airsim_interfaces__msg__GimbalAngleQuatCmd * msg)
{
  if (!msg) {
    return;
  }
  // header
  std_msgs__msg__Header__fini(&msg->header);
  // camera_name
  rosidl_runtime_c__String__fini(&msg->camera_name);
  // vehicle_name
  rosidl_runtime_c__String__fini(&msg->vehicle_name);
  // orientation
  geometry_msgs__msg__Quaternion__fini(&msg->orientation);
}

bool
airsim_interfaces__msg__GimbalAngleQuatCmd__are_equal(const airsim_interfaces__msg__GimbalAngleQuatCmd * lhs, const airsim_interfaces__msg__GimbalAngleQuatCmd * rhs)
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
  // camera_name
  if (!rosidl_runtime_c__String__are_equal(
      &(lhs->camera_name), &(rhs->camera_name)))
  {
    return false;
  }
  // vehicle_name
  if (!rosidl_runtime_c__String__are_equal(
      &(lhs->vehicle_name), &(rhs->vehicle_name)))
  {
    return false;
  }
  // orientation
  if (!geometry_msgs__msg__Quaternion__are_equal(
      &(lhs->orientation), &(rhs->orientation)))
  {
    return false;
  }
  return true;
}

bool
airsim_interfaces__msg__GimbalAngleQuatCmd__copy(
  const airsim_interfaces__msg__GimbalAngleQuatCmd * input,
  airsim_interfaces__msg__GimbalAngleQuatCmd * output)
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
  // camera_name
  if (!rosidl_runtime_c__String__copy(
      &(input->camera_name), &(output->camera_name)))
  {
    return false;
  }
  // vehicle_name
  if (!rosidl_runtime_c__String__copy(
      &(input->vehicle_name), &(output->vehicle_name)))
  {
    return false;
  }
  // orientation
  if (!geometry_msgs__msg__Quaternion__copy(
      &(input->orientation), &(output->orientation)))
  {
    return false;
  }
  return true;
}

airsim_interfaces__msg__GimbalAngleQuatCmd *
airsim_interfaces__msg__GimbalAngleQuatCmd__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  airsim_interfaces__msg__GimbalAngleQuatCmd * msg = (airsim_interfaces__msg__GimbalAngleQuatCmd *)allocator.allocate(sizeof(airsim_interfaces__msg__GimbalAngleQuatCmd), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(airsim_interfaces__msg__GimbalAngleQuatCmd));
  bool success = airsim_interfaces__msg__GimbalAngleQuatCmd__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
airsim_interfaces__msg__GimbalAngleQuatCmd__destroy(airsim_interfaces__msg__GimbalAngleQuatCmd * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    airsim_interfaces__msg__GimbalAngleQuatCmd__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
airsim_interfaces__msg__GimbalAngleQuatCmd__Sequence__init(airsim_interfaces__msg__GimbalAngleQuatCmd__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  airsim_interfaces__msg__GimbalAngleQuatCmd * data = NULL;

  if (size) {
    data = (airsim_interfaces__msg__GimbalAngleQuatCmd *)allocator.zero_allocate(size, sizeof(airsim_interfaces__msg__GimbalAngleQuatCmd), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = airsim_interfaces__msg__GimbalAngleQuatCmd__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        airsim_interfaces__msg__GimbalAngleQuatCmd__fini(&data[i - 1]);
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
airsim_interfaces__msg__GimbalAngleQuatCmd__Sequence__fini(airsim_interfaces__msg__GimbalAngleQuatCmd__Sequence * array)
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
      airsim_interfaces__msg__GimbalAngleQuatCmd__fini(&array->data[i]);
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

airsim_interfaces__msg__GimbalAngleQuatCmd__Sequence *
airsim_interfaces__msg__GimbalAngleQuatCmd__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  airsim_interfaces__msg__GimbalAngleQuatCmd__Sequence * array = (airsim_interfaces__msg__GimbalAngleQuatCmd__Sequence *)allocator.allocate(sizeof(airsim_interfaces__msg__GimbalAngleQuatCmd__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = airsim_interfaces__msg__GimbalAngleQuatCmd__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
airsim_interfaces__msg__GimbalAngleQuatCmd__Sequence__destroy(airsim_interfaces__msg__GimbalAngleQuatCmd__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    airsim_interfaces__msg__GimbalAngleQuatCmd__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
airsim_interfaces__msg__GimbalAngleQuatCmd__Sequence__are_equal(const airsim_interfaces__msg__GimbalAngleQuatCmd__Sequence * lhs, const airsim_interfaces__msg__GimbalAngleQuatCmd__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!airsim_interfaces__msg__GimbalAngleQuatCmd__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
airsim_interfaces__msg__GimbalAngleQuatCmd__Sequence__copy(
  const airsim_interfaces__msg__GimbalAngleQuatCmd__Sequence * input,
  airsim_interfaces__msg__GimbalAngleQuatCmd__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(airsim_interfaces__msg__GimbalAngleQuatCmd);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    airsim_interfaces__msg__GimbalAngleQuatCmd * data =
      (airsim_interfaces__msg__GimbalAngleQuatCmd *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!airsim_interfaces__msg__GimbalAngleQuatCmd__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          airsim_interfaces__msg__GimbalAngleQuatCmd__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!airsim_interfaces__msg__GimbalAngleQuatCmd__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
