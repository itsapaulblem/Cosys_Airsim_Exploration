// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from airsim_interfaces:msg/Environment.idl
// generated code does not contain a copyright notice
#include "airsim_interfaces/msg/detail/environment__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


// Include directives for member types
// Member `header`
#include "std_msgs/msg/detail/header__functions.h"
// Member `position`
// Member `gravity`
#include "geometry_msgs/msg/detail/vector3__functions.h"
// Member `geo_point`
#include "geographic_msgs/msg/detail/geo_point__functions.h"

bool
airsim_interfaces__msg__Environment__init(airsim_interfaces__msg__Environment * msg)
{
  if (!msg) {
    return false;
  }
  // header
  if (!std_msgs__msg__Header__init(&msg->header)) {
    airsim_interfaces__msg__Environment__fini(msg);
    return false;
  }
  // position
  if (!geometry_msgs__msg__Vector3__init(&msg->position)) {
    airsim_interfaces__msg__Environment__fini(msg);
    return false;
  }
  // geo_point
  if (!geographic_msgs__msg__GeoPoint__init(&msg->geo_point)) {
    airsim_interfaces__msg__Environment__fini(msg);
    return false;
  }
  // gravity
  if (!geometry_msgs__msg__Vector3__init(&msg->gravity)) {
    airsim_interfaces__msg__Environment__fini(msg);
    return false;
  }
  // air_pressure
  // temperature
  // air_density
  return true;
}

void
airsim_interfaces__msg__Environment__fini(airsim_interfaces__msg__Environment * msg)
{
  if (!msg) {
    return;
  }
  // header
  std_msgs__msg__Header__fini(&msg->header);
  // position
  geometry_msgs__msg__Vector3__fini(&msg->position);
  // geo_point
  geographic_msgs__msg__GeoPoint__fini(&msg->geo_point);
  // gravity
  geometry_msgs__msg__Vector3__fini(&msg->gravity);
  // air_pressure
  // temperature
  // air_density
}

bool
airsim_interfaces__msg__Environment__are_equal(const airsim_interfaces__msg__Environment * lhs, const airsim_interfaces__msg__Environment * rhs)
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
  // position
  if (!geometry_msgs__msg__Vector3__are_equal(
      &(lhs->position), &(rhs->position)))
  {
    return false;
  }
  // geo_point
  if (!geographic_msgs__msg__GeoPoint__are_equal(
      &(lhs->geo_point), &(rhs->geo_point)))
  {
    return false;
  }
  // gravity
  if (!geometry_msgs__msg__Vector3__are_equal(
      &(lhs->gravity), &(rhs->gravity)))
  {
    return false;
  }
  // air_pressure
  if (lhs->air_pressure != rhs->air_pressure) {
    return false;
  }
  // temperature
  if (lhs->temperature != rhs->temperature) {
    return false;
  }
  // air_density
  if (lhs->air_density != rhs->air_density) {
    return false;
  }
  return true;
}

bool
airsim_interfaces__msg__Environment__copy(
  const airsim_interfaces__msg__Environment * input,
  airsim_interfaces__msg__Environment * output)
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
  // position
  if (!geometry_msgs__msg__Vector3__copy(
      &(input->position), &(output->position)))
  {
    return false;
  }
  // geo_point
  if (!geographic_msgs__msg__GeoPoint__copy(
      &(input->geo_point), &(output->geo_point)))
  {
    return false;
  }
  // gravity
  if (!geometry_msgs__msg__Vector3__copy(
      &(input->gravity), &(output->gravity)))
  {
    return false;
  }
  // air_pressure
  output->air_pressure = input->air_pressure;
  // temperature
  output->temperature = input->temperature;
  // air_density
  output->air_density = input->air_density;
  return true;
}

airsim_interfaces__msg__Environment *
airsim_interfaces__msg__Environment__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  airsim_interfaces__msg__Environment * msg = (airsim_interfaces__msg__Environment *)allocator.allocate(sizeof(airsim_interfaces__msg__Environment), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(airsim_interfaces__msg__Environment));
  bool success = airsim_interfaces__msg__Environment__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
airsim_interfaces__msg__Environment__destroy(airsim_interfaces__msg__Environment * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    airsim_interfaces__msg__Environment__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
airsim_interfaces__msg__Environment__Sequence__init(airsim_interfaces__msg__Environment__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  airsim_interfaces__msg__Environment * data = NULL;

  if (size) {
    data = (airsim_interfaces__msg__Environment *)allocator.zero_allocate(size, sizeof(airsim_interfaces__msg__Environment), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = airsim_interfaces__msg__Environment__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        airsim_interfaces__msg__Environment__fini(&data[i - 1]);
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
airsim_interfaces__msg__Environment__Sequence__fini(airsim_interfaces__msg__Environment__Sequence * array)
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
      airsim_interfaces__msg__Environment__fini(&array->data[i]);
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

airsim_interfaces__msg__Environment__Sequence *
airsim_interfaces__msg__Environment__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  airsim_interfaces__msg__Environment__Sequence * array = (airsim_interfaces__msg__Environment__Sequence *)allocator.allocate(sizeof(airsim_interfaces__msg__Environment__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = airsim_interfaces__msg__Environment__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
airsim_interfaces__msg__Environment__Sequence__destroy(airsim_interfaces__msg__Environment__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    airsim_interfaces__msg__Environment__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
airsim_interfaces__msg__Environment__Sequence__are_equal(const airsim_interfaces__msg__Environment__Sequence * lhs, const airsim_interfaces__msg__Environment__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!airsim_interfaces__msg__Environment__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
airsim_interfaces__msg__Environment__Sequence__copy(
  const airsim_interfaces__msg__Environment__Sequence * input,
  airsim_interfaces__msg__Environment__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(airsim_interfaces__msg__Environment);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    airsim_interfaces__msg__Environment * data =
      (airsim_interfaces__msg__Environment *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!airsim_interfaces__msg__Environment__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          airsim_interfaces__msg__Environment__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!airsim_interfaces__msg__Environment__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
