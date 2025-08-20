// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from airsim_interfaces:msg/ObjectTransformsList.idl
// generated code does not contain a copyright notice

#ifndef AIRSIM_INTERFACES__MSG__DETAIL__OBJECT_TRANSFORMS_LIST__STRUCT_H_
#define AIRSIM_INTERFACES__MSG__DETAIL__OBJECT_TRANSFORMS_LIST__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__struct.h"
// Member 'objects'
#include "geometry_msgs/msg/detail/transform_stamped__struct.h"

/// Struct defined in msg/ObjectTransformsList in the package airsim_interfaces.
typedef struct airsim_interfaces__msg__ObjectTransformsList
{
  std_msgs__msg__Header header;
  geometry_msgs__msg__TransformStamped__Sequence objects;
} airsim_interfaces__msg__ObjectTransformsList;

// Struct for a sequence of airsim_interfaces__msg__ObjectTransformsList.
typedef struct airsim_interfaces__msg__ObjectTransformsList__Sequence
{
  airsim_interfaces__msg__ObjectTransformsList * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} airsim_interfaces__msg__ObjectTransformsList__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // AIRSIM_INTERFACES__MSG__DETAIL__OBJECT_TRANSFORMS_LIST__STRUCT_H_
