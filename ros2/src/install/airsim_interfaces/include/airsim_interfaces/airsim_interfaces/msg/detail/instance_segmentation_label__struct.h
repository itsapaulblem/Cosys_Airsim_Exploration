// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from airsim_interfaces:msg/InstanceSegmentationLabel.idl
// generated code does not contain a copyright notice

#ifndef AIRSIM_INTERFACES__MSG__DETAIL__INSTANCE_SEGMENTATION_LABEL__STRUCT_H_
#define AIRSIM_INTERFACES__MSG__DETAIL__INSTANCE_SEGMENTATION_LABEL__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'name'
#include "rosidl_runtime_c/string.h"

/// Struct defined in msg/InstanceSegmentationLabel in the package airsim_interfaces.
typedef struct airsim_interfaces__msg__InstanceSegmentationLabel
{
  rosidl_runtime_c__String name;
  uint32_t index;
  uint8_t r;
  uint8_t g;
  uint8_t b;
} airsim_interfaces__msg__InstanceSegmentationLabel;

// Struct for a sequence of airsim_interfaces__msg__InstanceSegmentationLabel.
typedef struct airsim_interfaces__msg__InstanceSegmentationLabel__Sequence
{
  airsim_interfaces__msg__InstanceSegmentationLabel * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} airsim_interfaces__msg__InstanceSegmentationLabel__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // AIRSIM_INTERFACES__MSG__DETAIL__INSTANCE_SEGMENTATION_LABEL__STRUCT_H_
