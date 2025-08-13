// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from airsim_interfaces:msg/InstanceSegmentationList.idl
// generated code does not contain a copyright notice

#ifndef AIRSIM_INTERFACES__MSG__DETAIL__INSTANCE_SEGMENTATION_LIST__STRUCT_H_
#define AIRSIM_INTERFACES__MSG__DETAIL__INSTANCE_SEGMENTATION_LIST__STRUCT_H_

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
// Member 'labels'
#include "airsim_interfaces/msg/detail/instance_segmentation_label__struct.h"

/// Struct defined in msg/InstanceSegmentationList in the package airsim_interfaces.
typedef struct airsim_interfaces__msg__InstanceSegmentationList
{
  std_msgs__msg__Header header;
  airsim_interfaces__msg__InstanceSegmentationLabel__Sequence labels;
} airsim_interfaces__msg__InstanceSegmentationList;

// Struct for a sequence of airsim_interfaces__msg__InstanceSegmentationList.
typedef struct airsim_interfaces__msg__InstanceSegmentationList__Sequence
{
  airsim_interfaces__msg__InstanceSegmentationList * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} airsim_interfaces__msg__InstanceSegmentationList__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // AIRSIM_INTERFACES__MSG__DETAIL__INSTANCE_SEGMENTATION_LIST__STRUCT_H_
