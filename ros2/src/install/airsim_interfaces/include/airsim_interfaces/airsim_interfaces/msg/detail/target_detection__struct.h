// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from airsim_interfaces:msg/TargetDetection.idl
// generated code does not contain a copyright notice

#ifndef AIRSIM_INTERFACES__MSG__DETAIL__TARGET_DETECTION__STRUCT_H_
#define AIRSIM_INTERFACES__MSG__DETAIL__TARGET_DETECTION__STRUCT_H_

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
// Member 'vehicle_name'
#include "rosidl_runtime_c/string.h"

/// Struct defined in msg/TargetDetection in the package airsim_interfaces.
typedef struct airsim_interfaces__msg__TargetDetection
{
  std_msgs__msg__Header header;
  rosidl_runtime_c__String vehicle_name;
  float target_x;
  float target_y;
  float target_z;
  float confidence;
} airsim_interfaces__msg__TargetDetection;

// Struct for a sequence of airsim_interfaces__msg__TargetDetection.
typedef struct airsim_interfaces__msg__TargetDetection__Sequence
{
  airsim_interfaces__msg__TargetDetection * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} airsim_interfaces__msg__TargetDetection__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // AIRSIM_INTERFACES__MSG__DETAIL__TARGET_DETECTION__STRUCT_H_
