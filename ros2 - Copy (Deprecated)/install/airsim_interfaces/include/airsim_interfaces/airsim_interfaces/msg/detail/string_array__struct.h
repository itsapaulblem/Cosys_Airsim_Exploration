// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from airsim_interfaces:msg/StringArray.idl
// generated code does not contain a copyright notice

#ifndef AIRSIM_INTERFACES__MSG__DETAIL__STRING_ARRAY__STRUCT_H_
#define AIRSIM_INTERFACES__MSG__DETAIL__STRING_ARRAY__STRUCT_H_

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
// Member 'data'
#include "rosidl_runtime_c/string.h"

/// Struct defined in msg/StringArray in the package airsim_interfaces.
typedef struct airsim_interfaces__msg__StringArray
{
  std_msgs__msg__Header header;
  rosidl_runtime_c__String__Sequence data;
} airsim_interfaces__msg__StringArray;

// Struct for a sequence of airsim_interfaces__msg__StringArray.
typedef struct airsim_interfaces__msg__StringArray__Sequence
{
  airsim_interfaces__msg__StringArray * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} airsim_interfaces__msg__StringArray__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // AIRSIM_INTERFACES__MSG__DETAIL__STRING_ARRAY__STRUCT_H_
