// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from airsim_interfaces:msg/YawMode.idl
// generated code does not contain a copyright notice

#ifndef AIRSIM_INTERFACES__MSG__DETAIL__YAW_MODE__STRUCT_H_
#define AIRSIM_INTERFACES__MSG__DETAIL__YAW_MODE__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

/// Struct defined in msg/YawMode in the package airsim_interfaces.
/**
  * Yaw mode configuration for vehicle movement
 */
typedef struct airsim_interfaces__msg__YawMode
{
  /// true = yaw rate, false = yaw angle
  bool is_rate;
  /// yaw angle in radians or yaw rate in radians/sec
  float yaw_or_rate;
} airsim_interfaces__msg__YawMode;

// Struct for a sequence of airsim_interfaces__msg__YawMode.
typedef struct airsim_interfaces__msg__YawMode__Sequence
{
  airsim_interfaces__msg__YawMode * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} airsim_interfaces__msg__YawMode__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // AIRSIM_INTERFACES__MSG__DETAIL__YAW_MODE__STRUCT_H_
