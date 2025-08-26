// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from airsim_interfaces:msg/VelCmd.idl
// generated code does not contain a copyright notice

#ifndef AIRSIM_INTERFACES__MSG__DETAIL__VEL_CMD__STRUCT_H_
#define AIRSIM_INTERFACES__MSG__DETAIL__VEL_CMD__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

/// Constant 'DRIVETRAIN_MAX_DEGREE_OF_FREEDOM'.
enum
{
  airsim_interfaces__msg__VelCmd__DRIVETRAIN_MAX_DEGREE_OF_FREEDOM = 0
};

/// Constant 'DRIVETRAIN_FORWARD_ONLY'.
enum
{
  airsim_interfaces__msg__VelCmd__DRIVETRAIN_FORWARD_ONLY = 1
};

// Include directives for member types
// Member 'twist'
#include "geometry_msgs/msg/detail/twist__struct.h"
// Member 'yaw_mode'
#include "airsim_interfaces/msg/detail/yaw_mode__struct.h"

/// Struct defined in msg/VelCmd in the package airsim_interfaces.
/**
  * Velocity command message for multirotor vehicles
 */
typedef struct airsim_interfaces__msg__VelCmd
{
  geometry_msgs__msg__Twist twist;
  /// Drivetrain type
  uint8_t drivetrain;
  /// Yaw mode configuration
  airsim_interfaces__msg__YawMode yaw_mode;
} airsim_interfaces__msg__VelCmd;

// Struct for a sequence of airsim_interfaces__msg__VelCmd.
typedef struct airsim_interfaces__msg__VelCmd__Sequence
{
  airsim_interfaces__msg__VelCmd * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} airsim_interfaces__msg__VelCmd__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // AIRSIM_INTERFACES__MSG__DETAIL__VEL_CMD__STRUCT_H_
