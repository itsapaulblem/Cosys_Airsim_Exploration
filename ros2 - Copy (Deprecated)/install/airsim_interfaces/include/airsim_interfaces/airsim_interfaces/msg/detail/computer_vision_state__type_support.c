// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from airsim_interfaces:msg/ComputerVisionState.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "airsim_interfaces/msg/detail/computer_vision_state__rosidl_typesupport_introspection_c.h"
#include "airsim_interfaces/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "airsim_interfaces/msg/detail/computer_vision_state__functions.h"
#include "airsim_interfaces/msg/detail/computer_vision_state__struct.h"


// Include directives for member types
// Member `header`
#include "std_msgs/msg/header.h"
// Member `header`
#include "std_msgs/msg/detail/header__rosidl_typesupport_introspection_c.h"
// Member `pose`
#include "geometry_msgs/msg/pose_with_covariance.h"
// Member `pose`
#include "geometry_msgs/msg/detail/pose_with_covariance__rosidl_typesupport_introspection_c.h"
// Member `twist`
#include "geometry_msgs/msg/twist_with_covariance.h"
// Member `twist`
#include "geometry_msgs/msg/detail/twist_with_covariance__rosidl_typesupport_introspection_c.h"

#ifdef __cplusplus
extern "C"
{
#endif

void airsim_interfaces__msg__ComputerVisionState__rosidl_typesupport_introspection_c__ComputerVisionState_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  airsim_interfaces__msg__ComputerVisionState__init(message_memory);
}

void airsim_interfaces__msg__ComputerVisionState__rosidl_typesupport_introspection_c__ComputerVisionState_fini_function(void * message_memory)
{
  airsim_interfaces__msg__ComputerVisionState__fini(message_memory);
}

static rosidl_typesupport_introspection_c__MessageMember airsim_interfaces__msg__ComputerVisionState__rosidl_typesupport_introspection_c__ComputerVisionState_message_member_array[3] = {
  {
    "header",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(airsim_interfaces__msg__ComputerVisionState, header),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "pose",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(airsim_interfaces__msg__ComputerVisionState, pose),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "twist",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(airsim_interfaces__msg__ComputerVisionState, twist),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers airsim_interfaces__msg__ComputerVisionState__rosidl_typesupport_introspection_c__ComputerVisionState_message_members = {
  "airsim_interfaces__msg",  // message namespace
  "ComputerVisionState",  // message name
  3,  // number of fields
  sizeof(airsim_interfaces__msg__ComputerVisionState),
  airsim_interfaces__msg__ComputerVisionState__rosidl_typesupport_introspection_c__ComputerVisionState_message_member_array,  // message members
  airsim_interfaces__msg__ComputerVisionState__rosidl_typesupport_introspection_c__ComputerVisionState_init_function,  // function to initialize message memory (memory has to be allocated)
  airsim_interfaces__msg__ComputerVisionState__rosidl_typesupport_introspection_c__ComputerVisionState_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t airsim_interfaces__msg__ComputerVisionState__rosidl_typesupport_introspection_c__ComputerVisionState_message_type_support_handle = {
  0,
  &airsim_interfaces__msg__ComputerVisionState__rosidl_typesupport_introspection_c__ComputerVisionState_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_airsim_interfaces
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, airsim_interfaces, msg, ComputerVisionState)() {
  airsim_interfaces__msg__ComputerVisionState__rosidl_typesupport_introspection_c__ComputerVisionState_message_member_array[0].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, std_msgs, msg, Header)();
  airsim_interfaces__msg__ComputerVisionState__rosidl_typesupport_introspection_c__ComputerVisionState_message_member_array[1].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, geometry_msgs, msg, PoseWithCovariance)();
  airsim_interfaces__msg__ComputerVisionState__rosidl_typesupport_introspection_c__ComputerVisionState_message_member_array[2].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, geometry_msgs, msg, TwistWithCovariance)();
  if (!airsim_interfaces__msg__ComputerVisionState__rosidl_typesupport_introspection_c__ComputerVisionState_message_type_support_handle.typesupport_identifier) {
    airsim_interfaces__msg__ComputerVisionState__rosidl_typesupport_introspection_c__ComputerVisionState_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &airsim_interfaces__msg__ComputerVisionState__rosidl_typesupport_introspection_c__ComputerVisionState_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif
