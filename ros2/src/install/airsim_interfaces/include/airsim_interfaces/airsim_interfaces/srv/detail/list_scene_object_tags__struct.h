// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from airsim_interfaces:srv/ListSceneObjectTags.idl
// generated code does not contain a copyright notice

#ifndef AIRSIM_INTERFACES__SRV__DETAIL__LIST_SCENE_OBJECT_TAGS__STRUCT_H_
#define AIRSIM_INTERFACES__SRV__DETAIL__LIST_SCENE_OBJECT_TAGS__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'regex_name'
#include "rosidl_runtime_c/string.h"

/// Struct defined in srv/ListSceneObjectTags in the package airsim_interfaces.
typedef struct airsim_interfaces__srv__ListSceneObjectTags_Request
{
  rosidl_runtime_c__String regex_name;
} airsim_interfaces__srv__ListSceneObjectTags_Request;

// Struct for a sequence of airsim_interfaces__srv__ListSceneObjectTags_Request.
typedef struct airsim_interfaces__srv__ListSceneObjectTags_Request__Sequence
{
  airsim_interfaces__srv__ListSceneObjectTags_Request * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} airsim_interfaces__srv__ListSceneObjectTags_Request__Sequence;


// Constants defined in the message

// Include directives for member types
// Member 'objects'
// Member 'tags'
// already included above
// #include "rosidl_runtime_c/string.h"

/// Struct defined in srv/ListSceneObjectTags in the package airsim_interfaces.
typedef struct airsim_interfaces__srv__ListSceneObjectTags_Response
{
  rosidl_runtime_c__String__Sequence objects;
  rosidl_runtime_c__String__Sequence tags;
} airsim_interfaces__srv__ListSceneObjectTags_Response;

// Struct for a sequence of airsim_interfaces__srv__ListSceneObjectTags_Response.
typedef struct airsim_interfaces__srv__ListSceneObjectTags_Response__Sequence
{
  airsim_interfaces__srv__ListSceneObjectTags_Response * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} airsim_interfaces__srv__ListSceneObjectTags_Response__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // AIRSIM_INTERFACES__SRV__DETAIL__LIST_SCENE_OBJECT_TAGS__STRUCT_H_
