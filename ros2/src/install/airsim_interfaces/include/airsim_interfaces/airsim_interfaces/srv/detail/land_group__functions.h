// generated from rosidl_generator_c/resource/idl__functions.h.em
// with input from airsim_interfaces:srv/LandGroup.idl
// generated code does not contain a copyright notice

#ifndef AIRSIM_INTERFACES__SRV__DETAIL__LAND_GROUP__FUNCTIONS_H_
#define AIRSIM_INTERFACES__SRV__DETAIL__LAND_GROUP__FUNCTIONS_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stdlib.h>

#include "rosidl_runtime_c/visibility_control.h"
#include "airsim_interfaces/msg/rosidl_generator_c__visibility_control.h"

#include "airsim_interfaces/srv/detail/land_group__struct.h"

/// Initialize srv/LandGroup message.
/**
 * If the init function is called twice for the same message without
 * calling fini inbetween previously allocated memory will be leaked.
 * \param[in,out] msg The previously allocated message pointer.
 * Fields without a default value will not be initialized by this function.
 * You might want to call memset(msg, 0, sizeof(
 * airsim_interfaces__srv__LandGroup_Request
 * )) before or use
 * airsim_interfaces__srv__LandGroup_Request__create()
 * to allocate and initialize the message.
 * \return true if initialization was successful, otherwise false
 */
ROSIDL_GENERATOR_C_PUBLIC_airsim_interfaces
bool
airsim_interfaces__srv__LandGroup_Request__init(airsim_interfaces__srv__LandGroup_Request * msg);

/// Finalize srv/LandGroup message.
/**
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_airsim_interfaces
void
airsim_interfaces__srv__LandGroup_Request__fini(airsim_interfaces__srv__LandGroup_Request * msg);

/// Create srv/LandGroup message.
/**
 * It allocates the memory for the message, sets the memory to zero, and
 * calls
 * airsim_interfaces__srv__LandGroup_Request__init().
 * \return The pointer to the initialized message if successful,
 * otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_airsim_interfaces
airsim_interfaces__srv__LandGroup_Request *
airsim_interfaces__srv__LandGroup_Request__create();

/// Destroy srv/LandGroup message.
/**
 * It calls
 * airsim_interfaces__srv__LandGroup_Request__fini()
 * and frees the memory of the message.
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_airsim_interfaces
void
airsim_interfaces__srv__LandGroup_Request__destroy(airsim_interfaces__srv__LandGroup_Request * msg);

/// Check for srv/LandGroup message equality.
/**
 * \param[in] lhs The message on the left hand size of the equality operator.
 * \param[in] rhs The message on the right hand size of the equality operator.
 * \return true if messages are equal, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_airsim_interfaces
bool
airsim_interfaces__srv__LandGroup_Request__are_equal(const airsim_interfaces__srv__LandGroup_Request * lhs, const airsim_interfaces__srv__LandGroup_Request * rhs);

/// Copy a srv/LandGroup message.
/**
 * This functions performs a deep copy, as opposed to the shallow copy that
 * plain assignment yields.
 *
 * \param[in] input The source message pointer.
 * \param[out] output The target message pointer, which must
 *   have been initialized before calling this function.
 * \return true if successful, or false if either pointer is null
 *   or memory allocation fails.
 */
ROSIDL_GENERATOR_C_PUBLIC_airsim_interfaces
bool
airsim_interfaces__srv__LandGroup_Request__copy(
  const airsim_interfaces__srv__LandGroup_Request * input,
  airsim_interfaces__srv__LandGroup_Request * output);

/// Initialize array of srv/LandGroup messages.
/**
 * It allocates the memory for the number of elements and calls
 * airsim_interfaces__srv__LandGroup_Request__init()
 * for each element of the array.
 * \param[in,out] array The allocated array pointer.
 * \param[in] size The size / capacity of the array.
 * \return true if initialization was successful, otherwise false
 * If the array pointer is valid and the size is zero it is guaranteed
 # to return true.
 */
ROSIDL_GENERATOR_C_PUBLIC_airsim_interfaces
bool
airsim_interfaces__srv__LandGroup_Request__Sequence__init(airsim_interfaces__srv__LandGroup_Request__Sequence * array, size_t size);

/// Finalize array of srv/LandGroup messages.
/**
 * It calls
 * airsim_interfaces__srv__LandGroup_Request__fini()
 * for each element of the array and frees the memory for the number of
 * elements.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_airsim_interfaces
void
airsim_interfaces__srv__LandGroup_Request__Sequence__fini(airsim_interfaces__srv__LandGroup_Request__Sequence * array);

/// Create array of srv/LandGroup messages.
/**
 * It allocates the memory for the array and calls
 * airsim_interfaces__srv__LandGroup_Request__Sequence__init().
 * \param[in] size The size / capacity of the array.
 * \return The pointer to the initialized array if successful, otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_airsim_interfaces
airsim_interfaces__srv__LandGroup_Request__Sequence *
airsim_interfaces__srv__LandGroup_Request__Sequence__create(size_t size);

/// Destroy array of srv/LandGroup messages.
/**
 * It calls
 * airsim_interfaces__srv__LandGroup_Request__Sequence__fini()
 * on the array,
 * and frees the memory of the array.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_airsim_interfaces
void
airsim_interfaces__srv__LandGroup_Request__Sequence__destroy(airsim_interfaces__srv__LandGroup_Request__Sequence * array);

/// Check for srv/LandGroup message array equality.
/**
 * \param[in] lhs The message array on the left hand size of the equality operator.
 * \param[in] rhs The message array on the right hand size of the equality operator.
 * \return true if message arrays are equal in size and content, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_airsim_interfaces
bool
airsim_interfaces__srv__LandGroup_Request__Sequence__are_equal(const airsim_interfaces__srv__LandGroup_Request__Sequence * lhs, const airsim_interfaces__srv__LandGroup_Request__Sequence * rhs);

/// Copy an array of srv/LandGroup messages.
/**
 * This functions performs a deep copy, as opposed to the shallow copy that
 * plain assignment yields.
 *
 * \param[in] input The source array pointer.
 * \param[out] output The target array pointer, which must
 *   have been initialized before calling this function.
 * \return true if successful, or false if either pointer
 *   is null or memory allocation fails.
 */
ROSIDL_GENERATOR_C_PUBLIC_airsim_interfaces
bool
airsim_interfaces__srv__LandGroup_Request__Sequence__copy(
  const airsim_interfaces__srv__LandGroup_Request__Sequence * input,
  airsim_interfaces__srv__LandGroup_Request__Sequence * output);

/// Initialize srv/LandGroup message.
/**
 * If the init function is called twice for the same message without
 * calling fini inbetween previously allocated memory will be leaked.
 * \param[in,out] msg The previously allocated message pointer.
 * Fields without a default value will not be initialized by this function.
 * You might want to call memset(msg, 0, sizeof(
 * airsim_interfaces__srv__LandGroup_Response
 * )) before or use
 * airsim_interfaces__srv__LandGroup_Response__create()
 * to allocate and initialize the message.
 * \return true if initialization was successful, otherwise false
 */
ROSIDL_GENERATOR_C_PUBLIC_airsim_interfaces
bool
airsim_interfaces__srv__LandGroup_Response__init(airsim_interfaces__srv__LandGroup_Response * msg);

/// Finalize srv/LandGroup message.
/**
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_airsim_interfaces
void
airsim_interfaces__srv__LandGroup_Response__fini(airsim_interfaces__srv__LandGroup_Response * msg);

/// Create srv/LandGroup message.
/**
 * It allocates the memory for the message, sets the memory to zero, and
 * calls
 * airsim_interfaces__srv__LandGroup_Response__init().
 * \return The pointer to the initialized message if successful,
 * otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_airsim_interfaces
airsim_interfaces__srv__LandGroup_Response *
airsim_interfaces__srv__LandGroup_Response__create();

/// Destroy srv/LandGroup message.
/**
 * It calls
 * airsim_interfaces__srv__LandGroup_Response__fini()
 * and frees the memory of the message.
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_airsim_interfaces
void
airsim_interfaces__srv__LandGroup_Response__destroy(airsim_interfaces__srv__LandGroup_Response * msg);

/// Check for srv/LandGroup message equality.
/**
 * \param[in] lhs The message on the left hand size of the equality operator.
 * \param[in] rhs The message on the right hand size of the equality operator.
 * \return true if messages are equal, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_airsim_interfaces
bool
airsim_interfaces__srv__LandGroup_Response__are_equal(const airsim_interfaces__srv__LandGroup_Response * lhs, const airsim_interfaces__srv__LandGroup_Response * rhs);

/// Copy a srv/LandGroup message.
/**
 * This functions performs a deep copy, as opposed to the shallow copy that
 * plain assignment yields.
 *
 * \param[in] input The source message pointer.
 * \param[out] output The target message pointer, which must
 *   have been initialized before calling this function.
 * \return true if successful, or false if either pointer is null
 *   or memory allocation fails.
 */
ROSIDL_GENERATOR_C_PUBLIC_airsim_interfaces
bool
airsim_interfaces__srv__LandGroup_Response__copy(
  const airsim_interfaces__srv__LandGroup_Response * input,
  airsim_interfaces__srv__LandGroup_Response * output);

/// Initialize array of srv/LandGroup messages.
/**
 * It allocates the memory for the number of elements and calls
 * airsim_interfaces__srv__LandGroup_Response__init()
 * for each element of the array.
 * \param[in,out] array The allocated array pointer.
 * \param[in] size The size / capacity of the array.
 * \return true if initialization was successful, otherwise false
 * If the array pointer is valid and the size is zero it is guaranteed
 # to return true.
 */
ROSIDL_GENERATOR_C_PUBLIC_airsim_interfaces
bool
airsim_interfaces__srv__LandGroup_Response__Sequence__init(airsim_interfaces__srv__LandGroup_Response__Sequence * array, size_t size);

/// Finalize array of srv/LandGroup messages.
/**
 * It calls
 * airsim_interfaces__srv__LandGroup_Response__fini()
 * for each element of the array and frees the memory for the number of
 * elements.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_airsim_interfaces
void
airsim_interfaces__srv__LandGroup_Response__Sequence__fini(airsim_interfaces__srv__LandGroup_Response__Sequence * array);

/// Create array of srv/LandGroup messages.
/**
 * It allocates the memory for the array and calls
 * airsim_interfaces__srv__LandGroup_Response__Sequence__init().
 * \param[in] size The size / capacity of the array.
 * \return The pointer to the initialized array if successful, otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_airsim_interfaces
airsim_interfaces__srv__LandGroup_Response__Sequence *
airsim_interfaces__srv__LandGroup_Response__Sequence__create(size_t size);

/// Destroy array of srv/LandGroup messages.
/**
 * It calls
 * airsim_interfaces__srv__LandGroup_Response__Sequence__fini()
 * on the array,
 * and frees the memory of the array.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_airsim_interfaces
void
airsim_interfaces__srv__LandGroup_Response__Sequence__destroy(airsim_interfaces__srv__LandGroup_Response__Sequence * array);

/// Check for srv/LandGroup message array equality.
/**
 * \param[in] lhs The message array on the left hand size of the equality operator.
 * \param[in] rhs The message array on the right hand size of the equality operator.
 * \return true if message arrays are equal in size and content, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_airsim_interfaces
bool
airsim_interfaces__srv__LandGroup_Response__Sequence__are_equal(const airsim_interfaces__srv__LandGroup_Response__Sequence * lhs, const airsim_interfaces__srv__LandGroup_Response__Sequence * rhs);

/// Copy an array of srv/LandGroup messages.
/**
 * This functions performs a deep copy, as opposed to the shallow copy that
 * plain assignment yields.
 *
 * \param[in] input The source array pointer.
 * \param[out] output The target array pointer, which must
 *   have been initialized before calling this function.
 * \return true if successful, or false if either pointer
 *   is null or memory allocation fails.
 */
ROSIDL_GENERATOR_C_PUBLIC_airsim_interfaces
bool
airsim_interfaces__srv__LandGroup_Response__Sequence__copy(
  const airsim_interfaces__srv__LandGroup_Response__Sequence * input,
  airsim_interfaces__srv__LandGroup_Response__Sequence * output);

#ifdef __cplusplus
}
#endif

#endif  // AIRSIM_INTERFACES__SRV__DETAIL__LAND_GROUP__FUNCTIONS_H_
