// generated from rosidl_generator_c/resource/idl__functions.h.em
// with input from rbpodo_msgs:msg/SystemState.idl
// generated code does not contain a copyright notice

#ifndef RBPODO_MSGS__MSG__DETAIL__SYSTEM_STATE__FUNCTIONS_H_
#define RBPODO_MSGS__MSG__DETAIL__SYSTEM_STATE__FUNCTIONS_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stdlib.h>

#include "rosidl_runtime_c/visibility_control.h"
#include "rbpodo_msgs/msg/rosidl_generator_c__visibility_control.h"

#include "rbpodo_msgs/msg/detail/system_state__struct.h"

/// Initialize msg/SystemState message.
/**
 * If the init function is called twice for the same message without
 * calling fini inbetween previously allocated memory will be leaked.
 * \param[in,out] msg The previously allocated message pointer.
 * Fields without a default value will not be initialized by this function.
 * You might want to call memset(msg, 0, sizeof(
 * rbpodo_msgs__msg__SystemState
 * )) before or use
 * rbpodo_msgs__msg__SystemState__create()
 * to allocate and initialize the message.
 * \return true if initialization was successful, otherwise false
 */
ROSIDL_GENERATOR_C_PUBLIC_rbpodo_msgs
bool
rbpodo_msgs__msg__SystemState__init(rbpodo_msgs__msg__SystemState * msg);

/// Finalize msg/SystemState message.
/**
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_rbpodo_msgs
void
rbpodo_msgs__msg__SystemState__fini(rbpodo_msgs__msg__SystemState * msg);

/// Create msg/SystemState message.
/**
 * It allocates the memory for the message, sets the memory to zero, and
 * calls
 * rbpodo_msgs__msg__SystemState__init().
 * \return The pointer to the initialized message if successful,
 * otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_rbpodo_msgs
rbpodo_msgs__msg__SystemState *
rbpodo_msgs__msg__SystemState__create();

/// Destroy msg/SystemState message.
/**
 * It calls
 * rbpodo_msgs__msg__SystemState__fini()
 * and frees the memory of the message.
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_rbpodo_msgs
void
rbpodo_msgs__msg__SystemState__destroy(rbpodo_msgs__msg__SystemState * msg);

/// Check for msg/SystemState message equality.
/**
 * \param[in] lhs The message on the left hand size of the equality operator.
 * \param[in] rhs The message on the right hand size of the equality operator.
 * \return true if messages are equal, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_rbpodo_msgs
bool
rbpodo_msgs__msg__SystemState__are_equal(const rbpodo_msgs__msg__SystemState * lhs, const rbpodo_msgs__msg__SystemState * rhs);

/// Copy a msg/SystemState message.
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
ROSIDL_GENERATOR_C_PUBLIC_rbpodo_msgs
bool
rbpodo_msgs__msg__SystemState__copy(
  const rbpodo_msgs__msg__SystemState * input,
  rbpodo_msgs__msg__SystemState * output);

/// Initialize array of msg/SystemState messages.
/**
 * It allocates the memory for the number of elements and calls
 * rbpodo_msgs__msg__SystemState__init()
 * for each element of the array.
 * \param[in,out] array The allocated array pointer.
 * \param[in] size The size / capacity of the array.
 * \return true if initialization was successful, otherwise false
 * If the array pointer is valid and the size is zero it is guaranteed
 # to return true.
 */
ROSIDL_GENERATOR_C_PUBLIC_rbpodo_msgs
bool
rbpodo_msgs__msg__SystemState__Sequence__init(rbpodo_msgs__msg__SystemState__Sequence * array, size_t size);

/// Finalize array of msg/SystemState messages.
/**
 * It calls
 * rbpodo_msgs__msg__SystemState__fini()
 * for each element of the array and frees the memory for the number of
 * elements.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_rbpodo_msgs
void
rbpodo_msgs__msg__SystemState__Sequence__fini(rbpodo_msgs__msg__SystemState__Sequence * array);

/// Create array of msg/SystemState messages.
/**
 * It allocates the memory for the array and calls
 * rbpodo_msgs__msg__SystemState__Sequence__init().
 * \param[in] size The size / capacity of the array.
 * \return The pointer to the initialized array if successful, otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_rbpodo_msgs
rbpodo_msgs__msg__SystemState__Sequence *
rbpodo_msgs__msg__SystemState__Sequence__create(size_t size);

/// Destroy array of msg/SystemState messages.
/**
 * It calls
 * rbpodo_msgs__msg__SystemState__Sequence__fini()
 * on the array,
 * and frees the memory of the array.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_rbpodo_msgs
void
rbpodo_msgs__msg__SystemState__Sequence__destroy(rbpodo_msgs__msg__SystemState__Sequence * array);

/// Check for msg/SystemState message array equality.
/**
 * \param[in] lhs The message array on the left hand size of the equality operator.
 * \param[in] rhs The message array on the right hand size of the equality operator.
 * \return true if message arrays are equal in size and content, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_rbpodo_msgs
bool
rbpodo_msgs__msg__SystemState__Sequence__are_equal(const rbpodo_msgs__msg__SystemState__Sequence * lhs, const rbpodo_msgs__msg__SystemState__Sequence * rhs);

/// Copy an array of msg/SystemState messages.
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
ROSIDL_GENERATOR_C_PUBLIC_rbpodo_msgs
bool
rbpodo_msgs__msg__SystemState__Sequence__copy(
  const rbpodo_msgs__msg__SystemState__Sequence * input,
  rbpodo_msgs__msg__SystemState__Sequence * output);

#ifdef __cplusplus
}
#endif

#endif  // RBPODO_MSGS__MSG__DETAIL__SYSTEM_STATE__FUNCTIONS_H_
