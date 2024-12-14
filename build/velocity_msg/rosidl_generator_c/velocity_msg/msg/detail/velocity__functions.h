// generated from rosidl_generator_c/resource/idl__functions.h.em
// with input from velocity_msg:msg/Velocity.idl
// generated code does not contain a copyright notice

#ifndef VELOCITY_MSG__MSG__DETAIL__VELOCITY__FUNCTIONS_H_
#define VELOCITY_MSG__MSG__DETAIL__VELOCITY__FUNCTIONS_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stdlib.h>

#include "rosidl_runtime_c/visibility_control.h"
#include "velocity_msg/msg/rosidl_generator_c__visibility_control.h"

#include "velocity_msg/msg/detail/velocity__struct.h"

/// Initialize msg/Velocity message.
/**
 * If the init function is called twice for the same message without
 * calling fini inbetween previously allocated memory will be leaked.
 * \param[in,out] msg The previously allocated message pointer.
 * Fields without a default value will not be initialized by this function.
 * You might want to call memset(msg, 0, sizeof(
 * velocity_msg__msg__Velocity
 * )) before or use
 * velocity_msg__msg__Velocity__create()
 * to allocate and initialize the message.
 * \return true if initialization was successful, otherwise false
 */
ROSIDL_GENERATOR_C_PUBLIC_velocity_msg
bool
velocity_msg__msg__Velocity__init(velocity_msg__msg__Velocity * msg);

/// Finalize msg/Velocity message.
/**
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_velocity_msg
void
velocity_msg__msg__Velocity__fini(velocity_msg__msg__Velocity * msg);

/// Create msg/Velocity message.
/**
 * It allocates the memory for the message, sets the memory to zero, and
 * calls
 * velocity_msg__msg__Velocity__init().
 * \return The pointer to the initialized message if successful,
 * otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_velocity_msg
velocity_msg__msg__Velocity *
velocity_msg__msg__Velocity__create();

/// Destroy msg/Velocity message.
/**
 * It calls
 * velocity_msg__msg__Velocity__fini()
 * and frees the memory of the message.
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_velocity_msg
void
velocity_msg__msg__Velocity__destroy(velocity_msg__msg__Velocity * msg);

/// Check for msg/Velocity message equality.
/**
 * \param[in] lhs The message on the left hand size of the equality operator.
 * \param[in] rhs The message on the right hand size of the equality operator.
 * \return true if messages are equal, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_velocity_msg
bool
velocity_msg__msg__Velocity__are_equal(const velocity_msg__msg__Velocity * lhs, const velocity_msg__msg__Velocity * rhs);

/// Copy a msg/Velocity message.
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
ROSIDL_GENERATOR_C_PUBLIC_velocity_msg
bool
velocity_msg__msg__Velocity__copy(
  const velocity_msg__msg__Velocity * input,
  velocity_msg__msg__Velocity * output);

/// Initialize array of msg/Velocity messages.
/**
 * It allocates the memory for the number of elements and calls
 * velocity_msg__msg__Velocity__init()
 * for each element of the array.
 * \param[in,out] array The allocated array pointer.
 * \param[in] size The size / capacity of the array.
 * \return true if initialization was successful, otherwise false
 * If the array pointer is valid and the size is zero it is guaranteed
 # to return true.
 */
ROSIDL_GENERATOR_C_PUBLIC_velocity_msg
bool
velocity_msg__msg__Velocity__Sequence__init(velocity_msg__msg__Velocity__Sequence * array, size_t size);

/// Finalize array of msg/Velocity messages.
/**
 * It calls
 * velocity_msg__msg__Velocity__fini()
 * for each element of the array and frees the memory for the number of
 * elements.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_velocity_msg
void
velocity_msg__msg__Velocity__Sequence__fini(velocity_msg__msg__Velocity__Sequence * array);

/// Create array of msg/Velocity messages.
/**
 * It allocates the memory for the array and calls
 * velocity_msg__msg__Velocity__Sequence__init().
 * \param[in] size The size / capacity of the array.
 * \return The pointer to the initialized array if successful, otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_velocity_msg
velocity_msg__msg__Velocity__Sequence *
velocity_msg__msg__Velocity__Sequence__create(size_t size);

/// Destroy array of msg/Velocity messages.
/**
 * It calls
 * velocity_msg__msg__Velocity__Sequence__fini()
 * on the array,
 * and frees the memory of the array.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_velocity_msg
void
velocity_msg__msg__Velocity__Sequence__destroy(velocity_msg__msg__Velocity__Sequence * array);

/// Check for msg/Velocity message array equality.
/**
 * \param[in] lhs The message array on the left hand size of the equality operator.
 * \param[in] rhs The message array on the right hand size of the equality operator.
 * \return true if message arrays are equal in size and content, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_velocity_msg
bool
velocity_msg__msg__Velocity__Sequence__are_equal(const velocity_msg__msg__Velocity__Sequence * lhs, const velocity_msg__msg__Velocity__Sequence * rhs);

/// Copy an array of msg/Velocity messages.
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
ROSIDL_GENERATOR_C_PUBLIC_velocity_msg
bool
velocity_msg__msg__Velocity__Sequence__copy(
  const velocity_msg__msg__Velocity__Sequence * input,
  velocity_msg__msg__Velocity__Sequence * output);

#ifdef __cplusplus
}
#endif

#endif  // VELOCITY_MSG__MSG__DETAIL__VELOCITY__FUNCTIONS_H_
