// generated from rosidl_generator_c/resource/idl__functions.h.em
// with input from chess_interfaces:msg/PieceDetection.idl
// generated code does not contain a copyright notice

#ifndef CHESS_INTERFACES__MSG__DETAIL__PIECE_DETECTION__FUNCTIONS_H_
#define CHESS_INTERFACES__MSG__DETAIL__PIECE_DETECTION__FUNCTIONS_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stdlib.h>

#include "rosidl_runtime_c/visibility_control.h"
#include "chess_interfaces/msg/rosidl_generator_c__visibility_control.h"

#include "chess_interfaces/msg/detail/piece_detection__struct.h"

/// Initialize msg/PieceDetection message.
/**
 * If the init function is called twice for the same message without
 * calling fini inbetween previously allocated memory will be leaked.
 * \param[in,out] msg The previously allocated message pointer.
 * Fields without a default value will not be initialized by this function.
 * You might want to call memset(msg, 0, sizeof(
 * chess_interfaces__msg__PieceDetection
 * )) before or use
 * chess_interfaces__msg__PieceDetection__create()
 * to allocate and initialize the message.
 * \return true if initialization was successful, otherwise false
 */
ROSIDL_GENERATOR_C_PUBLIC_chess_interfaces
bool
chess_interfaces__msg__PieceDetection__init(chess_interfaces__msg__PieceDetection * msg);

/// Finalize msg/PieceDetection message.
/**
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_chess_interfaces
void
chess_interfaces__msg__PieceDetection__fini(chess_interfaces__msg__PieceDetection * msg);

/// Create msg/PieceDetection message.
/**
 * It allocates the memory for the message, sets the memory to zero, and
 * calls
 * chess_interfaces__msg__PieceDetection__init().
 * \return The pointer to the initialized message if successful,
 * otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_chess_interfaces
chess_interfaces__msg__PieceDetection *
chess_interfaces__msg__PieceDetection__create();

/// Destroy msg/PieceDetection message.
/**
 * It calls
 * chess_interfaces__msg__PieceDetection__fini()
 * and frees the memory of the message.
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_chess_interfaces
void
chess_interfaces__msg__PieceDetection__destroy(chess_interfaces__msg__PieceDetection * msg);

/// Check for msg/PieceDetection message equality.
/**
 * \param[in] lhs The message on the left hand size of the equality operator.
 * \param[in] rhs The message on the right hand size of the equality operator.
 * \return true if messages are equal, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_chess_interfaces
bool
chess_interfaces__msg__PieceDetection__are_equal(const chess_interfaces__msg__PieceDetection * lhs, const chess_interfaces__msg__PieceDetection * rhs);

/// Copy a msg/PieceDetection message.
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
ROSIDL_GENERATOR_C_PUBLIC_chess_interfaces
bool
chess_interfaces__msg__PieceDetection__copy(
  const chess_interfaces__msg__PieceDetection * input,
  chess_interfaces__msg__PieceDetection * output);

/// Initialize array of msg/PieceDetection messages.
/**
 * It allocates the memory for the number of elements and calls
 * chess_interfaces__msg__PieceDetection__init()
 * for each element of the array.
 * \param[in,out] array The allocated array pointer.
 * \param[in] size The size / capacity of the array.
 * \return true if initialization was successful, otherwise false
 * If the array pointer is valid and the size is zero it is guaranteed
 # to return true.
 */
ROSIDL_GENERATOR_C_PUBLIC_chess_interfaces
bool
chess_interfaces__msg__PieceDetection__Sequence__init(chess_interfaces__msg__PieceDetection__Sequence * array, size_t size);

/// Finalize array of msg/PieceDetection messages.
/**
 * It calls
 * chess_interfaces__msg__PieceDetection__fini()
 * for each element of the array and frees the memory for the number of
 * elements.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_chess_interfaces
void
chess_interfaces__msg__PieceDetection__Sequence__fini(chess_interfaces__msg__PieceDetection__Sequence * array);

/// Create array of msg/PieceDetection messages.
/**
 * It allocates the memory for the array and calls
 * chess_interfaces__msg__PieceDetection__Sequence__init().
 * \param[in] size The size / capacity of the array.
 * \return The pointer to the initialized array if successful, otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_chess_interfaces
chess_interfaces__msg__PieceDetection__Sequence *
chess_interfaces__msg__PieceDetection__Sequence__create(size_t size);

/// Destroy array of msg/PieceDetection messages.
/**
 * It calls
 * chess_interfaces__msg__PieceDetection__Sequence__fini()
 * on the array,
 * and frees the memory of the array.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_chess_interfaces
void
chess_interfaces__msg__PieceDetection__Sequence__destroy(chess_interfaces__msg__PieceDetection__Sequence * array);

/// Check for msg/PieceDetection message array equality.
/**
 * \param[in] lhs The message array on the left hand size of the equality operator.
 * \param[in] rhs The message array on the right hand size of the equality operator.
 * \return true if message arrays are equal in size and content, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_chess_interfaces
bool
chess_interfaces__msg__PieceDetection__Sequence__are_equal(const chess_interfaces__msg__PieceDetection__Sequence * lhs, const chess_interfaces__msg__PieceDetection__Sequence * rhs);

/// Copy an array of msg/PieceDetection messages.
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
ROSIDL_GENERATOR_C_PUBLIC_chess_interfaces
bool
chess_interfaces__msg__PieceDetection__Sequence__copy(
  const chess_interfaces__msg__PieceDetection__Sequence * input,
  chess_interfaces__msg__PieceDetection__Sequence * output);

#ifdef __cplusplus
}
#endif

#endif  // CHESS_INTERFACES__MSG__DETAIL__PIECE_DETECTION__FUNCTIONS_H_
