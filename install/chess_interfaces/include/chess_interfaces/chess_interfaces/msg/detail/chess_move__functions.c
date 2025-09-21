// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from chess_interfaces:msg/ChessMove.idl
// generated code does not contain a copyright notice
#include "chess_interfaces/msg/detail/chess_move__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


// Include directives for member types
// Member `header`
#include "std_msgs/msg/detail/header__functions.h"

bool
chess_interfaces__msg__ChessMove__init(chess_interfaces__msg__ChessMove * msg)
{
  if (!msg) {
    return false;
  }
  // header
  if (!std_msgs__msg__Header__init(&msg->header)) {
    chess_interfaces__msg__ChessMove__fini(msg);
    return false;
  }
  // from_square
  // to_square
  // piece_type
  // captured_piece
  // promotion
  // is_castling
  // is_en_passant
  // is_check
  // is_checkmate
  // confidence
  // thinking_time
  return true;
}

void
chess_interfaces__msg__ChessMove__fini(chess_interfaces__msg__ChessMove * msg)
{
  if (!msg) {
    return;
  }
  // header
  std_msgs__msg__Header__fini(&msg->header);
  // from_square
  // to_square
  // piece_type
  // captured_piece
  // promotion
  // is_castling
  // is_en_passant
  // is_check
  // is_checkmate
  // confidence
  // thinking_time
}

bool
chess_interfaces__msg__ChessMove__are_equal(const chess_interfaces__msg__ChessMove * lhs, const chess_interfaces__msg__ChessMove * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // header
  if (!std_msgs__msg__Header__are_equal(
      &(lhs->header), &(rhs->header)))
  {
    return false;
  }
  // from_square
  if (lhs->from_square != rhs->from_square) {
    return false;
  }
  // to_square
  if (lhs->to_square != rhs->to_square) {
    return false;
  }
  // piece_type
  if (lhs->piece_type != rhs->piece_type) {
    return false;
  }
  // captured_piece
  if (lhs->captured_piece != rhs->captured_piece) {
    return false;
  }
  // promotion
  if (lhs->promotion != rhs->promotion) {
    return false;
  }
  // is_castling
  if (lhs->is_castling != rhs->is_castling) {
    return false;
  }
  // is_en_passant
  if (lhs->is_en_passant != rhs->is_en_passant) {
    return false;
  }
  // is_check
  if (lhs->is_check != rhs->is_check) {
    return false;
  }
  // is_checkmate
  if (lhs->is_checkmate != rhs->is_checkmate) {
    return false;
  }
  // confidence
  if (lhs->confidence != rhs->confidence) {
    return false;
  }
  // thinking_time
  if (lhs->thinking_time != rhs->thinking_time) {
    return false;
  }
  return true;
}

bool
chess_interfaces__msg__ChessMove__copy(
  const chess_interfaces__msg__ChessMove * input,
  chess_interfaces__msg__ChessMove * output)
{
  if (!input || !output) {
    return false;
  }
  // header
  if (!std_msgs__msg__Header__copy(
      &(input->header), &(output->header)))
  {
    return false;
  }
  // from_square
  output->from_square = input->from_square;
  // to_square
  output->to_square = input->to_square;
  // piece_type
  output->piece_type = input->piece_type;
  // captured_piece
  output->captured_piece = input->captured_piece;
  // promotion
  output->promotion = input->promotion;
  // is_castling
  output->is_castling = input->is_castling;
  // is_en_passant
  output->is_en_passant = input->is_en_passant;
  // is_check
  output->is_check = input->is_check;
  // is_checkmate
  output->is_checkmate = input->is_checkmate;
  // confidence
  output->confidence = input->confidence;
  // thinking_time
  output->thinking_time = input->thinking_time;
  return true;
}

chess_interfaces__msg__ChessMove *
chess_interfaces__msg__ChessMove__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  chess_interfaces__msg__ChessMove * msg = (chess_interfaces__msg__ChessMove *)allocator.allocate(sizeof(chess_interfaces__msg__ChessMove), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(chess_interfaces__msg__ChessMove));
  bool success = chess_interfaces__msg__ChessMove__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
chess_interfaces__msg__ChessMove__destroy(chess_interfaces__msg__ChessMove * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    chess_interfaces__msg__ChessMove__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
chess_interfaces__msg__ChessMove__Sequence__init(chess_interfaces__msg__ChessMove__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  chess_interfaces__msg__ChessMove * data = NULL;

  if (size) {
    data = (chess_interfaces__msg__ChessMove *)allocator.zero_allocate(size, sizeof(chess_interfaces__msg__ChessMove), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = chess_interfaces__msg__ChessMove__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        chess_interfaces__msg__ChessMove__fini(&data[i - 1]);
      }
      allocator.deallocate(data, allocator.state);
      return false;
    }
  }
  array->data = data;
  array->size = size;
  array->capacity = size;
  return true;
}

void
chess_interfaces__msg__ChessMove__Sequence__fini(chess_interfaces__msg__ChessMove__Sequence * array)
{
  if (!array) {
    return;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();

  if (array->data) {
    // ensure that data and capacity values are consistent
    assert(array->capacity > 0);
    // finalize all array elements
    for (size_t i = 0; i < array->capacity; ++i) {
      chess_interfaces__msg__ChessMove__fini(&array->data[i]);
    }
    allocator.deallocate(array->data, allocator.state);
    array->data = NULL;
    array->size = 0;
    array->capacity = 0;
  } else {
    // ensure that data, size, and capacity values are consistent
    assert(0 == array->size);
    assert(0 == array->capacity);
  }
}

chess_interfaces__msg__ChessMove__Sequence *
chess_interfaces__msg__ChessMove__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  chess_interfaces__msg__ChessMove__Sequence * array = (chess_interfaces__msg__ChessMove__Sequence *)allocator.allocate(sizeof(chess_interfaces__msg__ChessMove__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = chess_interfaces__msg__ChessMove__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
chess_interfaces__msg__ChessMove__Sequence__destroy(chess_interfaces__msg__ChessMove__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    chess_interfaces__msg__ChessMove__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
chess_interfaces__msg__ChessMove__Sequence__are_equal(const chess_interfaces__msg__ChessMove__Sequence * lhs, const chess_interfaces__msg__ChessMove__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!chess_interfaces__msg__ChessMove__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
chess_interfaces__msg__ChessMove__Sequence__copy(
  const chess_interfaces__msg__ChessMove__Sequence * input,
  chess_interfaces__msg__ChessMove__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(chess_interfaces__msg__ChessMove);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    chess_interfaces__msg__ChessMove * data =
      (chess_interfaces__msg__ChessMove *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!chess_interfaces__msg__ChessMove__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          chess_interfaces__msg__ChessMove__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!chess_interfaces__msg__ChessMove__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
