// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from chess_interfaces:msg/BoardState.idl
// generated code does not contain a copyright notice
#include "chess_interfaces/msg/detail/board_state__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


// Include directives for member types
// Member `header`
#include "std_msgs/msg/detail/header__functions.h"
// Member `square_positions_3d`
#include "geometry_msgs/msg/detail/point__functions.h"

bool
chess_interfaces__msg__BoardState__init(chess_interfaces__msg__BoardState * msg)
{
  if (!msg) {
    return false;
  }
  // header
  if (!std_msgs__msg__Header__init(&msg->header)) {
    chess_interfaces__msg__BoardState__fini(msg);
    return false;
  }
  // board_squares
  // square_positions_3d
  for (size_t i = 0; i < 64; ++i) {
    if (!geometry_msgs__msg__Point__init(&msg->square_positions_3d[i])) {
      chess_interfaces__msg__BoardState__fini(msg);
      return false;
    }
  }
  // white_to_move
  // castling_rights
  // en_passant_square
  // halfmove_clock
  // fullmove_number
  return true;
}

void
chess_interfaces__msg__BoardState__fini(chess_interfaces__msg__BoardState * msg)
{
  if (!msg) {
    return;
  }
  // header
  std_msgs__msg__Header__fini(&msg->header);
  // board_squares
  // square_positions_3d
  for (size_t i = 0; i < 64; ++i) {
    geometry_msgs__msg__Point__fini(&msg->square_positions_3d[i]);
  }
  // white_to_move
  // castling_rights
  // en_passant_square
  // halfmove_clock
  // fullmove_number
}

bool
chess_interfaces__msg__BoardState__are_equal(const chess_interfaces__msg__BoardState * lhs, const chess_interfaces__msg__BoardState * rhs)
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
  // board_squares
  for (size_t i = 0; i < 64; ++i) {
    if (lhs->board_squares[i] != rhs->board_squares[i]) {
      return false;
    }
  }
  // square_positions_3d
  for (size_t i = 0; i < 64; ++i) {
    if (!geometry_msgs__msg__Point__are_equal(
        &(lhs->square_positions_3d[i]), &(rhs->square_positions_3d[i])))
    {
      return false;
    }
  }
  // white_to_move
  if (lhs->white_to_move != rhs->white_to_move) {
    return false;
  }
  // castling_rights
  for (size_t i = 0; i < 4; ++i) {
    if (lhs->castling_rights[i] != rhs->castling_rights[i]) {
      return false;
    }
  }
  // en_passant_square
  if (lhs->en_passant_square != rhs->en_passant_square) {
    return false;
  }
  // halfmove_clock
  if (lhs->halfmove_clock != rhs->halfmove_clock) {
    return false;
  }
  // fullmove_number
  if (lhs->fullmove_number != rhs->fullmove_number) {
    return false;
  }
  return true;
}

bool
chess_interfaces__msg__BoardState__copy(
  const chess_interfaces__msg__BoardState * input,
  chess_interfaces__msg__BoardState * output)
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
  // board_squares
  for (size_t i = 0; i < 64; ++i) {
    output->board_squares[i] = input->board_squares[i];
  }
  // square_positions_3d
  for (size_t i = 0; i < 64; ++i) {
    if (!geometry_msgs__msg__Point__copy(
        &(input->square_positions_3d[i]), &(output->square_positions_3d[i])))
    {
      return false;
    }
  }
  // white_to_move
  output->white_to_move = input->white_to_move;
  // castling_rights
  for (size_t i = 0; i < 4; ++i) {
    output->castling_rights[i] = input->castling_rights[i];
  }
  // en_passant_square
  output->en_passant_square = input->en_passant_square;
  // halfmove_clock
  output->halfmove_clock = input->halfmove_clock;
  // fullmove_number
  output->fullmove_number = input->fullmove_number;
  return true;
}

chess_interfaces__msg__BoardState *
chess_interfaces__msg__BoardState__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  chess_interfaces__msg__BoardState * msg = (chess_interfaces__msg__BoardState *)allocator.allocate(sizeof(chess_interfaces__msg__BoardState), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(chess_interfaces__msg__BoardState));
  bool success = chess_interfaces__msg__BoardState__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
chess_interfaces__msg__BoardState__destroy(chess_interfaces__msg__BoardState * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    chess_interfaces__msg__BoardState__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
chess_interfaces__msg__BoardState__Sequence__init(chess_interfaces__msg__BoardState__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  chess_interfaces__msg__BoardState * data = NULL;

  if (size) {
    data = (chess_interfaces__msg__BoardState *)allocator.zero_allocate(size, sizeof(chess_interfaces__msg__BoardState), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = chess_interfaces__msg__BoardState__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        chess_interfaces__msg__BoardState__fini(&data[i - 1]);
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
chess_interfaces__msg__BoardState__Sequence__fini(chess_interfaces__msg__BoardState__Sequence * array)
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
      chess_interfaces__msg__BoardState__fini(&array->data[i]);
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

chess_interfaces__msg__BoardState__Sequence *
chess_interfaces__msg__BoardState__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  chess_interfaces__msg__BoardState__Sequence * array = (chess_interfaces__msg__BoardState__Sequence *)allocator.allocate(sizeof(chess_interfaces__msg__BoardState__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = chess_interfaces__msg__BoardState__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
chess_interfaces__msg__BoardState__Sequence__destroy(chess_interfaces__msg__BoardState__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    chess_interfaces__msg__BoardState__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
chess_interfaces__msg__BoardState__Sequence__are_equal(const chess_interfaces__msg__BoardState__Sequence * lhs, const chess_interfaces__msg__BoardState__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!chess_interfaces__msg__BoardState__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
chess_interfaces__msg__BoardState__Sequence__copy(
  const chess_interfaces__msg__BoardState__Sequence * input,
  chess_interfaces__msg__BoardState__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(chess_interfaces__msg__BoardState);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    chess_interfaces__msg__BoardState * data =
      (chess_interfaces__msg__BoardState *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!chess_interfaces__msg__BoardState__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          chess_interfaces__msg__BoardState__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!chess_interfaces__msg__BoardState__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
