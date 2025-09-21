// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from chess_interfaces:srv/ExecuteMove.idl
// generated code does not contain a copyright notice
#include "chess_interfaces/srv/detail/execute_move__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"

// Include directives for member types
// Member `chess_move`
#include "chess_interfaces/msg/detail/chess_move__functions.h"

bool
chess_interfaces__srv__ExecuteMove_Request__init(chess_interfaces__srv__ExecuteMove_Request * msg)
{
  if (!msg) {
    return false;
  }
  // chess_move
  if (!chess_interfaces__msg__ChessMove__init(&msg->chess_move)) {
    chess_interfaces__srv__ExecuteMove_Request__fini(msg);
    return false;
  }
  // confirm_execution
  return true;
}

void
chess_interfaces__srv__ExecuteMove_Request__fini(chess_interfaces__srv__ExecuteMove_Request * msg)
{
  if (!msg) {
    return;
  }
  // chess_move
  chess_interfaces__msg__ChessMove__fini(&msg->chess_move);
  // confirm_execution
}

bool
chess_interfaces__srv__ExecuteMove_Request__are_equal(const chess_interfaces__srv__ExecuteMove_Request * lhs, const chess_interfaces__srv__ExecuteMove_Request * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // chess_move
  if (!chess_interfaces__msg__ChessMove__are_equal(
      &(lhs->chess_move), &(rhs->chess_move)))
  {
    return false;
  }
  // confirm_execution
  if (lhs->confirm_execution != rhs->confirm_execution) {
    return false;
  }
  return true;
}

bool
chess_interfaces__srv__ExecuteMove_Request__copy(
  const chess_interfaces__srv__ExecuteMove_Request * input,
  chess_interfaces__srv__ExecuteMove_Request * output)
{
  if (!input || !output) {
    return false;
  }
  // chess_move
  if (!chess_interfaces__msg__ChessMove__copy(
      &(input->chess_move), &(output->chess_move)))
  {
    return false;
  }
  // confirm_execution
  output->confirm_execution = input->confirm_execution;
  return true;
}

chess_interfaces__srv__ExecuteMove_Request *
chess_interfaces__srv__ExecuteMove_Request__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  chess_interfaces__srv__ExecuteMove_Request * msg = (chess_interfaces__srv__ExecuteMove_Request *)allocator.allocate(sizeof(chess_interfaces__srv__ExecuteMove_Request), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(chess_interfaces__srv__ExecuteMove_Request));
  bool success = chess_interfaces__srv__ExecuteMove_Request__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
chess_interfaces__srv__ExecuteMove_Request__destroy(chess_interfaces__srv__ExecuteMove_Request * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    chess_interfaces__srv__ExecuteMove_Request__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
chess_interfaces__srv__ExecuteMove_Request__Sequence__init(chess_interfaces__srv__ExecuteMove_Request__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  chess_interfaces__srv__ExecuteMove_Request * data = NULL;

  if (size) {
    data = (chess_interfaces__srv__ExecuteMove_Request *)allocator.zero_allocate(size, sizeof(chess_interfaces__srv__ExecuteMove_Request), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = chess_interfaces__srv__ExecuteMove_Request__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        chess_interfaces__srv__ExecuteMove_Request__fini(&data[i - 1]);
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
chess_interfaces__srv__ExecuteMove_Request__Sequence__fini(chess_interfaces__srv__ExecuteMove_Request__Sequence * array)
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
      chess_interfaces__srv__ExecuteMove_Request__fini(&array->data[i]);
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

chess_interfaces__srv__ExecuteMove_Request__Sequence *
chess_interfaces__srv__ExecuteMove_Request__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  chess_interfaces__srv__ExecuteMove_Request__Sequence * array = (chess_interfaces__srv__ExecuteMove_Request__Sequence *)allocator.allocate(sizeof(chess_interfaces__srv__ExecuteMove_Request__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = chess_interfaces__srv__ExecuteMove_Request__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
chess_interfaces__srv__ExecuteMove_Request__Sequence__destroy(chess_interfaces__srv__ExecuteMove_Request__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    chess_interfaces__srv__ExecuteMove_Request__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
chess_interfaces__srv__ExecuteMove_Request__Sequence__are_equal(const chess_interfaces__srv__ExecuteMove_Request__Sequence * lhs, const chess_interfaces__srv__ExecuteMove_Request__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!chess_interfaces__srv__ExecuteMove_Request__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
chess_interfaces__srv__ExecuteMove_Request__Sequence__copy(
  const chess_interfaces__srv__ExecuteMove_Request__Sequence * input,
  chess_interfaces__srv__ExecuteMove_Request__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(chess_interfaces__srv__ExecuteMove_Request);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    chess_interfaces__srv__ExecuteMove_Request * data =
      (chess_interfaces__srv__ExecuteMove_Request *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!chess_interfaces__srv__ExecuteMove_Request__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          chess_interfaces__srv__ExecuteMove_Request__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!chess_interfaces__srv__ExecuteMove_Request__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}


// Include directives for member types
// Member `error_message`
#include "rosidl_runtime_c/string_functions.h"

bool
chess_interfaces__srv__ExecuteMove_Response__init(chess_interfaces__srv__ExecuteMove_Response * msg)
{
  if (!msg) {
    return false;
  }
  // success
  // error_message
  if (!rosidl_runtime_c__String__init(&msg->error_message)) {
    chess_interfaces__srv__ExecuteMove_Response__fini(msg);
    return false;
  }
  // actual_execution_time
  // piece_captured
  return true;
}

void
chess_interfaces__srv__ExecuteMove_Response__fini(chess_interfaces__srv__ExecuteMove_Response * msg)
{
  if (!msg) {
    return;
  }
  // success
  // error_message
  rosidl_runtime_c__String__fini(&msg->error_message);
  // actual_execution_time
  // piece_captured
}

bool
chess_interfaces__srv__ExecuteMove_Response__are_equal(const chess_interfaces__srv__ExecuteMove_Response * lhs, const chess_interfaces__srv__ExecuteMove_Response * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // success
  if (lhs->success != rhs->success) {
    return false;
  }
  // error_message
  if (!rosidl_runtime_c__String__are_equal(
      &(lhs->error_message), &(rhs->error_message)))
  {
    return false;
  }
  // actual_execution_time
  if (lhs->actual_execution_time != rhs->actual_execution_time) {
    return false;
  }
  // piece_captured
  if (lhs->piece_captured != rhs->piece_captured) {
    return false;
  }
  return true;
}

bool
chess_interfaces__srv__ExecuteMove_Response__copy(
  const chess_interfaces__srv__ExecuteMove_Response * input,
  chess_interfaces__srv__ExecuteMove_Response * output)
{
  if (!input || !output) {
    return false;
  }
  // success
  output->success = input->success;
  // error_message
  if (!rosidl_runtime_c__String__copy(
      &(input->error_message), &(output->error_message)))
  {
    return false;
  }
  // actual_execution_time
  output->actual_execution_time = input->actual_execution_time;
  // piece_captured
  output->piece_captured = input->piece_captured;
  return true;
}

chess_interfaces__srv__ExecuteMove_Response *
chess_interfaces__srv__ExecuteMove_Response__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  chess_interfaces__srv__ExecuteMove_Response * msg = (chess_interfaces__srv__ExecuteMove_Response *)allocator.allocate(sizeof(chess_interfaces__srv__ExecuteMove_Response), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(chess_interfaces__srv__ExecuteMove_Response));
  bool success = chess_interfaces__srv__ExecuteMove_Response__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
chess_interfaces__srv__ExecuteMove_Response__destroy(chess_interfaces__srv__ExecuteMove_Response * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    chess_interfaces__srv__ExecuteMove_Response__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
chess_interfaces__srv__ExecuteMove_Response__Sequence__init(chess_interfaces__srv__ExecuteMove_Response__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  chess_interfaces__srv__ExecuteMove_Response * data = NULL;

  if (size) {
    data = (chess_interfaces__srv__ExecuteMove_Response *)allocator.zero_allocate(size, sizeof(chess_interfaces__srv__ExecuteMove_Response), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = chess_interfaces__srv__ExecuteMove_Response__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        chess_interfaces__srv__ExecuteMove_Response__fini(&data[i - 1]);
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
chess_interfaces__srv__ExecuteMove_Response__Sequence__fini(chess_interfaces__srv__ExecuteMove_Response__Sequence * array)
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
      chess_interfaces__srv__ExecuteMove_Response__fini(&array->data[i]);
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

chess_interfaces__srv__ExecuteMove_Response__Sequence *
chess_interfaces__srv__ExecuteMove_Response__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  chess_interfaces__srv__ExecuteMove_Response__Sequence * array = (chess_interfaces__srv__ExecuteMove_Response__Sequence *)allocator.allocate(sizeof(chess_interfaces__srv__ExecuteMove_Response__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = chess_interfaces__srv__ExecuteMove_Response__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
chess_interfaces__srv__ExecuteMove_Response__Sequence__destroy(chess_interfaces__srv__ExecuteMove_Response__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    chess_interfaces__srv__ExecuteMove_Response__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
chess_interfaces__srv__ExecuteMove_Response__Sequence__are_equal(const chess_interfaces__srv__ExecuteMove_Response__Sequence * lhs, const chess_interfaces__srv__ExecuteMove_Response__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!chess_interfaces__srv__ExecuteMove_Response__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
chess_interfaces__srv__ExecuteMove_Response__Sequence__copy(
  const chess_interfaces__srv__ExecuteMove_Response__Sequence * input,
  chess_interfaces__srv__ExecuteMove_Response__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(chess_interfaces__srv__ExecuteMove_Response);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    chess_interfaces__srv__ExecuteMove_Response * data =
      (chess_interfaces__srv__ExecuteMove_Response *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!chess_interfaces__srv__ExecuteMove_Response__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          chess_interfaces__srv__ExecuteMove_Response__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!chess_interfaces__srv__ExecuteMove_Response__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
