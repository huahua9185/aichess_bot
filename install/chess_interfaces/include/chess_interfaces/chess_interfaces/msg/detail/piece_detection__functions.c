// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from chess_interfaces:msg/PieceDetection.idl
// generated code does not contain a copyright notice
#include "chess_interfaces/msg/detail/piece_detection__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


// Include directives for member types
// Member `header`
#include "std_msgs/msg/detail/header__functions.h"
// Member `position_3d`
// Member `position_2d`
#include "geometry_msgs/msg/detail/point__functions.h"
// Member `roi_image`
#include "sensor_msgs/msg/detail/image__functions.h"

bool
chess_interfaces__msg__PieceDetection__init(chess_interfaces__msg__PieceDetection * msg)
{
  if (!msg) {
    return false;
  }
  // header
  if (!std_msgs__msg__Header__init(&msg->header)) {
    chess_interfaces__msg__PieceDetection__fini(msg);
    return false;
  }
  // position_3d
  if (!geometry_msgs__msg__Point__init(&msg->position_3d)) {
    chess_interfaces__msg__PieceDetection__fini(msg);
    return false;
  }
  // position_2d
  if (!geometry_msgs__msg__Point__init(&msg->position_2d)) {
    chess_interfaces__msg__PieceDetection__fini(msg);
    return false;
  }
  // piece_type
  // piece_color
  // confidence
  // roi_image
  if (!sensor_msgs__msg__Image__init(&msg->roi_image)) {
    chess_interfaces__msg__PieceDetection__fini(msg);
    return false;
  }
  return true;
}

void
chess_interfaces__msg__PieceDetection__fini(chess_interfaces__msg__PieceDetection * msg)
{
  if (!msg) {
    return;
  }
  // header
  std_msgs__msg__Header__fini(&msg->header);
  // position_3d
  geometry_msgs__msg__Point__fini(&msg->position_3d);
  // position_2d
  geometry_msgs__msg__Point__fini(&msg->position_2d);
  // piece_type
  // piece_color
  // confidence
  // roi_image
  sensor_msgs__msg__Image__fini(&msg->roi_image);
}

bool
chess_interfaces__msg__PieceDetection__are_equal(const chess_interfaces__msg__PieceDetection * lhs, const chess_interfaces__msg__PieceDetection * rhs)
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
  // position_3d
  if (!geometry_msgs__msg__Point__are_equal(
      &(lhs->position_3d), &(rhs->position_3d)))
  {
    return false;
  }
  // position_2d
  if (!geometry_msgs__msg__Point__are_equal(
      &(lhs->position_2d), &(rhs->position_2d)))
  {
    return false;
  }
  // piece_type
  if (lhs->piece_type != rhs->piece_type) {
    return false;
  }
  // piece_color
  if (lhs->piece_color != rhs->piece_color) {
    return false;
  }
  // confidence
  if (lhs->confidence != rhs->confidence) {
    return false;
  }
  // roi_image
  if (!sensor_msgs__msg__Image__are_equal(
      &(lhs->roi_image), &(rhs->roi_image)))
  {
    return false;
  }
  return true;
}

bool
chess_interfaces__msg__PieceDetection__copy(
  const chess_interfaces__msg__PieceDetection * input,
  chess_interfaces__msg__PieceDetection * output)
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
  // position_3d
  if (!geometry_msgs__msg__Point__copy(
      &(input->position_3d), &(output->position_3d)))
  {
    return false;
  }
  // position_2d
  if (!geometry_msgs__msg__Point__copy(
      &(input->position_2d), &(output->position_2d)))
  {
    return false;
  }
  // piece_type
  output->piece_type = input->piece_type;
  // piece_color
  output->piece_color = input->piece_color;
  // confidence
  output->confidence = input->confidence;
  // roi_image
  if (!sensor_msgs__msg__Image__copy(
      &(input->roi_image), &(output->roi_image)))
  {
    return false;
  }
  return true;
}

chess_interfaces__msg__PieceDetection *
chess_interfaces__msg__PieceDetection__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  chess_interfaces__msg__PieceDetection * msg = (chess_interfaces__msg__PieceDetection *)allocator.allocate(sizeof(chess_interfaces__msg__PieceDetection), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(chess_interfaces__msg__PieceDetection));
  bool success = chess_interfaces__msg__PieceDetection__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
chess_interfaces__msg__PieceDetection__destroy(chess_interfaces__msg__PieceDetection * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    chess_interfaces__msg__PieceDetection__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
chess_interfaces__msg__PieceDetection__Sequence__init(chess_interfaces__msg__PieceDetection__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  chess_interfaces__msg__PieceDetection * data = NULL;

  if (size) {
    data = (chess_interfaces__msg__PieceDetection *)allocator.zero_allocate(size, sizeof(chess_interfaces__msg__PieceDetection), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = chess_interfaces__msg__PieceDetection__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        chess_interfaces__msg__PieceDetection__fini(&data[i - 1]);
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
chess_interfaces__msg__PieceDetection__Sequence__fini(chess_interfaces__msg__PieceDetection__Sequence * array)
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
      chess_interfaces__msg__PieceDetection__fini(&array->data[i]);
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

chess_interfaces__msg__PieceDetection__Sequence *
chess_interfaces__msg__PieceDetection__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  chess_interfaces__msg__PieceDetection__Sequence * array = (chess_interfaces__msg__PieceDetection__Sequence *)allocator.allocate(sizeof(chess_interfaces__msg__PieceDetection__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = chess_interfaces__msg__PieceDetection__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
chess_interfaces__msg__PieceDetection__Sequence__destroy(chess_interfaces__msg__PieceDetection__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    chess_interfaces__msg__PieceDetection__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
chess_interfaces__msg__PieceDetection__Sequence__are_equal(const chess_interfaces__msg__PieceDetection__Sequence * lhs, const chess_interfaces__msg__PieceDetection__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!chess_interfaces__msg__PieceDetection__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
chess_interfaces__msg__PieceDetection__Sequence__copy(
  const chess_interfaces__msg__PieceDetection__Sequence * input,
  chess_interfaces__msg__PieceDetection__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(chess_interfaces__msg__PieceDetection);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    chess_interfaces__msg__PieceDetection * data =
      (chess_interfaces__msg__PieceDetection *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!chess_interfaces__msg__PieceDetection__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          chess_interfaces__msg__PieceDetection__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!chess_interfaces__msg__PieceDetection__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
