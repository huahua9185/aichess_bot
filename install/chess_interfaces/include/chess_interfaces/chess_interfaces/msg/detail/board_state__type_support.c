// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from chess_interfaces:msg/BoardState.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "chess_interfaces/msg/detail/board_state__rosidl_typesupport_introspection_c.h"
#include "chess_interfaces/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "chess_interfaces/msg/detail/board_state__functions.h"
#include "chess_interfaces/msg/detail/board_state__struct.h"


// Include directives for member types
// Member `header`
#include "std_msgs/msg/header.h"
// Member `header`
#include "std_msgs/msg/detail/header__rosidl_typesupport_introspection_c.h"
// Member `square_positions_3d`
#include "geometry_msgs/msg/point.h"
// Member `square_positions_3d`
#include "geometry_msgs/msg/detail/point__rosidl_typesupport_introspection_c.h"

#ifdef __cplusplus
extern "C"
{
#endif

void chess_interfaces__msg__BoardState__rosidl_typesupport_introspection_c__BoardState_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  chess_interfaces__msg__BoardState__init(message_memory);
}

void chess_interfaces__msg__BoardState__rosidl_typesupport_introspection_c__BoardState_fini_function(void * message_memory)
{
  chess_interfaces__msg__BoardState__fini(message_memory);
}

size_t chess_interfaces__msg__BoardState__rosidl_typesupport_introspection_c__size_function__BoardState__board_squares(
  const void * untyped_member)
{
  (void)untyped_member;
  return 64;
}

const void * chess_interfaces__msg__BoardState__rosidl_typesupport_introspection_c__get_const_function__BoardState__board_squares(
  const void * untyped_member, size_t index)
{
  const int8_t * member =
    (const int8_t *)(untyped_member);
  return &member[index];
}

void * chess_interfaces__msg__BoardState__rosidl_typesupport_introspection_c__get_function__BoardState__board_squares(
  void * untyped_member, size_t index)
{
  int8_t * member =
    (int8_t *)(untyped_member);
  return &member[index];
}

void chess_interfaces__msg__BoardState__rosidl_typesupport_introspection_c__fetch_function__BoardState__board_squares(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const int8_t * item =
    ((const int8_t *)
    chess_interfaces__msg__BoardState__rosidl_typesupport_introspection_c__get_const_function__BoardState__board_squares(untyped_member, index));
  int8_t * value =
    (int8_t *)(untyped_value);
  *value = *item;
}

void chess_interfaces__msg__BoardState__rosidl_typesupport_introspection_c__assign_function__BoardState__board_squares(
  void * untyped_member, size_t index, const void * untyped_value)
{
  int8_t * item =
    ((int8_t *)
    chess_interfaces__msg__BoardState__rosidl_typesupport_introspection_c__get_function__BoardState__board_squares(untyped_member, index));
  const int8_t * value =
    (const int8_t *)(untyped_value);
  *item = *value;
}

size_t chess_interfaces__msg__BoardState__rosidl_typesupport_introspection_c__size_function__BoardState__square_positions_3d(
  const void * untyped_member)
{
  (void)untyped_member;
  return 64;
}

const void * chess_interfaces__msg__BoardState__rosidl_typesupport_introspection_c__get_const_function__BoardState__square_positions_3d(
  const void * untyped_member, size_t index)
{
  const geometry_msgs__msg__Point * member =
    (const geometry_msgs__msg__Point *)(untyped_member);
  return &member[index];
}

void * chess_interfaces__msg__BoardState__rosidl_typesupport_introspection_c__get_function__BoardState__square_positions_3d(
  void * untyped_member, size_t index)
{
  geometry_msgs__msg__Point * member =
    (geometry_msgs__msg__Point *)(untyped_member);
  return &member[index];
}

void chess_interfaces__msg__BoardState__rosidl_typesupport_introspection_c__fetch_function__BoardState__square_positions_3d(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const geometry_msgs__msg__Point * item =
    ((const geometry_msgs__msg__Point *)
    chess_interfaces__msg__BoardState__rosidl_typesupport_introspection_c__get_const_function__BoardState__square_positions_3d(untyped_member, index));
  geometry_msgs__msg__Point * value =
    (geometry_msgs__msg__Point *)(untyped_value);
  *value = *item;
}

void chess_interfaces__msg__BoardState__rosidl_typesupport_introspection_c__assign_function__BoardState__square_positions_3d(
  void * untyped_member, size_t index, const void * untyped_value)
{
  geometry_msgs__msg__Point * item =
    ((geometry_msgs__msg__Point *)
    chess_interfaces__msg__BoardState__rosidl_typesupport_introspection_c__get_function__BoardState__square_positions_3d(untyped_member, index));
  const geometry_msgs__msg__Point * value =
    (const geometry_msgs__msg__Point *)(untyped_value);
  *item = *value;
}

size_t chess_interfaces__msg__BoardState__rosidl_typesupport_introspection_c__size_function__BoardState__castling_rights(
  const void * untyped_member)
{
  (void)untyped_member;
  return 4;
}

const void * chess_interfaces__msg__BoardState__rosidl_typesupport_introspection_c__get_const_function__BoardState__castling_rights(
  const void * untyped_member, size_t index)
{
  const bool * member =
    (const bool *)(untyped_member);
  return &member[index];
}

void * chess_interfaces__msg__BoardState__rosidl_typesupport_introspection_c__get_function__BoardState__castling_rights(
  void * untyped_member, size_t index)
{
  bool * member =
    (bool *)(untyped_member);
  return &member[index];
}

void chess_interfaces__msg__BoardState__rosidl_typesupport_introspection_c__fetch_function__BoardState__castling_rights(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const bool * item =
    ((const bool *)
    chess_interfaces__msg__BoardState__rosidl_typesupport_introspection_c__get_const_function__BoardState__castling_rights(untyped_member, index));
  bool * value =
    (bool *)(untyped_value);
  *value = *item;
}

void chess_interfaces__msg__BoardState__rosidl_typesupport_introspection_c__assign_function__BoardState__castling_rights(
  void * untyped_member, size_t index, const void * untyped_value)
{
  bool * item =
    ((bool *)
    chess_interfaces__msg__BoardState__rosidl_typesupport_introspection_c__get_function__BoardState__castling_rights(untyped_member, index));
  const bool * value =
    (const bool *)(untyped_value);
  *item = *value;
}

static rosidl_typesupport_introspection_c__MessageMember chess_interfaces__msg__BoardState__rosidl_typesupport_introspection_c__BoardState_message_member_array[8] = {
  {
    "header",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(chess_interfaces__msg__BoardState, header),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "board_squares",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_INT8,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    true,  // is array
    64,  // array size
    false,  // is upper bound
    offsetof(chess_interfaces__msg__BoardState, board_squares),  // bytes offset in struct
    NULL,  // default value
    chess_interfaces__msg__BoardState__rosidl_typesupport_introspection_c__size_function__BoardState__board_squares,  // size() function pointer
    chess_interfaces__msg__BoardState__rosidl_typesupport_introspection_c__get_const_function__BoardState__board_squares,  // get_const(index) function pointer
    chess_interfaces__msg__BoardState__rosidl_typesupport_introspection_c__get_function__BoardState__board_squares,  // get(index) function pointer
    chess_interfaces__msg__BoardState__rosidl_typesupport_introspection_c__fetch_function__BoardState__board_squares,  // fetch(index, &value) function pointer
    chess_interfaces__msg__BoardState__rosidl_typesupport_introspection_c__assign_function__BoardState__board_squares,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "square_positions_3d",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    true,  // is array
    64,  // array size
    false,  // is upper bound
    offsetof(chess_interfaces__msg__BoardState, square_positions_3d),  // bytes offset in struct
    NULL,  // default value
    chess_interfaces__msg__BoardState__rosidl_typesupport_introspection_c__size_function__BoardState__square_positions_3d,  // size() function pointer
    chess_interfaces__msg__BoardState__rosidl_typesupport_introspection_c__get_const_function__BoardState__square_positions_3d,  // get_const(index) function pointer
    chess_interfaces__msg__BoardState__rosidl_typesupport_introspection_c__get_function__BoardState__square_positions_3d,  // get(index) function pointer
    chess_interfaces__msg__BoardState__rosidl_typesupport_introspection_c__fetch_function__BoardState__square_positions_3d,  // fetch(index, &value) function pointer
    chess_interfaces__msg__BoardState__rosidl_typesupport_introspection_c__assign_function__BoardState__square_positions_3d,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "white_to_move",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_BOOLEAN,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(chess_interfaces__msg__BoardState, white_to_move),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "castling_rights",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_BOOLEAN,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    true,  // is array
    4,  // array size
    false,  // is upper bound
    offsetof(chess_interfaces__msg__BoardState, castling_rights),  // bytes offset in struct
    NULL,  // default value
    chess_interfaces__msg__BoardState__rosidl_typesupport_introspection_c__size_function__BoardState__castling_rights,  // size() function pointer
    chess_interfaces__msg__BoardState__rosidl_typesupport_introspection_c__get_const_function__BoardState__castling_rights,  // get_const(index) function pointer
    chess_interfaces__msg__BoardState__rosidl_typesupport_introspection_c__get_function__BoardState__castling_rights,  // get(index) function pointer
    chess_interfaces__msg__BoardState__rosidl_typesupport_introspection_c__fetch_function__BoardState__castling_rights,  // fetch(index, &value) function pointer
    chess_interfaces__msg__BoardState__rosidl_typesupport_introspection_c__assign_function__BoardState__castling_rights,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "en_passant_square",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_INT8,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(chess_interfaces__msg__BoardState, en_passant_square),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "halfmove_clock",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_INT32,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(chess_interfaces__msg__BoardState, halfmove_clock),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "fullmove_number",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_INT32,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(chess_interfaces__msg__BoardState, fullmove_number),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers chess_interfaces__msg__BoardState__rosidl_typesupport_introspection_c__BoardState_message_members = {
  "chess_interfaces__msg",  // message namespace
  "BoardState",  // message name
  8,  // number of fields
  sizeof(chess_interfaces__msg__BoardState),
  chess_interfaces__msg__BoardState__rosidl_typesupport_introspection_c__BoardState_message_member_array,  // message members
  chess_interfaces__msg__BoardState__rosidl_typesupport_introspection_c__BoardState_init_function,  // function to initialize message memory (memory has to be allocated)
  chess_interfaces__msg__BoardState__rosidl_typesupport_introspection_c__BoardState_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t chess_interfaces__msg__BoardState__rosidl_typesupport_introspection_c__BoardState_message_type_support_handle = {
  0,
  &chess_interfaces__msg__BoardState__rosidl_typesupport_introspection_c__BoardState_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_chess_interfaces
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, chess_interfaces, msg, BoardState)() {
  chess_interfaces__msg__BoardState__rosidl_typesupport_introspection_c__BoardState_message_member_array[0].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, std_msgs, msg, Header)();
  chess_interfaces__msg__BoardState__rosidl_typesupport_introspection_c__BoardState_message_member_array[2].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, geometry_msgs, msg, Point)();
  if (!chess_interfaces__msg__BoardState__rosidl_typesupport_introspection_c__BoardState_message_type_support_handle.typesupport_identifier) {
    chess_interfaces__msg__BoardState__rosidl_typesupport_introspection_c__BoardState_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &chess_interfaces__msg__BoardState__rosidl_typesupport_introspection_c__BoardState_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif
