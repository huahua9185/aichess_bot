// generated from rosidl_typesupport_introspection_cpp/resource/idl__type_support.cpp.em
// with input from chess_interfaces:msg/BoardState.idl
// generated code does not contain a copyright notice

#include "array"
#include "cstddef"
#include "string"
#include "vector"
#include "rosidl_runtime_c/message_type_support_struct.h"
#include "rosidl_typesupport_cpp/message_type_support.hpp"
#include "rosidl_typesupport_interface/macros.h"
#include "chess_interfaces/msg/detail/board_state__struct.hpp"
#include "rosidl_typesupport_introspection_cpp/field_types.hpp"
#include "rosidl_typesupport_introspection_cpp/identifier.hpp"
#include "rosidl_typesupport_introspection_cpp/message_introspection.hpp"
#include "rosidl_typesupport_introspection_cpp/message_type_support_decl.hpp"
#include "rosidl_typesupport_introspection_cpp/visibility_control.h"

namespace chess_interfaces
{

namespace msg
{

namespace rosidl_typesupport_introspection_cpp
{

void BoardState_init_function(
  void * message_memory, rosidl_runtime_cpp::MessageInitialization _init)
{
  new (message_memory) chess_interfaces::msg::BoardState(_init);
}

void BoardState_fini_function(void * message_memory)
{
  auto typed_message = static_cast<chess_interfaces::msg::BoardState *>(message_memory);
  typed_message->~BoardState();
}

size_t size_function__BoardState__board_squares(const void * untyped_member)
{
  (void)untyped_member;
  return 64;
}

const void * get_const_function__BoardState__board_squares(const void * untyped_member, size_t index)
{
  const auto & member =
    *reinterpret_cast<const std::array<int8_t, 64> *>(untyped_member);
  return &member[index];
}

void * get_function__BoardState__board_squares(void * untyped_member, size_t index)
{
  auto & member =
    *reinterpret_cast<std::array<int8_t, 64> *>(untyped_member);
  return &member[index];
}

void fetch_function__BoardState__board_squares(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const auto & item = *reinterpret_cast<const int8_t *>(
    get_const_function__BoardState__board_squares(untyped_member, index));
  auto & value = *reinterpret_cast<int8_t *>(untyped_value);
  value = item;
}

void assign_function__BoardState__board_squares(
  void * untyped_member, size_t index, const void * untyped_value)
{
  auto & item = *reinterpret_cast<int8_t *>(
    get_function__BoardState__board_squares(untyped_member, index));
  const auto & value = *reinterpret_cast<const int8_t *>(untyped_value);
  item = value;
}

size_t size_function__BoardState__square_positions_3d(const void * untyped_member)
{
  (void)untyped_member;
  return 64;
}

const void * get_const_function__BoardState__square_positions_3d(const void * untyped_member, size_t index)
{
  const auto & member =
    *reinterpret_cast<const std::array<geometry_msgs::msg::Point, 64> *>(untyped_member);
  return &member[index];
}

void * get_function__BoardState__square_positions_3d(void * untyped_member, size_t index)
{
  auto & member =
    *reinterpret_cast<std::array<geometry_msgs::msg::Point, 64> *>(untyped_member);
  return &member[index];
}

void fetch_function__BoardState__square_positions_3d(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const auto & item = *reinterpret_cast<const geometry_msgs::msg::Point *>(
    get_const_function__BoardState__square_positions_3d(untyped_member, index));
  auto & value = *reinterpret_cast<geometry_msgs::msg::Point *>(untyped_value);
  value = item;
}

void assign_function__BoardState__square_positions_3d(
  void * untyped_member, size_t index, const void * untyped_value)
{
  auto & item = *reinterpret_cast<geometry_msgs::msg::Point *>(
    get_function__BoardState__square_positions_3d(untyped_member, index));
  const auto & value = *reinterpret_cast<const geometry_msgs::msg::Point *>(untyped_value);
  item = value;
}

size_t size_function__BoardState__castling_rights(const void * untyped_member)
{
  (void)untyped_member;
  return 4;
}

const void * get_const_function__BoardState__castling_rights(const void * untyped_member, size_t index)
{
  const auto & member =
    *reinterpret_cast<const std::array<bool, 4> *>(untyped_member);
  return &member[index];
}

void * get_function__BoardState__castling_rights(void * untyped_member, size_t index)
{
  auto & member =
    *reinterpret_cast<std::array<bool, 4> *>(untyped_member);
  return &member[index];
}

void fetch_function__BoardState__castling_rights(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const auto & item = *reinterpret_cast<const bool *>(
    get_const_function__BoardState__castling_rights(untyped_member, index));
  auto & value = *reinterpret_cast<bool *>(untyped_value);
  value = item;
}

void assign_function__BoardState__castling_rights(
  void * untyped_member, size_t index, const void * untyped_value)
{
  auto & item = *reinterpret_cast<bool *>(
    get_function__BoardState__castling_rights(untyped_member, index));
  const auto & value = *reinterpret_cast<const bool *>(untyped_value);
  item = value;
}

static const ::rosidl_typesupport_introspection_cpp::MessageMember BoardState_message_member_array[8] = {
  {
    "header",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    ::rosidl_typesupport_introspection_cpp::get_message_type_support_handle<std_msgs::msg::Header>(),  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(chess_interfaces::msg::BoardState, header),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr,  // fetch(index, &value) function pointer
    nullptr,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "board_squares",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_INT8,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    true,  // is array
    64,  // array size
    false,  // is upper bound
    offsetof(chess_interfaces::msg::BoardState, board_squares),  // bytes offset in struct
    nullptr,  // default value
    size_function__BoardState__board_squares,  // size() function pointer
    get_const_function__BoardState__board_squares,  // get_const(index) function pointer
    get_function__BoardState__board_squares,  // get(index) function pointer
    fetch_function__BoardState__board_squares,  // fetch(index, &value) function pointer
    assign_function__BoardState__board_squares,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "square_positions_3d",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    ::rosidl_typesupport_introspection_cpp::get_message_type_support_handle<geometry_msgs::msg::Point>(),  // members of sub message
    true,  // is array
    64,  // array size
    false,  // is upper bound
    offsetof(chess_interfaces::msg::BoardState, square_positions_3d),  // bytes offset in struct
    nullptr,  // default value
    size_function__BoardState__square_positions_3d,  // size() function pointer
    get_const_function__BoardState__square_positions_3d,  // get_const(index) function pointer
    get_function__BoardState__square_positions_3d,  // get(index) function pointer
    fetch_function__BoardState__square_positions_3d,  // fetch(index, &value) function pointer
    assign_function__BoardState__square_positions_3d,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "white_to_move",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_BOOLEAN,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(chess_interfaces::msg::BoardState, white_to_move),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr,  // fetch(index, &value) function pointer
    nullptr,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "castling_rights",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_BOOLEAN,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    true,  // is array
    4,  // array size
    false,  // is upper bound
    offsetof(chess_interfaces::msg::BoardState, castling_rights),  // bytes offset in struct
    nullptr,  // default value
    size_function__BoardState__castling_rights,  // size() function pointer
    get_const_function__BoardState__castling_rights,  // get_const(index) function pointer
    get_function__BoardState__castling_rights,  // get(index) function pointer
    fetch_function__BoardState__castling_rights,  // fetch(index, &value) function pointer
    assign_function__BoardState__castling_rights,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "en_passant_square",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_INT8,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(chess_interfaces::msg::BoardState, en_passant_square),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr,  // fetch(index, &value) function pointer
    nullptr,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "halfmove_clock",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_INT32,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(chess_interfaces::msg::BoardState, halfmove_clock),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr,  // fetch(index, &value) function pointer
    nullptr,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "fullmove_number",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_INT32,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(chess_interfaces::msg::BoardState, fullmove_number),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr,  // fetch(index, &value) function pointer
    nullptr,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  }
};

static const ::rosidl_typesupport_introspection_cpp::MessageMembers BoardState_message_members = {
  "chess_interfaces::msg",  // message namespace
  "BoardState",  // message name
  8,  // number of fields
  sizeof(chess_interfaces::msg::BoardState),
  BoardState_message_member_array,  // message members
  BoardState_init_function,  // function to initialize message memory (memory has to be allocated)
  BoardState_fini_function  // function to terminate message instance (will not free memory)
};

static const rosidl_message_type_support_t BoardState_message_type_support_handle = {
  ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  &BoardState_message_members,
  get_message_typesupport_handle_function,
};

}  // namespace rosidl_typesupport_introspection_cpp

}  // namespace msg

}  // namespace chess_interfaces


namespace rosidl_typesupport_introspection_cpp
{

template<>
ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
get_message_type_support_handle<chess_interfaces::msg::BoardState>()
{
  return &::chess_interfaces::msg::rosidl_typesupport_introspection_cpp::BoardState_message_type_support_handle;
}

}  // namespace rosidl_typesupport_introspection_cpp

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, chess_interfaces, msg, BoardState)() {
  return &::chess_interfaces::msg::rosidl_typesupport_introspection_cpp::BoardState_message_type_support_handle;
}

#ifdef __cplusplus
}
#endif
