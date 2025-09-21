// generated from rosidl_typesupport_fastrtps_c/resource/idl__type_support_c.cpp.em
// with input from chess_interfaces:msg/ChessMove.idl
// generated code does not contain a copyright notice
#include "chess_interfaces/msg/detail/chess_move__rosidl_typesupport_fastrtps_c.h"


#include <cassert>
#include <limits>
#include <string>
#include "rosidl_typesupport_fastrtps_c/identifier.h"
#include "rosidl_typesupport_fastrtps_c/wstring_conversion.hpp"
#include "rosidl_typesupport_fastrtps_cpp/message_type_support.h"
#include "chess_interfaces/msg/rosidl_typesupport_fastrtps_c__visibility_control.h"
#include "chess_interfaces/msg/detail/chess_move__struct.h"
#include "chess_interfaces/msg/detail/chess_move__functions.h"
#include "fastcdr/Cdr.h"

#ifndef _WIN32
# pragma GCC diagnostic push
# pragma GCC diagnostic ignored "-Wunused-parameter"
# ifdef __clang__
#  pragma clang diagnostic ignored "-Wdeprecated-register"
#  pragma clang diagnostic ignored "-Wreturn-type-c-linkage"
# endif
#endif
#ifndef _WIN32
# pragma GCC diagnostic pop
#endif

// includes and forward declarations of message dependencies and their conversion functions

#if defined(__cplusplus)
extern "C"
{
#endif

#include "std_msgs/msg/detail/header__functions.h"  // header

// forward declare type support functions
ROSIDL_TYPESUPPORT_FASTRTPS_C_IMPORT_chess_interfaces
size_t get_serialized_size_std_msgs__msg__Header(
  const void * untyped_ros_message,
  size_t current_alignment);

ROSIDL_TYPESUPPORT_FASTRTPS_C_IMPORT_chess_interfaces
size_t max_serialized_size_std_msgs__msg__Header(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment);

ROSIDL_TYPESUPPORT_FASTRTPS_C_IMPORT_chess_interfaces
const rosidl_message_type_support_t *
  ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, std_msgs, msg, Header)();


using _ChessMove__ros_msg_type = chess_interfaces__msg__ChessMove;

static bool _ChessMove__cdr_serialize(
  const void * untyped_ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  if (!untyped_ros_message) {
    fprintf(stderr, "ros message handle is null\n");
    return false;
  }
  const _ChessMove__ros_msg_type * ros_message = static_cast<const _ChessMove__ros_msg_type *>(untyped_ros_message);
  // Field name: header
  {
    const message_type_support_callbacks_t * callbacks =
      static_cast<const message_type_support_callbacks_t *>(
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(
        rosidl_typesupport_fastrtps_c, std_msgs, msg, Header
      )()->data);
    if (!callbacks->cdr_serialize(
        &ros_message->header, cdr))
    {
      return false;
    }
  }

  // Field name: from_square
  {
    cdr << ros_message->from_square;
  }

  // Field name: to_square
  {
    cdr << ros_message->to_square;
  }

  // Field name: piece_type
  {
    cdr << ros_message->piece_type;
  }

  // Field name: captured_piece
  {
    cdr << ros_message->captured_piece;
  }

  // Field name: promotion
  {
    cdr << ros_message->promotion;
  }

  // Field name: is_castling
  {
    cdr << (ros_message->is_castling ? true : false);
  }

  // Field name: is_en_passant
  {
    cdr << (ros_message->is_en_passant ? true : false);
  }

  // Field name: is_check
  {
    cdr << (ros_message->is_check ? true : false);
  }

  // Field name: is_checkmate
  {
    cdr << (ros_message->is_checkmate ? true : false);
  }

  // Field name: confidence
  {
    cdr << ros_message->confidence;
  }

  // Field name: thinking_time
  {
    cdr << ros_message->thinking_time;
  }

  return true;
}

static bool _ChessMove__cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  void * untyped_ros_message)
{
  if (!untyped_ros_message) {
    fprintf(stderr, "ros message handle is null\n");
    return false;
  }
  _ChessMove__ros_msg_type * ros_message = static_cast<_ChessMove__ros_msg_type *>(untyped_ros_message);
  // Field name: header
  {
    const message_type_support_callbacks_t * callbacks =
      static_cast<const message_type_support_callbacks_t *>(
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(
        rosidl_typesupport_fastrtps_c, std_msgs, msg, Header
      )()->data);
    if (!callbacks->cdr_deserialize(
        cdr, &ros_message->header))
    {
      return false;
    }
  }

  // Field name: from_square
  {
    cdr >> ros_message->from_square;
  }

  // Field name: to_square
  {
    cdr >> ros_message->to_square;
  }

  // Field name: piece_type
  {
    cdr >> ros_message->piece_type;
  }

  // Field name: captured_piece
  {
    cdr >> ros_message->captured_piece;
  }

  // Field name: promotion
  {
    cdr >> ros_message->promotion;
  }

  // Field name: is_castling
  {
    uint8_t tmp;
    cdr >> tmp;
    ros_message->is_castling = tmp ? true : false;
  }

  // Field name: is_en_passant
  {
    uint8_t tmp;
    cdr >> tmp;
    ros_message->is_en_passant = tmp ? true : false;
  }

  // Field name: is_check
  {
    uint8_t tmp;
    cdr >> tmp;
    ros_message->is_check = tmp ? true : false;
  }

  // Field name: is_checkmate
  {
    uint8_t tmp;
    cdr >> tmp;
    ros_message->is_checkmate = tmp ? true : false;
  }

  // Field name: confidence
  {
    cdr >> ros_message->confidence;
  }

  // Field name: thinking_time
  {
    cdr >> ros_message->thinking_time;
  }

  return true;
}  // NOLINT(readability/fn_size)

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_chess_interfaces
size_t get_serialized_size_chess_interfaces__msg__ChessMove(
  const void * untyped_ros_message,
  size_t current_alignment)
{
  const _ChessMove__ros_msg_type * ros_message = static_cast<const _ChessMove__ros_msg_type *>(untyped_ros_message);
  (void)ros_message;
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;

  // field.name header

  current_alignment += get_serialized_size_std_msgs__msg__Header(
    &(ros_message->header), current_alignment);
  // field.name from_square
  {
    size_t item_size = sizeof(ros_message->from_square);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name to_square
  {
    size_t item_size = sizeof(ros_message->to_square);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name piece_type
  {
    size_t item_size = sizeof(ros_message->piece_type);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name captured_piece
  {
    size_t item_size = sizeof(ros_message->captured_piece);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name promotion
  {
    size_t item_size = sizeof(ros_message->promotion);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name is_castling
  {
    size_t item_size = sizeof(ros_message->is_castling);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name is_en_passant
  {
    size_t item_size = sizeof(ros_message->is_en_passant);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name is_check
  {
    size_t item_size = sizeof(ros_message->is_check);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name is_checkmate
  {
    size_t item_size = sizeof(ros_message->is_checkmate);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name confidence
  {
    size_t item_size = sizeof(ros_message->confidence);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name thinking_time
  {
    size_t item_size = sizeof(ros_message->thinking_time);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  return current_alignment - initial_alignment;
}

static uint32_t _ChessMove__get_serialized_size(const void * untyped_ros_message)
{
  return static_cast<uint32_t>(
    get_serialized_size_chess_interfaces__msg__ChessMove(
      untyped_ros_message, 0));
}

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_chess_interfaces
size_t max_serialized_size_chess_interfaces__msg__ChessMove(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment)
{
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  size_t last_member_size = 0;
  (void)last_member_size;
  (void)padding;
  (void)wchar_size;

  full_bounded = true;
  is_plain = true;

  // member: header
  {
    size_t array_size = 1;


    last_member_size = 0;
    for (size_t index = 0; index < array_size; ++index) {
      bool inner_full_bounded;
      bool inner_is_plain;
      size_t inner_size;
      inner_size =
        max_serialized_size_std_msgs__msg__Header(
        inner_full_bounded, inner_is_plain, current_alignment);
      last_member_size += inner_size;
      current_alignment += inner_size;
      full_bounded &= inner_full_bounded;
      is_plain &= inner_is_plain;
    }
  }
  // member: from_square
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }
  // member: to_square
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }
  // member: piece_type
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }
  // member: captured_piece
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }
  // member: promotion
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }
  // member: is_castling
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }
  // member: is_en_passant
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }
  // member: is_check
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }
  // member: is_checkmate
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }
  // member: confidence
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }
  // member: thinking_time
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  size_t ret_val = current_alignment - initial_alignment;
  if (is_plain) {
    // All members are plain, and type is not empty.
    // We still need to check that the in-memory alignment
    // is the same as the CDR mandated alignment.
    using DataType = chess_interfaces__msg__ChessMove;
    is_plain =
      (
      offsetof(DataType, thinking_time) +
      last_member_size
      ) == ret_val;
  }

  return ret_val;
}

static size_t _ChessMove__max_serialized_size(char & bounds_info)
{
  bool full_bounded;
  bool is_plain;
  size_t ret_val;

  ret_val = max_serialized_size_chess_interfaces__msg__ChessMove(
    full_bounded, is_plain, 0);

  bounds_info =
    is_plain ? ROSIDL_TYPESUPPORT_FASTRTPS_PLAIN_TYPE :
    full_bounded ? ROSIDL_TYPESUPPORT_FASTRTPS_BOUNDED_TYPE : ROSIDL_TYPESUPPORT_FASTRTPS_UNBOUNDED_TYPE;
  return ret_val;
}


static message_type_support_callbacks_t __callbacks_ChessMove = {
  "chess_interfaces::msg",
  "ChessMove",
  _ChessMove__cdr_serialize,
  _ChessMove__cdr_deserialize,
  _ChessMove__get_serialized_size,
  _ChessMove__max_serialized_size
};

static rosidl_message_type_support_t _ChessMove__type_support = {
  rosidl_typesupport_fastrtps_c__identifier,
  &__callbacks_ChessMove,
  get_message_typesupport_handle_function,
};

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, chess_interfaces, msg, ChessMove)() {
  return &_ChessMove__type_support;
}

#if defined(__cplusplus)
}
#endif
