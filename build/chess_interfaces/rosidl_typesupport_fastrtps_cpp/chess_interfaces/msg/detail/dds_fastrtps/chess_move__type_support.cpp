// generated from rosidl_typesupport_fastrtps_cpp/resource/idl__type_support.cpp.em
// with input from chess_interfaces:msg/ChessMove.idl
// generated code does not contain a copyright notice
#include "chess_interfaces/msg/detail/chess_move__rosidl_typesupport_fastrtps_cpp.hpp"
#include "chess_interfaces/msg/detail/chess_move__struct.hpp"

#include <limits>
#include <stdexcept>
#include <string>
#include "rosidl_typesupport_cpp/message_type_support.hpp"
#include "rosidl_typesupport_fastrtps_cpp/identifier.hpp"
#include "rosidl_typesupport_fastrtps_cpp/message_type_support.h"
#include "rosidl_typesupport_fastrtps_cpp/message_type_support_decl.hpp"
#include "rosidl_typesupport_fastrtps_cpp/wstring_conversion.hpp"
#include "fastcdr/Cdr.h"


// forward declaration of message dependencies and their conversion functions
namespace std_msgs
{
namespace msg
{
namespace typesupport_fastrtps_cpp
{
bool cdr_serialize(
  const std_msgs::msg::Header &,
  eprosima::fastcdr::Cdr &);
bool cdr_deserialize(
  eprosima::fastcdr::Cdr &,
  std_msgs::msg::Header &);
size_t get_serialized_size(
  const std_msgs::msg::Header &,
  size_t current_alignment);
size_t
max_serialized_size_Header(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment);
}  // namespace typesupport_fastrtps_cpp
}  // namespace msg
}  // namespace std_msgs


namespace chess_interfaces
{

namespace msg
{

namespace typesupport_fastrtps_cpp
{

bool
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_chess_interfaces
cdr_serialize(
  const chess_interfaces::msg::ChessMove & ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  // Member: header
  std_msgs::msg::typesupport_fastrtps_cpp::cdr_serialize(
    ros_message.header,
    cdr);
  // Member: from_square
  cdr << ros_message.from_square;
  // Member: to_square
  cdr << ros_message.to_square;
  // Member: piece_type
  cdr << ros_message.piece_type;
  // Member: captured_piece
  cdr << ros_message.captured_piece;
  // Member: promotion
  cdr << ros_message.promotion;
  // Member: is_castling
  cdr << (ros_message.is_castling ? true : false);
  // Member: is_en_passant
  cdr << (ros_message.is_en_passant ? true : false);
  // Member: is_check
  cdr << (ros_message.is_check ? true : false);
  // Member: is_checkmate
  cdr << (ros_message.is_checkmate ? true : false);
  // Member: confidence
  cdr << ros_message.confidence;
  // Member: thinking_time
  cdr << ros_message.thinking_time;
  return true;
}

bool
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_chess_interfaces
cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  chess_interfaces::msg::ChessMove & ros_message)
{
  // Member: header
  std_msgs::msg::typesupport_fastrtps_cpp::cdr_deserialize(
    cdr, ros_message.header);

  // Member: from_square
  cdr >> ros_message.from_square;

  // Member: to_square
  cdr >> ros_message.to_square;

  // Member: piece_type
  cdr >> ros_message.piece_type;

  // Member: captured_piece
  cdr >> ros_message.captured_piece;

  // Member: promotion
  cdr >> ros_message.promotion;

  // Member: is_castling
  {
    uint8_t tmp;
    cdr >> tmp;
    ros_message.is_castling = tmp ? true : false;
  }

  // Member: is_en_passant
  {
    uint8_t tmp;
    cdr >> tmp;
    ros_message.is_en_passant = tmp ? true : false;
  }

  // Member: is_check
  {
    uint8_t tmp;
    cdr >> tmp;
    ros_message.is_check = tmp ? true : false;
  }

  // Member: is_checkmate
  {
    uint8_t tmp;
    cdr >> tmp;
    ros_message.is_checkmate = tmp ? true : false;
  }

  // Member: confidence
  cdr >> ros_message.confidence;

  // Member: thinking_time
  cdr >> ros_message.thinking_time;

  return true;
}

size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_chess_interfaces
get_serialized_size(
  const chess_interfaces::msg::ChessMove & ros_message,
  size_t current_alignment)
{
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;

  // Member: header

  current_alignment +=
    std_msgs::msg::typesupport_fastrtps_cpp::get_serialized_size(
    ros_message.header, current_alignment);
  // Member: from_square
  {
    size_t item_size = sizeof(ros_message.from_square);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: to_square
  {
    size_t item_size = sizeof(ros_message.to_square);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: piece_type
  {
    size_t item_size = sizeof(ros_message.piece_type);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: captured_piece
  {
    size_t item_size = sizeof(ros_message.captured_piece);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: promotion
  {
    size_t item_size = sizeof(ros_message.promotion);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: is_castling
  {
    size_t item_size = sizeof(ros_message.is_castling);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: is_en_passant
  {
    size_t item_size = sizeof(ros_message.is_en_passant);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: is_check
  {
    size_t item_size = sizeof(ros_message.is_check);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: is_checkmate
  {
    size_t item_size = sizeof(ros_message.is_checkmate);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: confidence
  {
    size_t item_size = sizeof(ros_message.confidence);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: thinking_time
  {
    size_t item_size = sizeof(ros_message.thinking_time);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  return current_alignment - initial_alignment;
}

size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_chess_interfaces
max_serialized_size_ChessMove(
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


  // Member: header
  {
    size_t array_size = 1;


    last_member_size = 0;
    for (size_t index = 0; index < array_size; ++index) {
      bool inner_full_bounded;
      bool inner_is_plain;
      size_t inner_size =
        std_msgs::msg::typesupport_fastrtps_cpp::max_serialized_size_Header(
        inner_full_bounded, inner_is_plain, current_alignment);
      last_member_size += inner_size;
      current_alignment += inner_size;
      full_bounded &= inner_full_bounded;
      is_plain &= inner_is_plain;
    }
  }

  // Member: from_square
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }

  // Member: to_square
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }

  // Member: piece_type
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }

  // Member: captured_piece
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }

  // Member: promotion
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }

  // Member: is_castling
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }

  // Member: is_en_passant
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }

  // Member: is_check
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }

  // Member: is_checkmate
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }

  // Member: confidence
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  // Member: thinking_time
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
    using DataType = chess_interfaces::msg::ChessMove;
    is_plain =
      (
      offsetof(DataType, thinking_time) +
      last_member_size
      ) == ret_val;
  }

  return ret_val;
}

static bool _ChessMove__cdr_serialize(
  const void * untyped_ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  auto typed_message =
    static_cast<const chess_interfaces::msg::ChessMove *>(
    untyped_ros_message);
  return cdr_serialize(*typed_message, cdr);
}

static bool _ChessMove__cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  void * untyped_ros_message)
{
  auto typed_message =
    static_cast<chess_interfaces::msg::ChessMove *>(
    untyped_ros_message);
  return cdr_deserialize(cdr, *typed_message);
}

static uint32_t _ChessMove__get_serialized_size(
  const void * untyped_ros_message)
{
  auto typed_message =
    static_cast<const chess_interfaces::msg::ChessMove *>(
    untyped_ros_message);
  return static_cast<uint32_t>(get_serialized_size(*typed_message, 0));
}

static size_t _ChessMove__max_serialized_size(char & bounds_info)
{
  bool full_bounded;
  bool is_plain;
  size_t ret_val;

  ret_val = max_serialized_size_ChessMove(full_bounded, is_plain, 0);

  bounds_info =
    is_plain ? ROSIDL_TYPESUPPORT_FASTRTPS_PLAIN_TYPE :
    full_bounded ? ROSIDL_TYPESUPPORT_FASTRTPS_BOUNDED_TYPE : ROSIDL_TYPESUPPORT_FASTRTPS_UNBOUNDED_TYPE;
  return ret_val;
}

static message_type_support_callbacks_t _ChessMove__callbacks = {
  "chess_interfaces::msg",
  "ChessMove",
  _ChessMove__cdr_serialize,
  _ChessMove__cdr_deserialize,
  _ChessMove__get_serialized_size,
  _ChessMove__max_serialized_size
};

static rosidl_message_type_support_t _ChessMove__handle = {
  rosidl_typesupport_fastrtps_cpp::typesupport_identifier,
  &_ChessMove__callbacks,
  get_message_typesupport_handle_function,
};

}  // namespace typesupport_fastrtps_cpp

}  // namespace msg

}  // namespace chess_interfaces

namespace rosidl_typesupport_fastrtps_cpp
{

template<>
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_EXPORT_chess_interfaces
const rosidl_message_type_support_t *
get_message_type_support_handle<chess_interfaces::msg::ChessMove>()
{
  return &chess_interfaces::msg::typesupport_fastrtps_cpp::_ChessMove__handle;
}

}  // namespace rosidl_typesupport_fastrtps_cpp

#ifdef __cplusplus
extern "C"
{
#endif

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_cpp, chess_interfaces, msg, ChessMove)() {
  return &chess_interfaces::msg::typesupport_fastrtps_cpp::_ChessMove__handle;
}

#ifdef __cplusplus
}
#endif
