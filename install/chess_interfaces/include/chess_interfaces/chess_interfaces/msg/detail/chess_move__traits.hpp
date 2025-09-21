// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from chess_interfaces:msg/ChessMove.idl
// generated code does not contain a copyright notice

#ifndef CHESS_INTERFACES__MSG__DETAIL__CHESS_MOVE__TRAITS_HPP_
#define CHESS_INTERFACES__MSG__DETAIL__CHESS_MOVE__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "chess_interfaces/msg/detail/chess_move__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__traits.hpp"

namespace chess_interfaces
{

namespace msg
{

inline void to_flow_style_yaml(
  const ChessMove & msg,
  std::ostream & out)
{
  out << "{";
  // member: header
  {
    out << "header: ";
    to_flow_style_yaml(msg.header, out);
    out << ", ";
  }

  // member: from_square
  {
    out << "from_square: ";
    rosidl_generator_traits::value_to_yaml(msg.from_square, out);
    out << ", ";
  }

  // member: to_square
  {
    out << "to_square: ";
    rosidl_generator_traits::value_to_yaml(msg.to_square, out);
    out << ", ";
  }

  // member: piece_type
  {
    out << "piece_type: ";
    rosidl_generator_traits::value_to_yaml(msg.piece_type, out);
    out << ", ";
  }

  // member: captured_piece
  {
    out << "captured_piece: ";
    rosidl_generator_traits::value_to_yaml(msg.captured_piece, out);
    out << ", ";
  }

  // member: promotion
  {
    out << "promotion: ";
    rosidl_generator_traits::value_to_yaml(msg.promotion, out);
    out << ", ";
  }

  // member: is_castling
  {
    out << "is_castling: ";
    rosidl_generator_traits::value_to_yaml(msg.is_castling, out);
    out << ", ";
  }

  // member: is_en_passant
  {
    out << "is_en_passant: ";
    rosidl_generator_traits::value_to_yaml(msg.is_en_passant, out);
    out << ", ";
  }

  // member: is_check
  {
    out << "is_check: ";
    rosidl_generator_traits::value_to_yaml(msg.is_check, out);
    out << ", ";
  }

  // member: is_checkmate
  {
    out << "is_checkmate: ";
    rosidl_generator_traits::value_to_yaml(msg.is_checkmate, out);
    out << ", ";
  }

  // member: confidence
  {
    out << "confidence: ";
    rosidl_generator_traits::value_to_yaml(msg.confidence, out);
    out << ", ";
  }

  // member: thinking_time
  {
    out << "thinking_time: ";
    rosidl_generator_traits::value_to_yaml(msg.thinking_time, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const ChessMove & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: header
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "header:\n";
    to_block_style_yaml(msg.header, out, indentation + 2);
  }

  // member: from_square
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "from_square: ";
    rosidl_generator_traits::value_to_yaml(msg.from_square, out);
    out << "\n";
  }

  // member: to_square
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "to_square: ";
    rosidl_generator_traits::value_to_yaml(msg.to_square, out);
    out << "\n";
  }

  // member: piece_type
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "piece_type: ";
    rosidl_generator_traits::value_to_yaml(msg.piece_type, out);
    out << "\n";
  }

  // member: captured_piece
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "captured_piece: ";
    rosidl_generator_traits::value_to_yaml(msg.captured_piece, out);
    out << "\n";
  }

  // member: promotion
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "promotion: ";
    rosidl_generator_traits::value_to_yaml(msg.promotion, out);
    out << "\n";
  }

  // member: is_castling
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "is_castling: ";
    rosidl_generator_traits::value_to_yaml(msg.is_castling, out);
    out << "\n";
  }

  // member: is_en_passant
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "is_en_passant: ";
    rosidl_generator_traits::value_to_yaml(msg.is_en_passant, out);
    out << "\n";
  }

  // member: is_check
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "is_check: ";
    rosidl_generator_traits::value_to_yaml(msg.is_check, out);
    out << "\n";
  }

  // member: is_checkmate
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "is_checkmate: ";
    rosidl_generator_traits::value_to_yaml(msg.is_checkmate, out);
    out << "\n";
  }

  // member: confidence
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "confidence: ";
    rosidl_generator_traits::value_to_yaml(msg.confidence, out);
    out << "\n";
  }

  // member: thinking_time
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "thinking_time: ";
    rosidl_generator_traits::value_to_yaml(msg.thinking_time, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const ChessMove & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace msg

}  // namespace chess_interfaces

namespace rosidl_generator_traits
{

[[deprecated("use chess_interfaces::msg::to_block_style_yaml() instead")]]
inline void to_yaml(
  const chess_interfaces::msg::ChessMove & msg,
  std::ostream & out, size_t indentation = 0)
{
  chess_interfaces::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use chess_interfaces::msg::to_yaml() instead")]]
inline std::string to_yaml(const chess_interfaces::msg::ChessMove & msg)
{
  return chess_interfaces::msg::to_yaml(msg);
}

template<>
inline const char * data_type<chess_interfaces::msg::ChessMove>()
{
  return "chess_interfaces::msg::ChessMove";
}

template<>
inline const char * name<chess_interfaces::msg::ChessMove>()
{
  return "chess_interfaces/msg/ChessMove";
}

template<>
struct has_fixed_size<chess_interfaces::msg::ChessMove>
  : std::integral_constant<bool, has_fixed_size<std_msgs::msg::Header>::value> {};

template<>
struct has_bounded_size<chess_interfaces::msg::ChessMove>
  : std::integral_constant<bool, has_bounded_size<std_msgs::msg::Header>::value> {};

template<>
struct is_message<chess_interfaces::msg::ChessMove>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // CHESS_INTERFACES__MSG__DETAIL__CHESS_MOVE__TRAITS_HPP_
