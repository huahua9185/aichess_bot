// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from chess_interfaces:msg/BoardState.idl
// generated code does not contain a copyright notice

#ifndef CHESS_INTERFACES__MSG__DETAIL__BOARD_STATE__TRAITS_HPP_
#define CHESS_INTERFACES__MSG__DETAIL__BOARD_STATE__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "chess_interfaces/msg/detail/board_state__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__traits.hpp"
// Member 'square_positions_3d'
#include "geometry_msgs/msg/detail/point__traits.hpp"

namespace chess_interfaces
{

namespace msg
{

inline void to_flow_style_yaml(
  const BoardState & msg,
  std::ostream & out)
{
  out << "{";
  // member: header
  {
    out << "header: ";
    to_flow_style_yaml(msg.header, out);
    out << ", ";
  }

  // member: board_squares
  {
    if (msg.board_squares.size() == 0) {
      out << "board_squares: []";
    } else {
      out << "board_squares: [";
      size_t pending_items = msg.board_squares.size();
      for (auto item : msg.board_squares) {
        rosidl_generator_traits::value_to_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
    out << ", ";
  }

  // member: square_positions_3d
  {
    if (msg.square_positions_3d.size() == 0) {
      out << "square_positions_3d: []";
    } else {
      out << "square_positions_3d: [";
      size_t pending_items = msg.square_positions_3d.size();
      for (auto item : msg.square_positions_3d) {
        to_flow_style_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
    out << ", ";
  }

  // member: white_to_move
  {
    out << "white_to_move: ";
    rosidl_generator_traits::value_to_yaml(msg.white_to_move, out);
    out << ", ";
  }

  // member: castling_rights
  {
    if (msg.castling_rights.size() == 0) {
      out << "castling_rights: []";
    } else {
      out << "castling_rights: [";
      size_t pending_items = msg.castling_rights.size();
      for (auto item : msg.castling_rights) {
        rosidl_generator_traits::value_to_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
    out << ", ";
  }

  // member: en_passant_square
  {
    out << "en_passant_square: ";
    rosidl_generator_traits::value_to_yaml(msg.en_passant_square, out);
    out << ", ";
  }

  // member: halfmove_clock
  {
    out << "halfmove_clock: ";
    rosidl_generator_traits::value_to_yaml(msg.halfmove_clock, out);
    out << ", ";
  }

  // member: fullmove_number
  {
    out << "fullmove_number: ";
    rosidl_generator_traits::value_to_yaml(msg.fullmove_number, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const BoardState & msg,
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

  // member: board_squares
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.board_squares.size() == 0) {
      out << "board_squares: []\n";
    } else {
      out << "board_squares:\n";
      for (auto item : msg.board_squares) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "- ";
        rosidl_generator_traits::value_to_yaml(item, out);
        out << "\n";
      }
    }
  }

  // member: square_positions_3d
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.square_positions_3d.size() == 0) {
      out << "square_positions_3d: []\n";
    } else {
      out << "square_positions_3d:\n";
      for (auto item : msg.square_positions_3d) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "-\n";
        to_block_style_yaml(item, out, indentation + 2);
      }
    }
  }

  // member: white_to_move
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "white_to_move: ";
    rosidl_generator_traits::value_to_yaml(msg.white_to_move, out);
    out << "\n";
  }

  // member: castling_rights
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.castling_rights.size() == 0) {
      out << "castling_rights: []\n";
    } else {
      out << "castling_rights:\n";
      for (auto item : msg.castling_rights) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "- ";
        rosidl_generator_traits::value_to_yaml(item, out);
        out << "\n";
      }
    }
  }

  // member: en_passant_square
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "en_passant_square: ";
    rosidl_generator_traits::value_to_yaml(msg.en_passant_square, out);
    out << "\n";
  }

  // member: halfmove_clock
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "halfmove_clock: ";
    rosidl_generator_traits::value_to_yaml(msg.halfmove_clock, out);
    out << "\n";
  }

  // member: fullmove_number
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "fullmove_number: ";
    rosidl_generator_traits::value_to_yaml(msg.fullmove_number, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const BoardState & msg, bool use_flow_style = false)
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
  const chess_interfaces::msg::BoardState & msg,
  std::ostream & out, size_t indentation = 0)
{
  chess_interfaces::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use chess_interfaces::msg::to_yaml() instead")]]
inline std::string to_yaml(const chess_interfaces::msg::BoardState & msg)
{
  return chess_interfaces::msg::to_yaml(msg);
}

template<>
inline const char * data_type<chess_interfaces::msg::BoardState>()
{
  return "chess_interfaces::msg::BoardState";
}

template<>
inline const char * name<chess_interfaces::msg::BoardState>()
{
  return "chess_interfaces/msg/BoardState";
}

template<>
struct has_fixed_size<chess_interfaces::msg::BoardState>
  : std::integral_constant<bool, has_fixed_size<geometry_msgs::msg::Point>::value && has_fixed_size<std_msgs::msg::Header>::value> {};

template<>
struct has_bounded_size<chess_interfaces::msg::BoardState>
  : std::integral_constant<bool, has_bounded_size<geometry_msgs::msg::Point>::value && has_bounded_size<std_msgs::msg::Header>::value> {};

template<>
struct is_message<chess_interfaces::msg::BoardState>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // CHESS_INTERFACES__MSG__DETAIL__BOARD_STATE__TRAITS_HPP_
