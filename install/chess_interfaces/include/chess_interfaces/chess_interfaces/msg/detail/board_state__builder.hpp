// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from chess_interfaces:msg/BoardState.idl
// generated code does not contain a copyright notice

#ifndef CHESS_INTERFACES__MSG__DETAIL__BOARD_STATE__BUILDER_HPP_
#define CHESS_INTERFACES__MSG__DETAIL__BOARD_STATE__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "chess_interfaces/msg/detail/board_state__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace chess_interfaces
{

namespace msg
{

namespace builder
{

class Init_BoardState_fullmove_number
{
public:
  explicit Init_BoardState_fullmove_number(::chess_interfaces::msg::BoardState & msg)
  : msg_(msg)
  {}
  ::chess_interfaces::msg::BoardState fullmove_number(::chess_interfaces::msg::BoardState::_fullmove_number_type arg)
  {
    msg_.fullmove_number = std::move(arg);
    return std::move(msg_);
  }

private:
  ::chess_interfaces::msg::BoardState msg_;
};

class Init_BoardState_halfmove_clock
{
public:
  explicit Init_BoardState_halfmove_clock(::chess_interfaces::msg::BoardState & msg)
  : msg_(msg)
  {}
  Init_BoardState_fullmove_number halfmove_clock(::chess_interfaces::msg::BoardState::_halfmove_clock_type arg)
  {
    msg_.halfmove_clock = std::move(arg);
    return Init_BoardState_fullmove_number(msg_);
  }

private:
  ::chess_interfaces::msg::BoardState msg_;
};

class Init_BoardState_en_passant_square
{
public:
  explicit Init_BoardState_en_passant_square(::chess_interfaces::msg::BoardState & msg)
  : msg_(msg)
  {}
  Init_BoardState_halfmove_clock en_passant_square(::chess_interfaces::msg::BoardState::_en_passant_square_type arg)
  {
    msg_.en_passant_square = std::move(arg);
    return Init_BoardState_halfmove_clock(msg_);
  }

private:
  ::chess_interfaces::msg::BoardState msg_;
};

class Init_BoardState_castling_rights
{
public:
  explicit Init_BoardState_castling_rights(::chess_interfaces::msg::BoardState & msg)
  : msg_(msg)
  {}
  Init_BoardState_en_passant_square castling_rights(::chess_interfaces::msg::BoardState::_castling_rights_type arg)
  {
    msg_.castling_rights = std::move(arg);
    return Init_BoardState_en_passant_square(msg_);
  }

private:
  ::chess_interfaces::msg::BoardState msg_;
};

class Init_BoardState_white_to_move
{
public:
  explicit Init_BoardState_white_to_move(::chess_interfaces::msg::BoardState & msg)
  : msg_(msg)
  {}
  Init_BoardState_castling_rights white_to_move(::chess_interfaces::msg::BoardState::_white_to_move_type arg)
  {
    msg_.white_to_move = std::move(arg);
    return Init_BoardState_castling_rights(msg_);
  }

private:
  ::chess_interfaces::msg::BoardState msg_;
};

class Init_BoardState_square_positions_3d
{
public:
  explicit Init_BoardState_square_positions_3d(::chess_interfaces::msg::BoardState & msg)
  : msg_(msg)
  {}
  Init_BoardState_white_to_move square_positions_3d(::chess_interfaces::msg::BoardState::_square_positions_3d_type arg)
  {
    msg_.square_positions_3d = std::move(arg);
    return Init_BoardState_white_to_move(msg_);
  }

private:
  ::chess_interfaces::msg::BoardState msg_;
};

class Init_BoardState_board_squares
{
public:
  explicit Init_BoardState_board_squares(::chess_interfaces::msg::BoardState & msg)
  : msg_(msg)
  {}
  Init_BoardState_square_positions_3d board_squares(::chess_interfaces::msg::BoardState::_board_squares_type arg)
  {
    msg_.board_squares = std::move(arg);
    return Init_BoardState_square_positions_3d(msg_);
  }

private:
  ::chess_interfaces::msg::BoardState msg_;
};

class Init_BoardState_header
{
public:
  Init_BoardState_header()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_BoardState_board_squares header(::chess_interfaces::msg::BoardState::_header_type arg)
  {
    msg_.header = std::move(arg);
    return Init_BoardState_board_squares(msg_);
  }

private:
  ::chess_interfaces::msg::BoardState msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::chess_interfaces::msg::BoardState>()
{
  return chess_interfaces::msg::builder::Init_BoardState_header();
}

}  // namespace chess_interfaces

#endif  // CHESS_INTERFACES__MSG__DETAIL__BOARD_STATE__BUILDER_HPP_
