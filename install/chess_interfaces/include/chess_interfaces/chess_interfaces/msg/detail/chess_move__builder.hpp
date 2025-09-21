// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from chess_interfaces:msg/ChessMove.idl
// generated code does not contain a copyright notice

#ifndef CHESS_INTERFACES__MSG__DETAIL__CHESS_MOVE__BUILDER_HPP_
#define CHESS_INTERFACES__MSG__DETAIL__CHESS_MOVE__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "chess_interfaces/msg/detail/chess_move__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace chess_interfaces
{

namespace msg
{

namespace builder
{

class Init_ChessMove_thinking_time
{
public:
  explicit Init_ChessMove_thinking_time(::chess_interfaces::msg::ChessMove & msg)
  : msg_(msg)
  {}
  ::chess_interfaces::msg::ChessMove thinking_time(::chess_interfaces::msg::ChessMove::_thinking_time_type arg)
  {
    msg_.thinking_time = std::move(arg);
    return std::move(msg_);
  }

private:
  ::chess_interfaces::msg::ChessMove msg_;
};

class Init_ChessMove_confidence
{
public:
  explicit Init_ChessMove_confidence(::chess_interfaces::msg::ChessMove & msg)
  : msg_(msg)
  {}
  Init_ChessMove_thinking_time confidence(::chess_interfaces::msg::ChessMove::_confidence_type arg)
  {
    msg_.confidence = std::move(arg);
    return Init_ChessMove_thinking_time(msg_);
  }

private:
  ::chess_interfaces::msg::ChessMove msg_;
};

class Init_ChessMove_is_checkmate
{
public:
  explicit Init_ChessMove_is_checkmate(::chess_interfaces::msg::ChessMove & msg)
  : msg_(msg)
  {}
  Init_ChessMove_confidence is_checkmate(::chess_interfaces::msg::ChessMove::_is_checkmate_type arg)
  {
    msg_.is_checkmate = std::move(arg);
    return Init_ChessMove_confidence(msg_);
  }

private:
  ::chess_interfaces::msg::ChessMove msg_;
};

class Init_ChessMove_is_check
{
public:
  explicit Init_ChessMove_is_check(::chess_interfaces::msg::ChessMove & msg)
  : msg_(msg)
  {}
  Init_ChessMove_is_checkmate is_check(::chess_interfaces::msg::ChessMove::_is_check_type arg)
  {
    msg_.is_check = std::move(arg);
    return Init_ChessMove_is_checkmate(msg_);
  }

private:
  ::chess_interfaces::msg::ChessMove msg_;
};

class Init_ChessMove_is_en_passant
{
public:
  explicit Init_ChessMove_is_en_passant(::chess_interfaces::msg::ChessMove & msg)
  : msg_(msg)
  {}
  Init_ChessMove_is_check is_en_passant(::chess_interfaces::msg::ChessMove::_is_en_passant_type arg)
  {
    msg_.is_en_passant = std::move(arg);
    return Init_ChessMove_is_check(msg_);
  }

private:
  ::chess_interfaces::msg::ChessMove msg_;
};

class Init_ChessMove_is_castling
{
public:
  explicit Init_ChessMove_is_castling(::chess_interfaces::msg::ChessMove & msg)
  : msg_(msg)
  {}
  Init_ChessMove_is_en_passant is_castling(::chess_interfaces::msg::ChessMove::_is_castling_type arg)
  {
    msg_.is_castling = std::move(arg);
    return Init_ChessMove_is_en_passant(msg_);
  }

private:
  ::chess_interfaces::msg::ChessMove msg_;
};

class Init_ChessMove_promotion
{
public:
  explicit Init_ChessMove_promotion(::chess_interfaces::msg::ChessMove & msg)
  : msg_(msg)
  {}
  Init_ChessMove_is_castling promotion(::chess_interfaces::msg::ChessMove::_promotion_type arg)
  {
    msg_.promotion = std::move(arg);
    return Init_ChessMove_is_castling(msg_);
  }

private:
  ::chess_interfaces::msg::ChessMove msg_;
};

class Init_ChessMove_captured_piece
{
public:
  explicit Init_ChessMove_captured_piece(::chess_interfaces::msg::ChessMove & msg)
  : msg_(msg)
  {}
  Init_ChessMove_promotion captured_piece(::chess_interfaces::msg::ChessMove::_captured_piece_type arg)
  {
    msg_.captured_piece = std::move(arg);
    return Init_ChessMove_promotion(msg_);
  }

private:
  ::chess_interfaces::msg::ChessMove msg_;
};

class Init_ChessMove_piece_type
{
public:
  explicit Init_ChessMove_piece_type(::chess_interfaces::msg::ChessMove & msg)
  : msg_(msg)
  {}
  Init_ChessMove_captured_piece piece_type(::chess_interfaces::msg::ChessMove::_piece_type_type arg)
  {
    msg_.piece_type = std::move(arg);
    return Init_ChessMove_captured_piece(msg_);
  }

private:
  ::chess_interfaces::msg::ChessMove msg_;
};

class Init_ChessMove_to_square
{
public:
  explicit Init_ChessMove_to_square(::chess_interfaces::msg::ChessMove & msg)
  : msg_(msg)
  {}
  Init_ChessMove_piece_type to_square(::chess_interfaces::msg::ChessMove::_to_square_type arg)
  {
    msg_.to_square = std::move(arg);
    return Init_ChessMove_piece_type(msg_);
  }

private:
  ::chess_interfaces::msg::ChessMove msg_;
};

class Init_ChessMove_from_square
{
public:
  explicit Init_ChessMove_from_square(::chess_interfaces::msg::ChessMove & msg)
  : msg_(msg)
  {}
  Init_ChessMove_to_square from_square(::chess_interfaces::msg::ChessMove::_from_square_type arg)
  {
    msg_.from_square = std::move(arg);
    return Init_ChessMove_to_square(msg_);
  }

private:
  ::chess_interfaces::msg::ChessMove msg_;
};

class Init_ChessMove_header
{
public:
  Init_ChessMove_header()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_ChessMove_from_square header(::chess_interfaces::msg::ChessMove::_header_type arg)
  {
    msg_.header = std::move(arg);
    return Init_ChessMove_from_square(msg_);
  }

private:
  ::chess_interfaces::msg::ChessMove msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::chess_interfaces::msg::ChessMove>()
{
  return chess_interfaces::msg::builder::Init_ChessMove_header();
}

}  // namespace chess_interfaces

#endif  // CHESS_INTERFACES__MSG__DETAIL__CHESS_MOVE__BUILDER_HPP_
