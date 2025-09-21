// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from chess_interfaces:msg/PieceDetection.idl
// generated code does not contain a copyright notice

#ifndef CHESS_INTERFACES__MSG__DETAIL__PIECE_DETECTION__BUILDER_HPP_
#define CHESS_INTERFACES__MSG__DETAIL__PIECE_DETECTION__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "chess_interfaces/msg/detail/piece_detection__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace chess_interfaces
{

namespace msg
{

namespace builder
{

class Init_PieceDetection_roi_image
{
public:
  explicit Init_PieceDetection_roi_image(::chess_interfaces::msg::PieceDetection & msg)
  : msg_(msg)
  {}
  ::chess_interfaces::msg::PieceDetection roi_image(::chess_interfaces::msg::PieceDetection::_roi_image_type arg)
  {
    msg_.roi_image = std::move(arg);
    return std::move(msg_);
  }

private:
  ::chess_interfaces::msg::PieceDetection msg_;
};

class Init_PieceDetection_confidence
{
public:
  explicit Init_PieceDetection_confidence(::chess_interfaces::msg::PieceDetection & msg)
  : msg_(msg)
  {}
  Init_PieceDetection_roi_image confidence(::chess_interfaces::msg::PieceDetection::_confidence_type arg)
  {
    msg_.confidence = std::move(arg);
    return Init_PieceDetection_roi_image(msg_);
  }

private:
  ::chess_interfaces::msg::PieceDetection msg_;
};

class Init_PieceDetection_piece_color
{
public:
  explicit Init_PieceDetection_piece_color(::chess_interfaces::msg::PieceDetection & msg)
  : msg_(msg)
  {}
  Init_PieceDetection_confidence piece_color(::chess_interfaces::msg::PieceDetection::_piece_color_type arg)
  {
    msg_.piece_color = std::move(arg);
    return Init_PieceDetection_confidence(msg_);
  }

private:
  ::chess_interfaces::msg::PieceDetection msg_;
};

class Init_PieceDetection_piece_type
{
public:
  explicit Init_PieceDetection_piece_type(::chess_interfaces::msg::PieceDetection & msg)
  : msg_(msg)
  {}
  Init_PieceDetection_piece_color piece_type(::chess_interfaces::msg::PieceDetection::_piece_type_type arg)
  {
    msg_.piece_type = std::move(arg);
    return Init_PieceDetection_piece_color(msg_);
  }

private:
  ::chess_interfaces::msg::PieceDetection msg_;
};

class Init_PieceDetection_position_2d
{
public:
  explicit Init_PieceDetection_position_2d(::chess_interfaces::msg::PieceDetection & msg)
  : msg_(msg)
  {}
  Init_PieceDetection_piece_type position_2d(::chess_interfaces::msg::PieceDetection::_position_2d_type arg)
  {
    msg_.position_2d = std::move(arg);
    return Init_PieceDetection_piece_type(msg_);
  }

private:
  ::chess_interfaces::msg::PieceDetection msg_;
};

class Init_PieceDetection_position_3d
{
public:
  explicit Init_PieceDetection_position_3d(::chess_interfaces::msg::PieceDetection & msg)
  : msg_(msg)
  {}
  Init_PieceDetection_position_2d position_3d(::chess_interfaces::msg::PieceDetection::_position_3d_type arg)
  {
    msg_.position_3d = std::move(arg);
    return Init_PieceDetection_position_2d(msg_);
  }

private:
  ::chess_interfaces::msg::PieceDetection msg_;
};

class Init_PieceDetection_header
{
public:
  Init_PieceDetection_header()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_PieceDetection_position_3d header(::chess_interfaces::msg::PieceDetection::_header_type arg)
  {
    msg_.header = std::move(arg);
    return Init_PieceDetection_position_3d(msg_);
  }

private:
  ::chess_interfaces::msg::PieceDetection msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::chess_interfaces::msg::PieceDetection>()
{
  return chess_interfaces::msg::builder::Init_PieceDetection_header();
}

}  // namespace chess_interfaces

#endif  // CHESS_INTERFACES__MSG__DETAIL__PIECE_DETECTION__BUILDER_HPP_
