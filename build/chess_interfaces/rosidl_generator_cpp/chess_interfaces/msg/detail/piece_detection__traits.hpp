// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from chess_interfaces:msg/PieceDetection.idl
// generated code does not contain a copyright notice

#ifndef CHESS_INTERFACES__MSG__DETAIL__PIECE_DETECTION__TRAITS_HPP_
#define CHESS_INTERFACES__MSG__DETAIL__PIECE_DETECTION__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "chess_interfaces/msg/detail/piece_detection__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__traits.hpp"
// Member 'position_3d'
// Member 'position_2d'
#include "geometry_msgs/msg/detail/point__traits.hpp"
// Member 'roi_image'
#include "sensor_msgs/msg/detail/image__traits.hpp"

namespace chess_interfaces
{

namespace msg
{

inline void to_flow_style_yaml(
  const PieceDetection & msg,
  std::ostream & out)
{
  out << "{";
  // member: header
  {
    out << "header: ";
    to_flow_style_yaml(msg.header, out);
    out << ", ";
  }

  // member: position_3d
  {
    out << "position_3d: ";
    to_flow_style_yaml(msg.position_3d, out);
    out << ", ";
  }

  // member: position_2d
  {
    out << "position_2d: ";
    to_flow_style_yaml(msg.position_2d, out);
    out << ", ";
  }

  // member: piece_type
  {
    out << "piece_type: ";
    rosidl_generator_traits::value_to_yaml(msg.piece_type, out);
    out << ", ";
  }

  // member: piece_color
  {
    out << "piece_color: ";
    rosidl_generator_traits::value_to_yaml(msg.piece_color, out);
    out << ", ";
  }

  // member: confidence
  {
    out << "confidence: ";
    rosidl_generator_traits::value_to_yaml(msg.confidence, out);
    out << ", ";
  }

  // member: roi_image
  {
    out << "roi_image: ";
    to_flow_style_yaml(msg.roi_image, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const PieceDetection & msg,
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

  // member: position_3d
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "position_3d:\n";
    to_block_style_yaml(msg.position_3d, out, indentation + 2);
  }

  // member: position_2d
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "position_2d:\n";
    to_block_style_yaml(msg.position_2d, out, indentation + 2);
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

  // member: piece_color
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "piece_color: ";
    rosidl_generator_traits::value_to_yaml(msg.piece_color, out);
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

  // member: roi_image
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "roi_image:\n";
    to_block_style_yaml(msg.roi_image, out, indentation + 2);
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const PieceDetection & msg, bool use_flow_style = false)
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
  const chess_interfaces::msg::PieceDetection & msg,
  std::ostream & out, size_t indentation = 0)
{
  chess_interfaces::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use chess_interfaces::msg::to_yaml() instead")]]
inline std::string to_yaml(const chess_interfaces::msg::PieceDetection & msg)
{
  return chess_interfaces::msg::to_yaml(msg);
}

template<>
inline const char * data_type<chess_interfaces::msg::PieceDetection>()
{
  return "chess_interfaces::msg::PieceDetection";
}

template<>
inline const char * name<chess_interfaces::msg::PieceDetection>()
{
  return "chess_interfaces/msg/PieceDetection";
}

template<>
struct has_fixed_size<chess_interfaces::msg::PieceDetection>
  : std::integral_constant<bool, has_fixed_size<geometry_msgs::msg::Point>::value && has_fixed_size<sensor_msgs::msg::Image>::value && has_fixed_size<std_msgs::msg::Header>::value> {};

template<>
struct has_bounded_size<chess_interfaces::msg::PieceDetection>
  : std::integral_constant<bool, has_bounded_size<geometry_msgs::msg::Point>::value && has_bounded_size<sensor_msgs::msg::Image>::value && has_bounded_size<std_msgs::msg::Header>::value> {};

template<>
struct is_message<chess_interfaces::msg::PieceDetection>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // CHESS_INTERFACES__MSG__DETAIL__PIECE_DETECTION__TRAITS_HPP_
