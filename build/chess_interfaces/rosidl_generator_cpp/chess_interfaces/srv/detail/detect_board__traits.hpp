// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from chess_interfaces:srv/DetectBoard.idl
// generated code does not contain a copyright notice

#ifndef CHESS_INTERFACES__SRV__DETAIL__DETECT_BOARD__TRAITS_HPP_
#define CHESS_INTERFACES__SRV__DETAIL__DETECT_BOARD__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "chess_interfaces/srv/detail/detect_board__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

// Include directives for member types
// Member 'rgb_image'
// Member 'depth_image'
#include "sensor_msgs/msg/detail/image__traits.hpp"

namespace chess_interfaces
{

namespace srv
{

inline void to_flow_style_yaml(
  const DetectBoard_Request & msg,
  std::ostream & out)
{
  out << "{";
  // member: rgb_image
  {
    out << "rgb_image: ";
    to_flow_style_yaml(msg.rgb_image, out);
    out << ", ";
  }

  // member: depth_image
  {
    out << "depth_image: ";
    to_flow_style_yaml(msg.depth_image, out);
    out << ", ";
  }

  // member: force_update
  {
    out << "force_update: ";
    rosidl_generator_traits::value_to_yaml(msg.force_update, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const DetectBoard_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: rgb_image
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "rgb_image:\n";
    to_block_style_yaml(msg.rgb_image, out, indentation + 2);
  }

  // member: depth_image
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "depth_image:\n";
    to_block_style_yaml(msg.depth_image, out, indentation + 2);
  }

  // member: force_update
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "force_update: ";
    rosidl_generator_traits::value_to_yaml(msg.force_update, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const DetectBoard_Request & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace srv

}  // namespace chess_interfaces

namespace rosidl_generator_traits
{

[[deprecated("use chess_interfaces::srv::to_block_style_yaml() instead")]]
inline void to_yaml(
  const chess_interfaces::srv::DetectBoard_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  chess_interfaces::srv::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use chess_interfaces::srv::to_yaml() instead")]]
inline std::string to_yaml(const chess_interfaces::srv::DetectBoard_Request & msg)
{
  return chess_interfaces::srv::to_yaml(msg);
}

template<>
inline const char * data_type<chess_interfaces::srv::DetectBoard_Request>()
{
  return "chess_interfaces::srv::DetectBoard_Request";
}

template<>
inline const char * name<chess_interfaces::srv::DetectBoard_Request>()
{
  return "chess_interfaces/srv/DetectBoard_Request";
}

template<>
struct has_fixed_size<chess_interfaces::srv::DetectBoard_Request>
  : std::integral_constant<bool, has_fixed_size<sensor_msgs::msg::Image>::value> {};

template<>
struct has_bounded_size<chess_interfaces::srv::DetectBoard_Request>
  : std::integral_constant<bool, has_bounded_size<sensor_msgs::msg::Image>::value> {};

template<>
struct is_message<chess_interfaces::srv::DetectBoard_Request>
  : std::true_type {};

}  // namespace rosidl_generator_traits

// Include directives for member types
// Member 'board_state'
#include "chess_interfaces/msg/detail/board_state__traits.hpp"
// Member 'board_transform'
#include "geometry_msgs/msg/detail/transform_stamped__traits.hpp"

namespace chess_interfaces
{

namespace srv
{

inline void to_flow_style_yaml(
  const DetectBoard_Response & msg,
  std::ostream & out)
{
  out << "{";
  // member: success
  {
    out << "success: ";
    rosidl_generator_traits::value_to_yaml(msg.success, out);
    out << ", ";
  }

  // member: error_message
  {
    out << "error_message: ";
    rosidl_generator_traits::value_to_yaml(msg.error_message, out);
    out << ", ";
  }

  // member: board_state
  {
    out << "board_state: ";
    to_flow_style_yaml(msg.board_state, out);
    out << ", ";
  }

  // member: board_transform
  {
    out << "board_transform: ";
    to_flow_style_yaml(msg.board_transform, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const DetectBoard_Response & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: success
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "success: ";
    rosidl_generator_traits::value_to_yaml(msg.success, out);
    out << "\n";
  }

  // member: error_message
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "error_message: ";
    rosidl_generator_traits::value_to_yaml(msg.error_message, out);
    out << "\n";
  }

  // member: board_state
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "board_state:\n";
    to_block_style_yaml(msg.board_state, out, indentation + 2);
  }

  // member: board_transform
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "board_transform:\n";
    to_block_style_yaml(msg.board_transform, out, indentation + 2);
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const DetectBoard_Response & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace srv

}  // namespace chess_interfaces

namespace rosidl_generator_traits
{

[[deprecated("use chess_interfaces::srv::to_block_style_yaml() instead")]]
inline void to_yaml(
  const chess_interfaces::srv::DetectBoard_Response & msg,
  std::ostream & out, size_t indentation = 0)
{
  chess_interfaces::srv::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use chess_interfaces::srv::to_yaml() instead")]]
inline std::string to_yaml(const chess_interfaces::srv::DetectBoard_Response & msg)
{
  return chess_interfaces::srv::to_yaml(msg);
}

template<>
inline const char * data_type<chess_interfaces::srv::DetectBoard_Response>()
{
  return "chess_interfaces::srv::DetectBoard_Response";
}

template<>
inline const char * name<chess_interfaces::srv::DetectBoard_Response>()
{
  return "chess_interfaces/srv/DetectBoard_Response";
}

template<>
struct has_fixed_size<chess_interfaces::srv::DetectBoard_Response>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<chess_interfaces::srv::DetectBoard_Response>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<chess_interfaces::srv::DetectBoard_Response>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<chess_interfaces::srv::DetectBoard>()
{
  return "chess_interfaces::srv::DetectBoard";
}

template<>
inline const char * name<chess_interfaces::srv::DetectBoard>()
{
  return "chess_interfaces/srv/DetectBoard";
}

template<>
struct has_fixed_size<chess_interfaces::srv::DetectBoard>
  : std::integral_constant<
    bool,
    has_fixed_size<chess_interfaces::srv::DetectBoard_Request>::value &&
    has_fixed_size<chess_interfaces::srv::DetectBoard_Response>::value
  >
{
};

template<>
struct has_bounded_size<chess_interfaces::srv::DetectBoard>
  : std::integral_constant<
    bool,
    has_bounded_size<chess_interfaces::srv::DetectBoard_Request>::value &&
    has_bounded_size<chess_interfaces::srv::DetectBoard_Response>::value
  >
{
};

template<>
struct is_service<chess_interfaces::srv::DetectBoard>
  : std::true_type
{
};

template<>
struct is_service_request<chess_interfaces::srv::DetectBoard_Request>
  : std::true_type
{
};

template<>
struct is_service_response<chess_interfaces::srv::DetectBoard_Response>
  : std::true_type
{
};

}  // namespace rosidl_generator_traits

#endif  // CHESS_INTERFACES__SRV__DETAIL__DETECT_BOARD__TRAITS_HPP_
