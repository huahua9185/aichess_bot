// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from chess_interfaces:srv/ExecuteMove.idl
// generated code does not contain a copyright notice

#ifndef CHESS_INTERFACES__SRV__DETAIL__EXECUTE_MOVE__TRAITS_HPP_
#define CHESS_INTERFACES__SRV__DETAIL__EXECUTE_MOVE__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "chess_interfaces/srv/detail/execute_move__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

// Include directives for member types
// Member 'chess_move'
#include "chess_interfaces/msg/detail/chess_move__traits.hpp"

namespace chess_interfaces
{

namespace srv
{

inline void to_flow_style_yaml(
  const ExecuteMove_Request & msg,
  std::ostream & out)
{
  out << "{";
  // member: chess_move
  {
    out << "chess_move: ";
    to_flow_style_yaml(msg.chess_move, out);
    out << ", ";
  }

  // member: confirm_execution
  {
    out << "confirm_execution: ";
    rosidl_generator_traits::value_to_yaml(msg.confirm_execution, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const ExecuteMove_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: chess_move
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "chess_move:\n";
    to_block_style_yaml(msg.chess_move, out, indentation + 2);
  }

  // member: confirm_execution
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "confirm_execution: ";
    rosidl_generator_traits::value_to_yaml(msg.confirm_execution, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const ExecuteMove_Request & msg, bool use_flow_style = false)
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
  const chess_interfaces::srv::ExecuteMove_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  chess_interfaces::srv::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use chess_interfaces::srv::to_yaml() instead")]]
inline std::string to_yaml(const chess_interfaces::srv::ExecuteMove_Request & msg)
{
  return chess_interfaces::srv::to_yaml(msg);
}

template<>
inline const char * data_type<chess_interfaces::srv::ExecuteMove_Request>()
{
  return "chess_interfaces::srv::ExecuteMove_Request";
}

template<>
inline const char * name<chess_interfaces::srv::ExecuteMove_Request>()
{
  return "chess_interfaces/srv/ExecuteMove_Request";
}

template<>
struct has_fixed_size<chess_interfaces::srv::ExecuteMove_Request>
  : std::integral_constant<bool, has_fixed_size<chess_interfaces::msg::ChessMove>::value> {};

template<>
struct has_bounded_size<chess_interfaces::srv::ExecuteMove_Request>
  : std::integral_constant<bool, has_bounded_size<chess_interfaces::msg::ChessMove>::value> {};

template<>
struct is_message<chess_interfaces::srv::ExecuteMove_Request>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace chess_interfaces
{

namespace srv
{

inline void to_flow_style_yaml(
  const ExecuteMove_Response & msg,
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

  // member: actual_execution_time
  {
    out << "actual_execution_time: ";
    rosidl_generator_traits::value_to_yaml(msg.actual_execution_time, out);
    out << ", ";
  }

  // member: piece_captured
  {
    out << "piece_captured: ";
    rosidl_generator_traits::value_to_yaml(msg.piece_captured, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const ExecuteMove_Response & msg,
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

  // member: actual_execution_time
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "actual_execution_time: ";
    rosidl_generator_traits::value_to_yaml(msg.actual_execution_time, out);
    out << "\n";
  }

  // member: piece_captured
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "piece_captured: ";
    rosidl_generator_traits::value_to_yaml(msg.piece_captured, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const ExecuteMove_Response & msg, bool use_flow_style = false)
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
  const chess_interfaces::srv::ExecuteMove_Response & msg,
  std::ostream & out, size_t indentation = 0)
{
  chess_interfaces::srv::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use chess_interfaces::srv::to_yaml() instead")]]
inline std::string to_yaml(const chess_interfaces::srv::ExecuteMove_Response & msg)
{
  return chess_interfaces::srv::to_yaml(msg);
}

template<>
inline const char * data_type<chess_interfaces::srv::ExecuteMove_Response>()
{
  return "chess_interfaces::srv::ExecuteMove_Response";
}

template<>
inline const char * name<chess_interfaces::srv::ExecuteMove_Response>()
{
  return "chess_interfaces/srv/ExecuteMove_Response";
}

template<>
struct has_fixed_size<chess_interfaces::srv::ExecuteMove_Response>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<chess_interfaces::srv::ExecuteMove_Response>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<chess_interfaces::srv::ExecuteMove_Response>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<chess_interfaces::srv::ExecuteMove>()
{
  return "chess_interfaces::srv::ExecuteMove";
}

template<>
inline const char * name<chess_interfaces::srv::ExecuteMove>()
{
  return "chess_interfaces/srv/ExecuteMove";
}

template<>
struct has_fixed_size<chess_interfaces::srv::ExecuteMove>
  : std::integral_constant<
    bool,
    has_fixed_size<chess_interfaces::srv::ExecuteMove_Request>::value &&
    has_fixed_size<chess_interfaces::srv::ExecuteMove_Response>::value
  >
{
};

template<>
struct has_bounded_size<chess_interfaces::srv::ExecuteMove>
  : std::integral_constant<
    bool,
    has_bounded_size<chess_interfaces::srv::ExecuteMove_Request>::value &&
    has_bounded_size<chess_interfaces::srv::ExecuteMove_Response>::value
  >
{
};

template<>
struct is_service<chess_interfaces::srv::ExecuteMove>
  : std::true_type
{
};

template<>
struct is_service_request<chess_interfaces::srv::ExecuteMove_Request>
  : std::true_type
{
};

template<>
struct is_service_response<chess_interfaces::srv::ExecuteMove_Response>
  : std::true_type
{
};

}  // namespace rosidl_generator_traits

#endif  // CHESS_INTERFACES__SRV__DETAIL__EXECUTE_MOVE__TRAITS_HPP_
