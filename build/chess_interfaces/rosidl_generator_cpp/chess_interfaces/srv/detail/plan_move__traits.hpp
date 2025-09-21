// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from chess_interfaces:srv/PlanMove.idl
// generated code does not contain a copyright notice

#ifndef CHESS_INTERFACES__SRV__DETAIL__PLAN_MOVE__TRAITS_HPP_
#define CHESS_INTERFACES__SRV__DETAIL__PLAN_MOVE__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "chess_interfaces/srv/detail/plan_move__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

// Include directives for member types
// Member 'chess_move'
#include "chess_interfaces/msg/detail/chess_move__traits.hpp"

namespace chess_interfaces
{

namespace srv
{

inline void to_flow_style_yaml(
  const PlanMove_Request & msg,
  std::ostream & out)
{
  out << "{";
  // member: chess_move
  {
    out << "chess_move: ";
    to_flow_style_yaml(msg.chess_move, out);
    out << ", ";
  }

  // member: speed_factor
  {
    out << "speed_factor: ";
    rosidl_generator_traits::value_to_yaml(msg.speed_factor, out);
    out << ", ";
  }

  // member: avoid_pieces
  {
    out << "avoid_pieces: ";
    rosidl_generator_traits::value_to_yaml(msg.avoid_pieces, out);
    out << ", ";
  }

  // member: use_safe_approach
  {
    out << "use_safe_approach: ";
    rosidl_generator_traits::value_to_yaml(msg.use_safe_approach, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const PlanMove_Request & msg,
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

  // member: speed_factor
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "speed_factor: ";
    rosidl_generator_traits::value_to_yaml(msg.speed_factor, out);
    out << "\n";
  }

  // member: avoid_pieces
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "avoid_pieces: ";
    rosidl_generator_traits::value_to_yaml(msg.avoid_pieces, out);
    out << "\n";
  }

  // member: use_safe_approach
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "use_safe_approach: ";
    rosidl_generator_traits::value_to_yaml(msg.use_safe_approach, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const PlanMove_Request & msg, bool use_flow_style = false)
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
  const chess_interfaces::srv::PlanMove_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  chess_interfaces::srv::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use chess_interfaces::srv::to_yaml() instead")]]
inline std::string to_yaml(const chess_interfaces::srv::PlanMove_Request & msg)
{
  return chess_interfaces::srv::to_yaml(msg);
}

template<>
inline const char * data_type<chess_interfaces::srv::PlanMove_Request>()
{
  return "chess_interfaces::srv::PlanMove_Request";
}

template<>
inline const char * name<chess_interfaces::srv::PlanMove_Request>()
{
  return "chess_interfaces/srv/PlanMove_Request";
}

template<>
struct has_fixed_size<chess_interfaces::srv::PlanMove_Request>
  : std::integral_constant<bool, has_fixed_size<chess_interfaces::msg::ChessMove>::value> {};

template<>
struct has_bounded_size<chess_interfaces::srv::PlanMove_Request>
  : std::integral_constant<bool, has_bounded_size<chess_interfaces::msg::ChessMove>::value> {};

template<>
struct is_message<chess_interfaces::srv::PlanMove_Request>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace chess_interfaces
{

namespace srv
{

inline void to_flow_style_yaml(
  const PlanMove_Response & msg,
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

  // member: execution_time
  {
    out << "execution_time: ";
    rosidl_generator_traits::value_to_yaml(msg.execution_time, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const PlanMove_Response & msg,
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

  // member: execution_time
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "execution_time: ";
    rosidl_generator_traits::value_to_yaml(msg.execution_time, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const PlanMove_Response & msg, bool use_flow_style = false)
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
  const chess_interfaces::srv::PlanMove_Response & msg,
  std::ostream & out, size_t indentation = 0)
{
  chess_interfaces::srv::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use chess_interfaces::srv::to_yaml() instead")]]
inline std::string to_yaml(const chess_interfaces::srv::PlanMove_Response & msg)
{
  return chess_interfaces::srv::to_yaml(msg);
}

template<>
inline const char * data_type<chess_interfaces::srv::PlanMove_Response>()
{
  return "chess_interfaces::srv::PlanMove_Response";
}

template<>
inline const char * name<chess_interfaces::srv::PlanMove_Response>()
{
  return "chess_interfaces/srv/PlanMove_Response";
}

template<>
struct has_fixed_size<chess_interfaces::srv::PlanMove_Response>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<chess_interfaces::srv::PlanMove_Response>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<chess_interfaces::srv::PlanMove_Response>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<chess_interfaces::srv::PlanMove>()
{
  return "chess_interfaces::srv::PlanMove";
}

template<>
inline const char * name<chess_interfaces::srv::PlanMove>()
{
  return "chess_interfaces/srv/PlanMove";
}

template<>
struct has_fixed_size<chess_interfaces::srv::PlanMove>
  : std::integral_constant<
    bool,
    has_fixed_size<chess_interfaces::srv::PlanMove_Request>::value &&
    has_fixed_size<chess_interfaces::srv::PlanMove_Response>::value
  >
{
};

template<>
struct has_bounded_size<chess_interfaces::srv::PlanMove>
  : std::integral_constant<
    bool,
    has_bounded_size<chess_interfaces::srv::PlanMove_Request>::value &&
    has_bounded_size<chess_interfaces::srv::PlanMove_Response>::value
  >
{
};

template<>
struct is_service<chess_interfaces::srv::PlanMove>
  : std::true_type
{
};

template<>
struct is_service_request<chess_interfaces::srv::PlanMove_Request>
  : std::true_type
{
};

template<>
struct is_service_response<chess_interfaces::srv::PlanMove_Response>
  : std::true_type
{
};

}  // namespace rosidl_generator_traits

#endif  // CHESS_INTERFACES__SRV__DETAIL__PLAN_MOVE__TRAITS_HPP_
