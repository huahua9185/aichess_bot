// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from chess_interfaces:srv/PlanMove.idl
// generated code does not contain a copyright notice

#ifndef CHESS_INTERFACES__SRV__DETAIL__PLAN_MOVE__BUILDER_HPP_
#define CHESS_INTERFACES__SRV__DETAIL__PLAN_MOVE__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "chess_interfaces/srv/detail/plan_move__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace chess_interfaces
{

namespace srv
{

namespace builder
{

class Init_PlanMove_Request_use_safe_approach
{
public:
  explicit Init_PlanMove_Request_use_safe_approach(::chess_interfaces::srv::PlanMove_Request & msg)
  : msg_(msg)
  {}
  ::chess_interfaces::srv::PlanMove_Request use_safe_approach(::chess_interfaces::srv::PlanMove_Request::_use_safe_approach_type arg)
  {
    msg_.use_safe_approach = std::move(arg);
    return std::move(msg_);
  }

private:
  ::chess_interfaces::srv::PlanMove_Request msg_;
};

class Init_PlanMove_Request_avoid_pieces
{
public:
  explicit Init_PlanMove_Request_avoid_pieces(::chess_interfaces::srv::PlanMove_Request & msg)
  : msg_(msg)
  {}
  Init_PlanMove_Request_use_safe_approach avoid_pieces(::chess_interfaces::srv::PlanMove_Request::_avoid_pieces_type arg)
  {
    msg_.avoid_pieces = std::move(arg);
    return Init_PlanMove_Request_use_safe_approach(msg_);
  }

private:
  ::chess_interfaces::srv::PlanMove_Request msg_;
};

class Init_PlanMove_Request_speed_factor
{
public:
  explicit Init_PlanMove_Request_speed_factor(::chess_interfaces::srv::PlanMove_Request & msg)
  : msg_(msg)
  {}
  Init_PlanMove_Request_avoid_pieces speed_factor(::chess_interfaces::srv::PlanMove_Request::_speed_factor_type arg)
  {
    msg_.speed_factor = std::move(arg);
    return Init_PlanMove_Request_avoid_pieces(msg_);
  }

private:
  ::chess_interfaces::srv::PlanMove_Request msg_;
};

class Init_PlanMove_Request_chess_move
{
public:
  Init_PlanMove_Request_chess_move()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_PlanMove_Request_speed_factor chess_move(::chess_interfaces::srv::PlanMove_Request::_chess_move_type arg)
  {
    msg_.chess_move = std::move(arg);
    return Init_PlanMove_Request_speed_factor(msg_);
  }

private:
  ::chess_interfaces::srv::PlanMove_Request msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::chess_interfaces::srv::PlanMove_Request>()
{
  return chess_interfaces::srv::builder::Init_PlanMove_Request_chess_move();
}

}  // namespace chess_interfaces


namespace chess_interfaces
{

namespace srv
{

namespace builder
{

class Init_PlanMove_Response_execution_time
{
public:
  explicit Init_PlanMove_Response_execution_time(::chess_interfaces::srv::PlanMove_Response & msg)
  : msg_(msg)
  {}
  ::chess_interfaces::srv::PlanMove_Response execution_time(::chess_interfaces::srv::PlanMove_Response::_execution_time_type arg)
  {
    msg_.execution_time = std::move(arg);
    return std::move(msg_);
  }

private:
  ::chess_interfaces::srv::PlanMove_Response msg_;
};

class Init_PlanMove_Response_error_message
{
public:
  explicit Init_PlanMove_Response_error_message(::chess_interfaces::srv::PlanMove_Response & msg)
  : msg_(msg)
  {}
  Init_PlanMove_Response_execution_time error_message(::chess_interfaces::srv::PlanMove_Response::_error_message_type arg)
  {
    msg_.error_message = std::move(arg);
    return Init_PlanMove_Response_execution_time(msg_);
  }

private:
  ::chess_interfaces::srv::PlanMove_Response msg_;
};

class Init_PlanMove_Response_success
{
public:
  Init_PlanMove_Response_success()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_PlanMove_Response_error_message success(::chess_interfaces::srv::PlanMove_Response::_success_type arg)
  {
    msg_.success = std::move(arg);
    return Init_PlanMove_Response_error_message(msg_);
  }

private:
  ::chess_interfaces::srv::PlanMove_Response msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::chess_interfaces::srv::PlanMove_Response>()
{
  return chess_interfaces::srv::builder::Init_PlanMove_Response_success();
}

}  // namespace chess_interfaces

#endif  // CHESS_INTERFACES__SRV__DETAIL__PLAN_MOVE__BUILDER_HPP_
