// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from chess_interfaces:srv/ExecuteMove.idl
// generated code does not contain a copyright notice

#ifndef CHESS_INTERFACES__SRV__DETAIL__EXECUTE_MOVE__BUILDER_HPP_
#define CHESS_INTERFACES__SRV__DETAIL__EXECUTE_MOVE__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "chess_interfaces/srv/detail/execute_move__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace chess_interfaces
{

namespace srv
{

namespace builder
{

class Init_ExecuteMove_Request_confirm_execution
{
public:
  explicit Init_ExecuteMove_Request_confirm_execution(::chess_interfaces::srv::ExecuteMove_Request & msg)
  : msg_(msg)
  {}
  ::chess_interfaces::srv::ExecuteMove_Request confirm_execution(::chess_interfaces::srv::ExecuteMove_Request::_confirm_execution_type arg)
  {
    msg_.confirm_execution = std::move(arg);
    return std::move(msg_);
  }

private:
  ::chess_interfaces::srv::ExecuteMove_Request msg_;
};

class Init_ExecuteMove_Request_chess_move
{
public:
  Init_ExecuteMove_Request_chess_move()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_ExecuteMove_Request_confirm_execution chess_move(::chess_interfaces::srv::ExecuteMove_Request::_chess_move_type arg)
  {
    msg_.chess_move = std::move(arg);
    return Init_ExecuteMove_Request_confirm_execution(msg_);
  }

private:
  ::chess_interfaces::srv::ExecuteMove_Request msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::chess_interfaces::srv::ExecuteMove_Request>()
{
  return chess_interfaces::srv::builder::Init_ExecuteMove_Request_chess_move();
}

}  // namespace chess_interfaces


namespace chess_interfaces
{

namespace srv
{

namespace builder
{

class Init_ExecuteMove_Response_piece_captured
{
public:
  explicit Init_ExecuteMove_Response_piece_captured(::chess_interfaces::srv::ExecuteMove_Response & msg)
  : msg_(msg)
  {}
  ::chess_interfaces::srv::ExecuteMove_Response piece_captured(::chess_interfaces::srv::ExecuteMove_Response::_piece_captured_type arg)
  {
    msg_.piece_captured = std::move(arg);
    return std::move(msg_);
  }

private:
  ::chess_interfaces::srv::ExecuteMove_Response msg_;
};

class Init_ExecuteMove_Response_actual_execution_time
{
public:
  explicit Init_ExecuteMove_Response_actual_execution_time(::chess_interfaces::srv::ExecuteMove_Response & msg)
  : msg_(msg)
  {}
  Init_ExecuteMove_Response_piece_captured actual_execution_time(::chess_interfaces::srv::ExecuteMove_Response::_actual_execution_time_type arg)
  {
    msg_.actual_execution_time = std::move(arg);
    return Init_ExecuteMove_Response_piece_captured(msg_);
  }

private:
  ::chess_interfaces::srv::ExecuteMove_Response msg_;
};

class Init_ExecuteMove_Response_error_message
{
public:
  explicit Init_ExecuteMove_Response_error_message(::chess_interfaces::srv::ExecuteMove_Response & msg)
  : msg_(msg)
  {}
  Init_ExecuteMove_Response_actual_execution_time error_message(::chess_interfaces::srv::ExecuteMove_Response::_error_message_type arg)
  {
    msg_.error_message = std::move(arg);
    return Init_ExecuteMove_Response_actual_execution_time(msg_);
  }

private:
  ::chess_interfaces::srv::ExecuteMove_Response msg_;
};

class Init_ExecuteMove_Response_success
{
public:
  Init_ExecuteMove_Response_success()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_ExecuteMove_Response_error_message success(::chess_interfaces::srv::ExecuteMove_Response::_success_type arg)
  {
    msg_.success = std::move(arg);
    return Init_ExecuteMove_Response_error_message(msg_);
  }

private:
  ::chess_interfaces::srv::ExecuteMove_Response msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::chess_interfaces::srv::ExecuteMove_Response>()
{
  return chess_interfaces::srv::builder::Init_ExecuteMove_Response_success();
}

}  // namespace chess_interfaces

#endif  // CHESS_INTERFACES__SRV__DETAIL__EXECUTE_MOVE__BUILDER_HPP_
