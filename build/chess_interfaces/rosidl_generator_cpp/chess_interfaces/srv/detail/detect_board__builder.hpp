// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from chess_interfaces:srv/DetectBoard.idl
// generated code does not contain a copyright notice

#ifndef CHESS_INTERFACES__SRV__DETAIL__DETECT_BOARD__BUILDER_HPP_
#define CHESS_INTERFACES__SRV__DETAIL__DETECT_BOARD__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "chess_interfaces/srv/detail/detect_board__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace chess_interfaces
{

namespace srv
{

namespace builder
{

class Init_DetectBoard_Request_force_update
{
public:
  explicit Init_DetectBoard_Request_force_update(::chess_interfaces::srv::DetectBoard_Request & msg)
  : msg_(msg)
  {}
  ::chess_interfaces::srv::DetectBoard_Request force_update(::chess_interfaces::srv::DetectBoard_Request::_force_update_type arg)
  {
    msg_.force_update = std::move(arg);
    return std::move(msg_);
  }

private:
  ::chess_interfaces::srv::DetectBoard_Request msg_;
};

class Init_DetectBoard_Request_depth_image
{
public:
  explicit Init_DetectBoard_Request_depth_image(::chess_interfaces::srv::DetectBoard_Request & msg)
  : msg_(msg)
  {}
  Init_DetectBoard_Request_force_update depth_image(::chess_interfaces::srv::DetectBoard_Request::_depth_image_type arg)
  {
    msg_.depth_image = std::move(arg);
    return Init_DetectBoard_Request_force_update(msg_);
  }

private:
  ::chess_interfaces::srv::DetectBoard_Request msg_;
};

class Init_DetectBoard_Request_rgb_image
{
public:
  Init_DetectBoard_Request_rgb_image()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_DetectBoard_Request_depth_image rgb_image(::chess_interfaces::srv::DetectBoard_Request::_rgb_image_type arg)
  {
    msg_.rgb_image = std::move(arg);
    return Init_DetectBoard_Request_depth_image(msg_);
  }

private:
  ::chess_interfaces::srv::DetectBoard_Request msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::chess_interfaces::srv::DetectBoard_Request>()
{
  return chess_interfaces::srv::builder::Init_DetectBoard_Request_rgb_image();
}

}  // namespace chess_interfaces


namespace chess_interfaces
{

namespace srv
{

namespace builder
{

class Init_DetectBoard_Response_board_transform
{
public:
  explicit Init_DetectBoard_Response_board_transform(::chess_interfaces::srv::DetectBoard_Response & msg)
  : msg_(msg)
  {}
  ::chess_interfaces::srv::DetectBoard_Response board_transform(::chess_interfaces::srv::DetectBoard_Response::_board_transform_type arg)
  {
    msg_.board_transform = std::move(arg);
    return std::move(msg_);
  }

private:
  ::chess_interfaces::srv::DetectBoard_Response msg_;
};

class Init_DetectBoard_Response_board_state
{
public:
  explicit Init_DetectBoard_Response_board_state(::chess_interfaces::srv::DetectBoard_Response & msg)
  : msg_(msg)
  {}
  Init_DetectBoard_Response_board_transform board_state(::chess_interfaces::srv::DetectBoard_Response::_board_state_type arg)
  {
    msg_.board_state = std::move(arg);
    return Init_DetectBoard_Response_board_transform(msg_);
  }

private:
  ::chess_interfaces::srv::DetectBoard_Response msg_;
};

class Init_DetectBoard_Response_error_message
{
public:
  explicit Init_DetectBoard_Response_error_message(::chess_interfaces::srv::DetectBoard_Response & msg)
  : msg_(msg)
  {}
  Init_DetectBoard_Response_board_state error_message(::chess_interfaces::srv::DetectBoard_Response::_error_message_type arg)
  {
    msg_.error_message = std::move(arg);
    return Init_DetectBoard_Response_board_state(msg_);
  }

private:
  ::chess_interfaces::srv::DetectBoard_Response msg_;
};

class Init_DetectBoard_Response_success
{
public:
  Init_DetectBoard_Response_success()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_DetectBoard_Response_error_message success(::chess_interfaces::srv::DetectBoard_Response::_success_type arg)
  {
    msg_.success = std::move(arg);
    return Init_DetectBoard_Response_error_message(msg_);
  }

private:
  ::chess_interfaces::srv::DetectBoard_Response msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::chess_interfaces::srv::DetectBoard_Response>()
{
  return chess_interfaces::srv::builder::Init_DetectBoard_Response_success();
}

}  // namespace chess_interfaces

#endif  // CHESS_INTERFACES__SRV__DETAIL__DETECT_BOARD__BUILDER_HPP_
