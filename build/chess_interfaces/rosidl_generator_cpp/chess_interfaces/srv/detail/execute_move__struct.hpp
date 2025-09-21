// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from chess_interfaces:srv/ExecuteMove.idl
// generated code does not contain a copyright notice

#ifndef CHESS_INTERFACES__SRV__DETAIL__EXECUTE_MOVE__STRUCT_HPP_
#define CHESS_INTERFACES__SRV__DETAIL__EXECUTE_MOVE__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


// Include directives for member types
// Member 'chess_move'
#include "chess_interfaces/msg/detail/chess_move__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__chess_interfaces__srv__ExecuteMove_Request __attribute__((deprecated))
#else
# define DEPRECATED__chess_interfaces__srv__ExecuteMove_Request __declspec(deprecated)
#endif

namespace chess_interfaces
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct ExecuteMove_Request_
{
  using Type = ExecuteMove_Request_<ContainerAllocator>;

  explicit ExecuteMove_Request_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : chess_move(_init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->confirm_execution = false;
    }
  }

  explicit ExecuteMove_Request_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : chess_move(_alloc, _init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->confirm_execution = false;
    }
  }

  // field types and members
  using _chess_move_type =
    chess_interfaces::msg::ChessMove_<ContainerAllocator>;
  _chess_move_type chess_move;
  using _confirm_execution_type =
    bool;
  _confirm_execution_type confirm_execution;

  // setters for named parameter idiom
  Type & set__chess_move(
    const chess_interfaces::msg::ChessMove_<ContainerAllocator> & _arg)
  {
    this->chess_move = _arg;
    return *this;
  }
  Type & set__confirm_execution(
    const bool & _arg)
  {
    this->confirm_execution = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    chess_interfaces::srv::ExecuteMove_Request_<ContainerAllocator> *;
  using ConstRawPtr =
    const chess_interfaces::srv::ExecuteMove_Request_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<chess_interfaces::srv::ExecuteMove_Request_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<chess_interfaces::srv::ExecuteMove_Request_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      chess_interfaces::srv::ExecuteMove_Request_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<chess_interfaces::srv::ExecuteMove_Request_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      chess_interfaces::srv::ExecuteMove_Request_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<chess_interfaces::srv::ExecuteMove_Request_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<chess_interfaces::srv::ExecuteMove_Request_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<chess_interfaces::srv::ExecuteMove_Request_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__chess_interfaces__srv__ExecuteMove_Request
    std::shared_ptr<chess_interfaces::srv::ExecuteMove_Request_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__chess_interfaces__srv__ExecuteMove_Request
    std::shared_ptr<chess_interfaces::srv::ExecuteMove_Request_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const ExecuteMove_Request_ & other) const
  {
    if (this->chess_move != other.chess_move) {
      return false;
    }
    if (this->confirm_execution != other.confirm_execution) {
      return false;
    }
    return true;
  }
  bool operator!=(const ExecuteMove_Request_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct ExecuteMove_Request_

// alias to use template instance with default allocator
using ExecuteMove_Request =
  chess_interfaces::srv::ExecuteMove_Request_<std::allocator<void>>;

// constant definitions

}  // namespace srv

}  // namespace chess_interfaces


#ifndef _WIN32
# define DEPRECATED__chess_interfaces__srv__ExecuteMove_Response __attribute__((deprecated))
#else
# define DEPRECATED__chess_interfaces__srv__ExecuteMove_Response __declspec(deprecated)
#endif

namespace chess_interfaces
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct ExecuteMove_Response_
{
  using Type = ExecuteMove_Response_<ContainerAllocator>;

  explicit ExecuteMove_Response_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->success = false;
      this->error_message = "";
      this->actual_execution_time = 0.0f;
      this->piece_captured = false;
    }
  }

  explicit ExecuteMove_Response_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : error_message(_alloc)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->success = false;
      this->error_message = "";
      this->actual_execution_time = 0.0f;
      this->piece_captured = false;
    }
  }

  // field types and members
  using _success_type =
    bool;
  _success_type success;
  using _error_message_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _error_message_type error_message;
  using _actual_execution_time_type =
    float;
  _actual_execution_time_type actual_execution_time;
  using _piece_captured_type =
    bool;
  _piece_captured_type piece_captured;

  // setters for named parameter idiom
  Type & set__success(
    const bool & _arg)
  {
    this->success = _arg;
    return *this;
  }
  Type & set__error_message(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->error_message = _arg;
    return *this;
  }
  Type & set__actual_execution_time(
    const float & _arg)
  {
    this->actual_execution_time = _arg;
    return *this;
  }
  Type & set__piece_captured(
    const bool & _arg)
  {
    this->piece_captured = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    chess_interfaces::srv::ExecuteMove_Response_<ContainerAllocator> *;
  using ConstRawPtr =
    const chess_interfaces::srv::ExecuteMove_Response_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<chess_interfaces::srv::ExecuteMove_Response_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<chess_interfaces::srv::ExecuteMove_Response_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      chess_interfaces::srv::ExecuteMove_Response_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<chess_interfaces::srv::ExecuteMove_Response_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      chess_interfaces::srv::ExecuteMove_Response_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<chess_interfaces::srv::ExecuteMove_Response_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<chess_interfaces::srv::ExecuteMove_Response_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<chess_interfaces::srv::ExecuteMove_Response_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__chess_interfaces__srv__ExecuteMove_Response
    std::shared_ptr<chess_interfaces::srv::ExecuteMove_Response_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__chess_interfaces__srv__ExecuteMove_Response
    std::shared_ptr<chess_interfaces::srv::ExecuteMove_Response_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const ExecuteMove_Response_ & other) const
  {
    if (this->success != other.success) {
      return false;
    }
    if (this->error_message != other.error_message) {
      return false;
    }
    if (this->actual_execution_time != other.actual_execution_time) {
      return false;
    }
    if (this->piece_captured != other.piece_captured) {
      return false;
    }
    return true;
  }
  bool operator!=(const ExecuteMove_Response_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct ExecuteMove_Response_

// alias to use template instance with default allocator
using ExecuteMove_Response =
  chess_interfaces::srv::ExecuteMove_Response_<std::allocator<void>>;

// constant definitions

}  // namespace srv

}  // namespace chess_interfaces

namespace chess_interfaces
{

namespace srv
{

struct ExecuteMove
{
  using Request = chess_interfaces::srv::ExecuteMove_Request;
  using Response = chess_interfaces::srv::ExecuteMove_Response;
};

}  // namespace srv

}  // namespace chess_interfaces

#endif  // CHESS_INTERFACES__SRV__DETAIL__EXECUTE_MOVE__STRUCT_HPP_
