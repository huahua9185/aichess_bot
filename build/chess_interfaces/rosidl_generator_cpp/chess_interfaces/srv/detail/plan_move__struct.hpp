// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from chess_interfaces:srv/PlanMove.idl
// generated code does not contain a copyright notice

#ifndef CHESS_INTERFACES__SRV__DETAIL__PLAN_MOVE__STRUCT_HPP_
#define CHESS_INTERFACES__SRV__DETAIL__PLAN_MOVE__STRUCT_HPP_

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
# define DEPRECATED__chess_interfaces__srv__PlanMove_Request __attribute__((deprecated))
#else
# define DEPRECATED__chess_interfaces__srv__PlanMove_Request __declspec(deprecated)
#endif

namespace chess_interfaces
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct PlanMove_Request_
{
  using Type = PlanMove_Request_<ContainerAllocator>;

  explicit PlanMove_Request_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : chess_move(_init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->speed_factor = 0.0f;
      this->avoid_pieces = false;
      this->use_safe_approach = false;
    }
  }

  explicit PlanMove_Request_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : chess_move(_alloc, _init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->speed_factor = 0.0f;
      this->avoid_pieces = false;
      this->use_safe_approach = false;
    }
  }

  // field types and members
  using _chess_move_type =
    chess_interfaces::msg::ChessMove_<ContainerAllocator>;
  _chess_move_type chess_move;
  using _speed_factor_type =
    float;
  _speed_factor_type speed_factor;
  using _avoid_pieces_type =
    bool;
  _avoid_pieces_type avoid_pieces;
  using _use_safe_approach_type =
    bool;
  _use_safe_approach_type use_safe_approach;

  // setters for named parameter idiom
  Type & set__chess_move(
    const chess_interfaces::msg::ChessMove_<ContainerAllocator> & _arg)
  {
    this->chess_move = _arg;
    return *this;
  }
  Type & set__speed_factor(
    const float & _arg)
  {
    this->speed_factor = _arg;
    return *this;
  }
  Type & set__avoid_pieces(
    const bool & _arg)
  {
    this->avoid_pieces = _arg;
    return *this;
  }
  Type & set__use_safe_approach(
    const bool & _arg)
  {
    this->use_safe_approach = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    chess_interfaces::srv::PlanMove_Request_<ContainerAllocator> *;
  using ConstRawPtr =
    const chess_interfaces::srv::PlanMove_Request_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<chess_interfaces::srv::PlanMove_Request_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<chess_interfaces::srv::PlanMove_Request_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      chess_interfaces::srv::PlanMove_Request_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<chess_interfaces::srv::PlanMove_Request_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      chess_interfaces::srv::PlanMove_Request_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<chess_interfaces::srv::PlanMove_Request_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<chess_interfaces::srv::PlanMove_Request_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<chess_interfaces::srv::PlanMove_Request_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__chess_interfaces__srv__PlanMove_Request
    std::shared_ptr<chess_interfaces::srv::PlanMove_Request_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__chess_interfaces__srv__PlanMove_Request
    std::shared_ptr<chess_interfaces::srv::PlanMove_Request_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const PlanMove_Request_ & other) const
  {
    if (this->chess_move != other.chess_move) {
      return false;
    }
    if (this->speed_factor != other.speed_factor) {
      return false;
    }
    if (this->avoid_pieces != other.avoid_pieces) {
      return false;
    }
    if (this->use_safe_approach != other.use_safe_approach) {
      return false;
    }
    return true;
  }
  bool operator!=(const PlanMove_Request_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct PlanMove_Request_

// alias to use template instance with default allocator
using PlanMove_Request =
  chess_interfaces::srv::PlanMove_Request_<std::allocator<void>>;

// constant definitions

}  // namespace srv

}  // namespace chess_interfaces


#ifndef _WIN32
# define DEPRECATED__chess_interfaces__srv__PlanMove_Response __attribute__((deprecated))
#else
# define DEPRECATED__chess_interfaces__srv__PlanMove_Response __declspec(deprecated)
#endif

namespace chess_interfaces
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct PlanMove_Response_
{
  using Type = PlanMove_Response_<ContainerAllocator>;

  explicit PlanMove_Response_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->success = false;
      this->error_message = "";
      this->execution_time = 0.0f;
    }
  }

  explicit PlanMove_Response_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : error_message(_alloc)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->success = false;
      this->error_message = "";
      this->execution_time = 0.0f;
    }
  }

  // field types and members
  using _success_type =
    bool;
  _success_type success;
  using _error_message_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _error_message_type error_message;
  using _execution_time_type =
    float;
  _execution_time_type execution_time;

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
  Type & set__execution_time(
    const float & _arg)
  {
    this->execution_time = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    chess_interfaces::srv::PlanMove_Response_<ContainerAllocator> *;
  using ConstRawPtr =
    const chess_interfaces::srv::PlanMove_Response_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<chess_interfaces::srv::PlanMove_Response_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<chess_interfaces::srv::PlanMove_Response_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      chess_interfaces::srv::PlanMove_Response_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<chess_interfaces::srv::PlanMove_Response_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      chess_interfaces::srv::PlanMove_Response_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<chess_interfaces::srv::PlanMove_Response_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<chess_interfaces::srv::PlanMove_Response_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<chess_interfaces::srv::PlanMove_Response_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__chess_interfaces__srv__PlanMove_Response
    std::shared_ptr<chess_interfaces::srv::PlanMove_Response_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__chess_interfaces__srv__PlanMove_Response
    std::shared_ptr<chess_interfaces::srv::PlanMove_Response_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const PlanMove_Response_ & other) const
  {
    if (this->success != other.success) {
      return false;
    }
    if (this->error_message != other.error_message) {
      return false;
    }
    if (this->execution_time != other.execution_time) {
      return false;
    }
    return true;
  }
  bool operator!=(const PlanMove_Response_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct PlanMove_Response_

// alias to use template instance with default allocator
using PlanMove_Response =
  chess_interfaces::srv::PlanMove_Response_<std::allocator<void>>;

// constant definitions

}  // namespace srv

}  // namespace chess_interfaces

namespace chess_interfaces
{

namespace srv
{

struct PlanMove
{
  using Request = chess_interfaces::srv::PlanMove_Request;
  using Response = chess_interfaces::srv::PlanMove_Response;
};

}  // namespace srv

}  // namespace chess_interfaces

#endif  // CHESS_INTERFACES__SRV__DETAIL__PLAN_MOVE__STRUCT_HPP_
