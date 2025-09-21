// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from chess_interfaces:msg/ChessMove.idl
// generated code does not contain a copyright notice

#ifndef CHESS_INTERFACES__MSG__DETAIL__CHESS_MOVE__STRUCT_HPP_
#define CHESS_INTERFACES__MSG__DETAIL__CHESS_MOVE__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__chess_interfaces__msg__ChessMove __attribute__((deprecated))
#else
# define DEPRECATED__chess_interfaces__msg__ChessMove __declspec(deprecated)
#endif

namespace chess_interfaces
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct ChessMove_
{
  using Type = ChessMove_<ContainerAllocator>;

  explicit ChessMove_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->from_square = 0;
      this->to_square = 0;
      this->piece_type = 0;
      this->captured_piece = 0;
      this->promotion = 0;
      this->is_castling = false;
      this->is_en_passant = false;
      this->is_check = false;
      this->is_checkmate = false;
      this->confidence = 0.0f;
      this->thinking_time = 0.0f;
    }
  }

  explicit ChessMove_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_alloc, _init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->from_square = 0;
      this->to_square = 0;
      this->piece_type = 0;
      this->captured_piece = 0;
      this->promotion = 0;
      this->is_castling = false;
      this->is_en_passant = false;
      this->is_check = false;
      this->is_checkmate = false;
      this->confidence = 0.0f;
      this->thinking_time = 0.0f;
    }
  }

  // field types and members
  using _header_type =
    std_msgs::msg::Header_<ContainerAllocator>;
  _header_type header;
  using _from_square_type =
    int8_t;
  _from_square_type from_square;
  using _to_square_type =
    int8_t;
  _to_square_type to_square;
  using _piece_type_type =
    int8_t;
  _piece_type_type piece_type;
  using _captured_piece_type =
    int8_t;
  _captured_piece_type captured_piece;
  using _promotion_type =
    int8_t;
  _promotion_type promotion;
  using _is_castling_type =
    bool;
  _is_castling_type is_castling;
  using _is_en_passant_type =
    bool;
  _is_en_passant_type is_en_passant;
  using _is_check_type =
    bool;
  _is_check_type is_check;
  using _is_checkmate_type =
    bool;
  _is_checkmate_type is_checkmate;
  using _confidence_type =
    float;
  _confidence_type confidence;
  using _thinking_time_type =
    float;
  _thinking_time_type thinking_time;

  // setters for named parameter idiom
  Type & set__header(
    const std_msgs::msg::Header_<ContainerAllocator> & _arg)
  {
    this->header = _arg;
    return *this;
  }
  Type & set__from_square(
    const int8_t & _arg)
  {
    this->from_square = _arg;
    return *this;
  }
  Type & set__to_square(
    const int8_t & _arg)
  {
    this->to_square = _arg;
    return *this;
  }
  Type & set__piece_type(
    const int8_t & _arg)
  {
    this->piece_type = _arg;
    return *this;
  }
  Type & set__captured_piece(
    const int8_t & _arg)
  {
    this->captured_piece = _arg;
    return *this;
  }
  Type & set__promotion(
    const int8_t & _arg)
  {
    this->promotion = _arg;
    return *this;
  }
  Type & set__is_castling(
    const bool & _arg)
  {
    this->is_castling = _arg;
    return *this;
  }
  Type & set__is_en_passant(
    const bool & _arg)
  {
    this->is_en_passant = _arg;
    return *this;
  }
  Type & set__is_check(
    const bool & _arg)
  {
    this->is_check = _arg;
    return *this;
  }
  Type & set__is_checkmate(
    const bool & _arg)
  {
    this->is_checkmate = _arg;
    return *this;
  }
  Type & set__confidence(
    const float & _arg)
  {
    this->confidence = _arg;
    return *this;
  }
  Type & set__thinking_time(
    const float & _arg)
  {
    this->thinking_time = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    chess_interfaces::msg::ChessMove_<ContainerAllocator> *;
  using ConstRawPtr =
    const chess_interfaces::msg::ChessMove_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<chess_interfaces::msg::ChessMove_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<chess_interfaces::msg::ChessMove_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      chess_interfaces::msg::ChessMove_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<chess_interfaces::msg::ChessMove_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      chess_interfaces::msg::ChessMove_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<chess_interfaces::msg::ChessMove_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<chess_interfaces::msg::ChessMove_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<chess_interfaces::msg::ChessMove_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__chess_interfaces__msg__ChessMove
    std::shared_ptr<chess_interfaces::msg::ChessMove_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__chess_interfaces__msg__ChessMove
    std::shared_ptr<chess_interfaces::msg::ChessMove_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const ChessMove_ & other) const
  {
    if (this->header != other.header) {
      return false;
    }
    if (this->from_square != other.from_square) {
      return false;
    }
    if (this->to_square != other.to_square) {
      return false;
    }
    if (this->piece_type != other.piece_type) {
      return false;
    }
    if (this->captured_piece != other.captured_piece) {
      return false;
    }
    if (this->promotion != other.promotion) {
      return false;
    }
    if (this->is_castling != other.is_castling) {
      return false;
    }
    if (this->is_en_passant != other.is_en_passant) {
      return false;
    }
    if (this->is_check != other.is_check) {
      return false;
    }
    if (this->is_checkmate != other.is_checkmate) {
      return false;
    }
    if (this->confidence != other.confidence) {
      return false;
    }
    if (this->thinking_time != other.thinking_time) {
      return false;
    }
    return true;
  }
  bool operator!=(const ChessMove_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct ChessMove_

// alias to use template instance with default allocator
using ChessMove =
  chess_interfaces::msg::ChessMove_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace chess_interfaces

#endif  // CHESS_INTERFACES__MSG__DETAIL__CHESS_MOVE__STRUCT_HPP_
