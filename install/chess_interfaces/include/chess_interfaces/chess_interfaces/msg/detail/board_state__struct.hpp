// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from chess_interfaces:msg/BoardState.idl
// generated code does not contain a copyright notice

#ifndef CHESS_INTERFACES__MSG__DETAIL__BOARD_STATE__STRUCT_HPP_
#define CHESS_INTERFACES__MSG__DETAIL__BOARD_STATE__STRUCT_HPP_

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
// Member 'square_positions_3d'
#include "geometry_msgs/msg/detail/point__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__chess_interfaces__msg__BoardState __attribute__((deprecated))
#else
# define DEPRECATED__chess_interfaces__msg__BoardState __declspec(deprecated)
#endif

namespace chess_interfaces
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct BoardState_
{
  using Type = BoardState_<ContainerAllocator>;

  explicit BoardState_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      std::fill<typename std::array<int8_t, 64>::iterator, int8_t>(this->board_squares.begin(), this->board_squares.end(), 0);
      this->square_positions_3d.fill(geometry_msgs::msg::Point_<ContainerAllocator>{_init});
      this->white_to_move = false;
      std::fill<typename std::array<bool, 4>::iterator, bool>(this->castling_rights.begin(), this->castling_rights.end(), false);
      this->en_passant_square = 0;
      this->halfmove_clock = 0l;
      this->fullmove_number = 0l;
    }
  }

  explicit BoardState_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_alloc, _init),
    board_squares(_alloc),
    square_positions_3d(_alloc),
    castling_rights(_alloc)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      std::fill<typename std::array<int8_t, 64>::iterator, int8_t>(this->board_squares.begin(), this->board_squares.end(), 0);
      this->square_positions_3d.fill(geometry_msgs::msg::Point_<ContainerAllocator>{_alloc, _init});
      this->white_to_move = false;
      std::fill<typename std::array<bool, 4>::iterator, bool>(this->castling_rights.begin(), this->castling_rights.end(), false);
      this->en_passant_square = 0;
      this->halfmove_clock = 0l;
      this->fullmove_number = 0l;
    }
  }

  // field types and members
  using _header_type =
    std_msgs::msg::Header_<ContainerAllocator>;
  _header_type header;
  using _board_squares_type =
    std::array<int8_t, 64>;
  _board_squares_type board_squares;
  using _square_positions_3d_type =
    std::array<geometry_msgs::msg::Point_<ContainerAllocator>, 64>;
  _square_positions_3d_type square_positions_3d;
  using _white_to_move_type =
    bool;
  _white_to_move_type white_to_move;
  using _castling_rights_type =
    std::array<bool, 4>;
  _castling_rights_type castling_rights;
  using _en_passant_square_type =
    int8_t;
  _en_passant_square_type en_passant_square;
  using _halfmove_clock_type =
    int32_t;
  _halfmove_clock_type halfmove_clock;
  using _fullmove_number_type =
    int32_t;
  _fullmove_number_type fullmove_number;

  // setters for named parameter idiom
  Type & set__header(
    const std_msgs::msg::Header_<ContainerAllocator> & _arg)
  {
    this->header = _arg;
    return *this;
  }
  Type & set__board_squares(
    const std::array<int8_t, 64> & _arg)
  {
    this->board_squares = _arg;
    return *this;
  }
  Type & set__square_positions_3d(
    const std::array<geometry_msgs::msg::Point_<ContainerAllocator>, 64> & _arg)
  {
    this->square_positions_3d = _arg;
    return *this;
  }
  Type & set__white_to_move(
    const bool & _arg)
  {
    this->white_to_move = _arg;
    return *this;
  }
  Type & set__castling_rights(
    const std::array<bool, 4> & _arg)
  {
    this->castling_rights = _arg;
    return *this;
  }
  Type & set__en_passant_square(
    const int8_t & _arg)
  {
    this->en_passant_square = _arg;
    return *this;
  }
  Type & set__halfmove_clock(
    const int32_t & _arg)
  {
    this->halfmove_clock = _arg;
    return *this;
  }
  Type & set__fullmove_number(
    const int32_t & _arg)
  {
    this->fullmove_number = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    chess_interfaces::msg::BoardState_<ContainerAllocator> *;
  using ConstRawPtr =
    const chess_interfaces::msg::BoardState_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<chess_interfaces::msg::BoardState_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<chess_interfaces::msg::BoardState_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      chess_interfaces::msg::BoardState_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<chess_interfaces::msg::BoardState_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      chess_interfaces::msg::BoardState_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<chess_interfaces::msg::BoardState_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<chess_interfaces::msg::BoardState_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<chess_interfaces::msg::BoardState_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__chess_interfaces__msg__BoardState
    std::shared_ptr<chess_interfaces::msg::BoardState_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__chess_interfaces__msg__BoardState
    std::shared_ptr<chess_interfaces::msg::BoardState_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const BoardState_ & other) const
  {
    if (this->header != other.header) {
      return false;
    }
    if (this->board_squares != other.board_squares) {
      return false;
    }
    if (this->square_positions_3d != other.square_positions_3d) {
      return false;
    }
    if (this->white_to_move != other.white_to_move) {
      return false;
    }
    if (this->castling_rights != other.castling_rights) {
      return false;
    }
    if (this->en_passant_square != other.en_passant_square) {
      return false;
    }
    if (this->halfmove_clock != other.halfmove_clock) {
      return false;
    }
    if (this->fullmove_number != other.fullmove_number) {
      return false;
    }
    return true;
  }
  bool operator!=(const BoardState_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct BoardState_

// alias to use template instance with default allocator
using BoardState =
  chess_interfaces::msg::BoardState_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace chess_interfaces

#endif  // CHESS_INTERFACES__MSG__DETAIL__BOARD_STATE__STRUCT_HPP_
