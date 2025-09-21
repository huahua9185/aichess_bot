// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from chess_interfaces:msg/PieceDetection.idl
// generated code does not contain a copyright notice

#ifndef CHESS_INTERFACES__MSG__DETAIL__PIECE_DETECTION__STRUCT_HPP_
#define CHESS_INTERFACES__MSG__DETAIL__PIECE_DETECTION__STRUCT_HPP_

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
// Member 'position_3d'
// Member 'position_2d'
#include "geometry_msgs/msg/detail/point__struct.hpp"
// Member 'roi_image'
#include "sensor_msgs/msg/detail/image__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__chess_interfaces__msg__PieceDetection __attribute__((deprecated))
#else
# define DEPRECATED__chess_interfaces__msg__PieceDetection __declspec(deprecated)
#endif

namespace chess_interfaces
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct PieceDetection_
{
  using Type = PieceDetection_<ContainerAllocator>;

  explicit PieceDetection_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_init),
    position_3d(_init),
    position_2d(_init),
    roi_image(_init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->piece_type = 0;
      this->piece_color = 0;
      this->confidence = 0.0f;
    }
  }

  explicit PieceDetection_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_alloc, _init),
    position_3d(_alloc, _init),
    position_2d(_alloc, _init),
    roi_image(_alloc, _init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->piece_type = 0;
      this->piece_color = 0;
      this->confidence = 0.0f;
    }
  }

  // field types and members
  using _header_type =
    std_msgs::msg::Header_<ContainerAllocator>;
  _header_type header;
  using _position_3d_type =
    geometry_msgs::msg::Point_<ContainerAllocator>;
  _position_3d_type position_3d;
  using _position_2d_type =
    geometry_msgs::msg::Point_<ContainerAllocator>;
  _position_2d_type position_2d;
  using _piece_type_type =
    int8_t;
  _piece_type_type piece_type;
  using _piece_color_type =
    int8_t;
  _piece_color_type piece_color;
  using _confidence_type =
    float;
  _confidence_type confidence;
  using _roi_image_type =
    sensor_msgs::msg::Image_<ContainerAllocator>;
  _roi_image_type roi_image;

  // setters for named parameter idiom
  Type & set__header(
    const std_msgs::msg::Header_<ContainerAllocator> & _arg)
  {
    this->header = _arg;
    return *this;
  }
  Type & set__position_3d(
    const geometry_msgs::msg::Point_<ContainerAllocator> & _arg)
  {
    this->position_3d = _arg;
    return *this;
  }
  Type & set__position_2d(
    const geometry_msgs::msg::Point_<ContainerAllocator> & _arg)
  {
    this->position_2d = _arg;
    return *this;
  }
  Type & set__piece_type(
    const int8_t & _arg)
  {
    this->piece_type = _arg;
    return *this;
  }
  Type & set__piece_color(
    const int8_t & _arg)
  {
    this->piece_color = _arg;
    return *this;
  }
  Type & set__confidence(
    const float & _arg)
  {
    this->confidence = _arg;
    return *this;
  }
  Type & set__roi_image(
    const sensor_msgs::msg::Image_<ContainerAllocator> & _arg)
  {
    this->roi_image = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    chess_interfaces::msg::PieceDetection_<ContainerAllocator> *;
  using ConstRawPtr =
    const chess_interfaces::msg::PieceDetection_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<chess_interfaces::msg::PieceDetection_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<chess_interfaces::msg::PieceDetection_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      chess_interfaces::msg::PieceDetection_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<chess_interfaces::msg::PieceDetection_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      chess_interfaces::msg::PieceDetection_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<chess_interfaces::msg::PieceDetection_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<chess_interfaces::msg::PieceDetection_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<chess_interfaces::msg::PieceDetection_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__chess_interfaces__msg__PieceDetection
    std::shared_ptr<chess_interfaces::msg::PieceDetection_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__chess_interfaces__msg__PieceDetection
    std::shared_ptr<chess_interfaces::msg::PieceDetection_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const PieceDetection_ & other) const
  {
    if (this->header != other.header) {
      return false;
    }
    if (this->position_3d != other.position_3d) {
      return false;
    }
    if (this->position_2d != other.position_2d) {
      return false;
    }
    if (this->piece_type != other.piece_type) {
      return false;
    }
    if (this->piece_color != other.piece_color) {
      return false;
    }
    if (this->confidence != other.confidence) {
      return false;
    }
    if (this->roi_image != other.roi_image) {
      return false;
    }
    return true;
  }
  bool operator!=(const PieceDetection_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct PieceDetection_

// alias to use template instance with default allocator
using PieceDetection =
  chess_interfaces::msg::PieceDetection_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace chess_interfaces

#endif  // CHESS_INTERFACES__MSG__DETAIL__PIECE_DETECTION__STRUCT_HPP_
