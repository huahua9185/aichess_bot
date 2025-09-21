// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from chess_interfaces:srv/DetectBoard.idl
// generated code does not contain a copyright notice

#ifndef CHESS_INTERFACES__SRV__DETAIL__DETECT_BOARD__STRUCT_HPP_
#define CHESS_INTERFACES__SRV__DETAIL__DETECT_BOARD__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


// Include directives for member types
// Member 'rgb_image'
// Member 'depth_image'
#include "sensor_msgs/msg/detail/image__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__chess_interfaces__srv__DetectBoard_Request __attribute__((deprecated))
#else
# define DEPRECATED__chess_interfaces__srv__DetectBoard_Request __declspec(deprecated)
#endif

namespace chess_interfaces
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct DetectBoard_Request_
{
  using Type = DetectBoard_Request_<ContainerAllocator>;

  explicit DetectBoard_Request_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : rgb_image(_init),
    depth_image(_init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->force_update = false;
    }
  }

  explicit DetectBoard_Request_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : rgb_image(_alloc, _init),
    depth_image(_alloc, _init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->force_update = false;
    }
  }

  // field types and members
  using _rgb_image_type =
    sensor_msgs::msg::Image_<ContainerAllocator>;
  _rgb_image_type rgb_image;
  using _depth_image_type =
    sensor_msgs::msg::Image_<ContainerAllocator>;
  _depth_image_type depth_image;
  using _force_update_type =
    bool;
  _force_update_type force_update;

  // setters for named parameter idiom
  Type & set__rgb_image(
    const sensor_msgs::msg::Image_<ContainerAllocator> & _arg)
  {
    this->rgb_image = _arg;
    return *this;
  }
  Type & set__depth_image(
    const sensor_msgs::msg::Image_<ContainerAllocator> & _arg)
  {
    this->depth_image = _arg;
    return *this;
  }
  Type & set__force_update(
    const bool & _arg)
  {
    this->force_update = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    chess_interfaces::srv::DetectBoard_Request_<ContainerAllocator> *;
  using ConstRawPtr =
    const chess_interfaces::srv::DetectBoard_Request_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<chess_interfaces::srv::DetectBoard_Request_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<chess_interfaces::srv::DetectBoard_Request_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      chess_interfaces::srv::DetectBoard_Request_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<chess_interfaces::srv::DetectBoard_Request_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      chess_interfaces::srv::DetectBoard_Request_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<chess_interfaces::srv::DetectBoard_Request_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<chess_interfaces::srv::DetectBoard_Request_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<chess_interfaces::srv::DetectBoard_Request_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__chess_interfaces__srv__DetectBoard_Request
    std::shared_ptr<chess_interfaces::srv::DetectBoard_Request_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__chess_interfaces__srv__DetectBoard_Request
    std::shared_ptr<chess_interfaces::srv::DetectBoard_Request_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const DetectBoard_Request_ & other) const
  {
    if (this->rgb_image != other.rgb_image) {
      return false;
    }
    if (this->depth_image != other.depth_image) {
      return false;
    }
    if (this->force_update != other.force_update) {
      return false;
    }
    return true;
  }
  bool operator!=(const DetectBoard_Request_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct DetectBoard_Request_

// alias to use template instance with default allocator
using DetectBoard_Request =
  chess_interfaces::srv::DetectBoard_Request_<std::allocator<void>>;

// constant definitions

}  // namespace srv

}  // namespace chess_interfaces


// Include directives for member types
// Member 'board_state'
#include "chess_interfaces/msg/detail/board_state__struct.hpp"
// Member 'board_transform'
#include "geometry_msgs/msg/detail/transform_stamped__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__chess_interfaces__srv__DetectBoard_Response __attribute__((deprecated))
#else
# define DEPRECATED__chess_interfaces__srv__DetectBoard_Response __declspec(deprecated)
#endif

namespace chess_interfaces
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct DetectBoard_Response_
{
  using Type = DetectBoard_Response_<ContainerAllocator>;

  explicit DetectBoard_Response_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : board_state(_init),
    board_transform(_init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->success = false;
      this->error_message = "";
    }
  }

  explicit DetectBoard_Response_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : error_message(_alloc),
    board_state(_alloc, _init),
    board_transform(_alloc, _init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->success = false;
      this->error_message = "";
    }
  }

  // field types and members
  using _success_type =
    bool;
  _success_type success;
  using _error_message_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _error_message_type error_message;
  using _board_state_type =
    chess_interfaces::msg::BoardState_<ContainerAllocator>;
  _board_state_type board_state;
  using _board_transform_type =
    geometry_msgs::msg::TransformStamped_<ContainerAllocator>;
  _board_transform_type board_transform;

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
  Type & set__board_state(
    const chess_interfaces::msg::BoardState_<ContainerAllocator> & _arg)
  {
    this->board_state = _arg;
    return *this;
  }
  Type & set__board_transform(
    const geometry_msgs::msg::TransformStamped_<ContainerAllocator> & _arg)
  {
    this->board_transform = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    chess_interfaces::srv::DetectBoard_Response_<ContainerAllocator> *;
  using ConstRawPtr =
    const chess_interfaces::srv::DetectBoard_Response_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<chess_interfaces::srv::DetectBoard_Response_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<chess_interfaces::srv::DetectBoard_Response_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      chess_interfaces::srv::DetectBoard_Response_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<chess_interfaces::srv::DetectBoard_Response_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      chess_interfaces::srv::DetectBoard_Response_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<chess_interfaces::srv::DetectBoard_Response_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<chess_interfaces::srv::DetectBoard_Response_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<chess_interfaces::srv::DetectBoard_Response_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__chess_interfaces__srv__DetectBoard_Response
    std::shared_ptr<chess_interfaces::srv::DetectBoard_Response_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__chess_interfaces__srv__DetectBoard_Response
    std::shared_ptr<chess_interfaces::srv::DetectBoard_Response_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const DetectBoard_Response_ & other) const
  {
    if (this->success != other.success) {
      return false;
    }
    if (this->error_message != other.error_message) {
      return false;
    }
    if (this->board_state != other.board_state) {
      return false;
    }
    if (this->board_transform != other.board_transform) {
      return false;
    }
    return true;
  }
  bool operator!=(const DetectBoard_Response_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct DetectBoard_Response_

// alias to use template instance with default allocator
using DetectBoard_Response =
  chess_interfaces::srv::DetectBoard_Response_<std::allocator<void>>;

// constant definitions

}  // namespace srv

}  // namespace chess_interfaces

namespace chess_interfaces
{

namespace srv
{

struct DetectBoard
{
  using Request = chess_interfaces::srv::DetectBoard_Request;
  using Response = chess_interfaces::srv::DetectBoard_Response;
};

}  // namespace srv

}  // namespace chess_interfaces

#endif  // CHESS_INTERFACES__SRV__DETAIL__DETECT_BOARD__STRUCT_HPP_
