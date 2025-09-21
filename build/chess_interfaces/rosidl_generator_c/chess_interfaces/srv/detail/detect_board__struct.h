// NOLINT: This file starts with a BOM since it contain non-ASCII characters
// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from chess_interfaces:srv/DetectBoard.idl
// generated code does not contain a copyright notice

#ifndef CHESS_INTERFACES__SRV__DETAIL__DETECT_BOARD__STRUCT_H_
#define CHESS_INTERFACES__SRV__DETAIL__DETECT_BOARD__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'rgb_image'
// Member 'depth_image'
#include "sensor_msgs/msg/detail/image__struct.h"

/// Struct defined in srv/DetectBoard in the package chess_interfaces.
typedef struct chess_interfaces__srv__DetectBoard_Request
{
  /// 请求
  /// RGB图像
  sensor_msgs__msg__Image rgb_image;
  /// 深度图像
  sensor_msgs__msg__Image depth_image;
  /// 强制重新检测
  bool force_update;
} chess_interfaces__srv__DetectBoard_Request;

// Struct for a sequence of chess_interfaces__srv__DetectBoard_Request.
typedef struct chess_interfaces__srv__DetectBoard_Request__Sequence
{
  chess_interfaces__srv__DetectBoard_Request * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} chess_interfaces__srv__DetectBoard_Request__Sequence;


// Constants defined in the message

// Include directives for member types
// Member 'error_message'
#include "rosidl_runtime_c/string.h"
// Member 'board_state'
#include "chess_interfaces/msg/detail/board_state__struct.h"
// Member 'board_transform'
#include "geometry_msgs/msg/detail/transform_stamped__struct.h"

/// Struct defined in srv/DetectBoard in the package chess_interfaces.
typedef struct chess_interfaces__srv__DetectBoard_Response
{
  /// 响应
  /// 检测是否成功
  bool success;
  /// 错误信息
  rosidl_runtime_c__String error_message;
  /// 检测到的棋盘状态
  chess_interfaces__msg__BoardState board_state;
  /// 棋盘坐标变换
  geometry_msgs__msg__TransformStamped board_transform;
} chess_interfaces__srv__DetectBoard_Response;

// Struct for a sequence of chess_interfaces__srv__DetectBoard_Response.
typedef struct chess_interfaces__srv__DetectBoard_Response__Sequence
{
  chess_interfaces__srv__DetectBoard_Response * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} chess_interfaces__srv__DetectBoard_Response__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // CHESS_INTERFACES__SRV__DETAIL__DETECT_BOARD__STRUCT_H_
