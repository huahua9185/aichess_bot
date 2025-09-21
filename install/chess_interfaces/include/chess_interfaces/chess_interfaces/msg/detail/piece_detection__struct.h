// NOLINT: This file starts with a BOM since it contain non-ASCII characters
// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from chess_interfaces:msg/PieceDetection.idl
// generated code does not contain a copyright notice

#ifndef CHESS_INTERFACES__MSG__DETAIL__PIECE_DETECTION__STRUCT_H_
#define CHESS_INTERFACES__MSG__DETAIL__PIECE_DETECTION__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__struct.h"
// Member 'position_3d'
// Member 'position_2d'
#include "geometry_msgs/msg/detail/point__struct.h"
// Member 'roi_image'
#include "sensor_msgs/msg/detail/image__struct.h"

/// Struct defined in msg/PieceDetection in the package chess_interfaces.
/**
  * 棋子检测结果消息
  * 用于表示单个棋子的检测结果
 */
typedef struct chess_interfaces__msg__PieceDetection
{
  std_msgs__msg__Header header;
  /// 棋子位置信息
  /// 3D世界坐标
  geometry_msgs__msg__Point position_3d;
  /// 2D图像坐标
  geometry_msgs__msg__Point position_2d;
  /// 棋子属性
  /// 棋子类型 (1=王, 2=后, 3=车, 4=象, 5=马, 6=兵)
  int8_t piece_type;
  /// 棋子颜色 (1=白, -1=黑, 0=未知)
  int8_t piece_color;
  /// 检测置信度 (0.0-1.0)
  float confidence;
  /// 图像数据
  /// 棋子区域图像
  sensor_msgs__msg__Image roi_image;
} chess_interfaces__msg__PieceDetection;

// Struct for a sequence of chess_interfaces__msg__PieceDetection.
typedef struct chess_interfaces__msg__PieceDetection__Sequence
{
  chess_interfaces__msg__PieceDetection * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} chess_interfaces__msg__PieceDetection__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // CHESS_INTERFACES__MSG__DETAIL__PIECE_DETECTION__STRUCT_H_
