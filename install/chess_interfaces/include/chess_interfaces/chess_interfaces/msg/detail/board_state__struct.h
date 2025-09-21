// NOLINT: This file starts with a BOM since it contain non-ASCII characters
// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from chess_interfaces:msg/BoardState.idl
// generated code does not contain a copyright notice

#ifndef CHESS_INTERFACES__MSG__DETAIL__BOARD_STATE__STRUCT_H_
#define CHESS_INTERFACES__MSG__DETAIL__BOARD_STATE__STRUCT_H_

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
// Member 'square_positions_3d'
#include "geometry_msgs/msg/detail/point__struct.h"

/// Struct defined in msg/BoardState in the package chess_interfaces.
/**
  * 棋盘状态消息
  * 用于表示8x8象棋棋盘的完整状态
 */
typedef struct chess_interfaces__msg__BoardState
{
  std_msgs__msg__Header header;
  /// 棋盘格子状态 (64个格子，按行列顺序)
  /// 0=空格 1-6=白方棋子 -1到-6=黑方棋子
  /// 1=王 2=后 3=车 4=象 5=马 6=兵
  int8_t board_squares[64];
  /// 每个格子的3D坐标位置
  geometry_msgs__msg__Point square_positions_3d[64];
  /// 游戏状态信息
  /// 是否轮到白方
  bool white_to_move;
  /// 易位权利 [白王侧, 白后侧, 黑王侧, 黑后侧]
  bool castling_rights[4];
  /// 吃过路兵目标格子 (-1表示无)
  int8_t en_passant_square;
  /// 半回合计数
  int32_t halfmove_clock;
  /// 完整回合数
  int32_t fullmove_number;
} chess_interfaces__msg__BoardState;

// Struct for a sequence of chess_interfaces__msg__BoardState.
typedef struct chess_interfaces__msg__BoardState__Sequence
{
  chess_interfaces__msg__BoardState * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} chess_interfaces__msg__BoardState__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // CHESS_INTERFACES__MSG__DETAIL__BOARD_STATE__STRUCT_H_
