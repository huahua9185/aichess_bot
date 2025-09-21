// NOLINT: This file starts with a BOM since it contain non-ASCII characters
// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from chess_interfaces:msg/ChessMove.idl
// generated code does not contain a copyright notice

#ifndef CHESS_INTERFACES__MSG__DETAIL__CHESS_MOVE__STRUCT_H_
#define CHESS_INTERFACES__MSG__DETAIL__CHESS_MOVE__STRUCT_H_

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

/// Struct defined in msg/ChessMove in the package chess_interfaces.
/**
  * 象棋移动消息
  * 用于表示一次象棋移动的完整信息
 */
typedef struct chess_interfaces__msg__ChessMove
{
  std_msgs__msg__Header header;
  /// 移动基本信息
  /// 起始格子 (0-63)
  int8_t from_square;
  /// 目标格子 (0-63)
  int8_t to_square;
  /// 移动的棋子类型 (1-6)
  int8_t piece_type;
  /// 被吃的棋子类型 (0表示无)
  int8_t captured_piece;
  /// 兵升变类型 (0表示无)
  int8_t promotion;
  /// 特殊移动标记
  /// 是否为易位
  bool is_castling;
  /// 是否为吃过路兵
  bool is_en_passant;
  /// 是否将军
  bool is_check;
  /// 是否将死
  bool is_checkmate;
  /// 置信度和时间信息
  /// 识别置信度 (0.0-1.0)
  float confidence;
  /// AI思考时间 (秒)
  float thinking_time;
} chess_interfaces__msg__ChessMove;

// Struct for a sequence of chess_interfaces__msg__ChessMove.
typedef struct chess_interfaces__msg__ChessMove__Sequence
{
  chess_interfaces__msg__ChessMove * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} chess_interfaces__msg__ChessMove__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // CHESS_INTERFACES__MSG__DETAIL__CHESS_MOVE__STRUCT_H_
