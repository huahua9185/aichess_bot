// NOLINT: This file starts with a BOM since it contain non-ASCII characters
// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from chess_interfaces:srv/ExecuteMove.idl
// generated code does not contain a copyright notice

#ifndef CHESS_INTERFACES__SRV__DETAIL__EXECUTE_MOVE__STRUCT_H_
#define CHESS_INTERFACES__SRV__DETAIL__EXECUTE_MOVE__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'chess_move'
#include "chess_interfaces/msg/detail/chess_move__struct.h"

/// Struct defined in srv/ExecuteMove in the package chess_interfaces.
typedef struct chess_interfaces__srv__ExecuteMove_Request
{
  /// 请求
  /// 要执行的象棋移动
  chess_interfaces__msg__ChessMove chess_move;
  /// moveit_msgs/RobotTrajectory trajectory    # 要执行的轨迹
  /// 是否需要确认执行
  bool confirm_execution;
} chess_interfaces__srv__ExecuteMove_Request;

// Struct for a sequence of chess_interfaces__srv__ExecuteMove_Request.
typedef struct chess_interfaces__srv__ExecuteMove_Request__Sequence
{
  chess_interfaces__srv__ExecuteMove_Request * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} chess_interfaces__srv__ExecuteMove_Request__Sequence;


// Constants defined in the message

// Include directives for member types
// Member 'error_message'
#include "rosidl_runtime_c/string.h"

/// Struct defined in srv/ExecuteMove in the package chess_interfaces.
typedef struct chess_interfaces__srv__ExecuteMove_Response
{
  /// 响应
  /// 执行是否成功
  bool success;
  /// 错误信息
  rosidl_runtime_c__String error_message;
  /// 实际执行时间
  float actual_execution_time;
  /// 是否成功吃子
  bool piece_captured;
} chess_interfaces__srv__ExecuteMove_Response;

// Struct for a sequence of chess_interfaces__srv__ExecuteMove_Response.
typedef struct chess_interfaces__srv__ExecuteMove_Response__Sequence
{
  chess_interfaces__srv__ExecuteMove_Response * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} chess_interfaces__srv__ExecuteMove_Response__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // CHESS_INTERFACES__SRV__DETAIL__EXECUTE_MOVE__STRUCT_H_
