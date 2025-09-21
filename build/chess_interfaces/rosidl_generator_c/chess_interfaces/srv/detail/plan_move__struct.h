// NOLINT: This file starts with a BOM since it contain non-ASCII characters
// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from chess_interfaces:srv/PlanMove.idl
// generated code does not contain a copyright notice

#ifndef CHESS_INTERFACES__SRV__DETAIL__PLAN_MOVE__STRUCT_H_
#define CHESS_INTERFACES__SRV__DETAIL__PLAN_MOVE__STRUCT_H_

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

/// Struct defined in srv/PlanMove in the package chess_interfaces.
typedef struct chess_interfaces__srv__PlanMove_Request
{
  /// 请求
  /// 要执行的象棋移动
  chess_interfaces__msg__ChessMove chess_move;
  /// 速度系数 (0.1-1.0)
  float speed_factor;
  /// 是否避开其他棋子
  bool avoid_pieces;
  /// 是否使用安全接近路径
  bool use_safe_approach;
} chess_interfaces__srv__PlanMove_Request;

// Struct for a sequence of chess_interfaces__srv__PlanMove_Request.
typedef struct chess_interfaces__srv__PlanMove_Request__Sequence
{
  chess_interfaces__srv__PlanMove_Request * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} chess_interfaces__srv__PlanMove_Request__Sequence;


// Constants defined in the message

// Include directives for member types
// Member 'error_message'
#include "rosidl_runtime_c/string.h"

/// Struct defined in srv/PlanMove in the package chess_interfaces.
typedef struct chess_interfaces__srv__PlanMove_Response
{
  /// 响应
  /// 规划是否成功
  bool success;
  /// 错误信息
  rosidl_runtime_c__String error_message;
  /// moveit_msgs/RobotTrajectory trajectory    # 规划的轨迹
  /// 预计执行时间
  float execution_time;
} chess_interfaces__srv__PlanMove_Response;

// Struct for a sequence of chess_interfaces__srv__PlanMove_Response.
typedef struct chess_interfaces__srv__PlanMove_Response__Sequence
{
  chess_interfaces__srv__PlanMove_Response * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} chess_interfaces__srv__PlanMove_Response__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // CHESS_INTERFACES__SRV__DETAIL__PLAN_MOVE__STRUCT_H_
