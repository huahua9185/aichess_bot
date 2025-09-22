# 国际象棋棋盘状态消息定义
# 包含64个格子的棋盘状态和3D坐标信息

# 时间戳
builtin_interfaces/Time timestamp

# 棋盘状态 - 64个格子，使用标准国际象棋坐标系统 (a1-h8)
# 0: 空格子
# 正数: 白方棋子 (1=王King, 2=后Queen, 3=车Rook, 4=象Bishop, 5=马Knight, 6=兵Pawn)
# 负数: 黑方棋子 (-1=王, -2=后, -3=车, -4=象, -5=马, -6=兵)
int8[64] board_squares

# FEN (Forsyth-Edwards Notation) 棋局表示法字符串
string fen_notation

# 轮到谁下棋 (true=白方, false=黑方)
bool white_to_move

# 每个格子的3D世界坐标 (相对于机械臂base_link)
geometry_msgs/Point[64] square_positions

# 棋盘检测置信度 (0.0-1.0)
float32 detection_confidence

# 是否检测到有效棋盘
bool board_detected

# 检测到的棋子数量
int32 piece_count

# 棋盘朝向角度 (弧度)
float32 board_orientation

# ArUco标记检测状态
bool aruco_detected
geometry_msgs/Pose aruco_pose