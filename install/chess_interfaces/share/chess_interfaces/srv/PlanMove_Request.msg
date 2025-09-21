# 运动规划服务
# 为指定的象棋移动规划机械臂轨迹

# 请求
ChessMove chess_move           # 要执行的象棋移动
float32 speed_factor          # 速度系数 (0.1-1.0)
bool avoid_pieces             # 是否避开其他棋子
bool use_safe_approach        # 是否使用安全接近路径

