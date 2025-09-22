# 国际象棋移动指令消息定义
# 包含起止位置和特殊规则处理

# 时间戳
builtin_interfaces/Time timestamp

# 移动类型
uint8 NORMAL_MOVE = 0
uint8 CAPTURE_MOVE = 1
uint8 CASTLE_MOVE = 2          # 王车易位
uint8 EN_PASSANT_MOVE = 3      # 吃过路兵
uint8 PROMOTION_MOVE = 4       # 兵升变
uint8 INVALID_MOVE = 255

uint8 move_type

# 起始位置 (国际象棋坐标，如"e2", "e4")
string from_square

# 目标位置
string to_square

# UCI格式移动表示 (如"e2e4", "e7e8q")
string uci_move

# 代数记号 (如"e4", "Nf3", "O-O")
string algebraic_notation

# 起始位置索引 (0-63)
int32 from_square_index

# 目标位置索引 (0-63)
int32 to_square_index

# 移动的棋子类型 (1=王, 2=后, 3=车, 4=象, 5=马, 6=兵)
int8 piece_type

# 被吃的棋子类型 (如果是吃子，0=无)
int8 captured_piece_type

# 兵升变的棋子类型 (2=后, 3=车, 4=象, 5=马，0=无升变)
int8 promotion_piece_type

# 移动评分 (来自Stockfish引擎，单位厘兵)
float32 evaluation_score

# 移动后的局面评估
float32 position_evaluation

# 是否为将军移动
bool gives_check

# 是否为将死移动
bool gives_checkmate

# 移动的3D路径点 (用于机械臂规划)
geometry_msgs/Point[] trajectory_points

# 预计执行时间 (秒)
float32 estimated_duration

# 抓取力度设置 (牛顿)
float32 grip_force