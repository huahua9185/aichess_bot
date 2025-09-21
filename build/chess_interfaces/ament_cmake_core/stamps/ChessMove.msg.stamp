# 象棋移动消息
# 用于表示一次象棋移动的完整信息

std_msgs/Header header

# 移动基本信息
int8 from_square        # 起始格子 (0-63)
int8 to_square          # 目标格子 (0-63)
int8 piece_type         # 移动的棋子类型 (1-6)
int8 captured_piece     # 被吃的棋子类型 (0表示无)
int8 promotion          # 兵升变类型 (0表示无)

# 特殊移动标记
bool is_castling        # 是否为易位
bool is_en_passant      # 是否为吃过路兵
bool is_check           # 是否将军
bool is_checkmate       # 是否将死

# 置信度和时间信息
float32 confidence      # 识别置信度 (0.0-1.0)
float32 thinking_time   # AI思考时间 (秒)