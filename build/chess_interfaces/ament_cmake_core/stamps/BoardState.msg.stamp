# 棋盘状态消息
# 用于表示8x8象棋棋盘的完整状态

std_msgs/Header header

# 棋盘格子状态 (64个格子，按行列顺序)
# 0=空格 1-6=白方棋子 -1到-6=黑方棋子
# 1=王 2=后 3=车 4=象 5=马 6=兵
int8[64] board_squares

# 每个格子的3D坐标位置
geometry_msgs/Point[64] square_positions_3d

# 游戏状态信息
bool white_to_move              # 是否轮到白方
bool[4] castling_rights         # 易位权利 [白王侧, 白后侧, 黑王侧, 黑后侧]
int8 en_passant_square          # 吃过路兵目标格子 (-1表示无)
int32 halfmove_clock            # 半回合计数
int32 fullmove_number           # 完整回合数