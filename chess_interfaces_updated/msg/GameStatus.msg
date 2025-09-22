# 游戏状态消息定义
# 包含当前游戏的完整状态信息

# 时间戳
builtin_interfaces/Time timestamp

# 游戏状态枚举
uint8 GAME_IDLE = 0
uint8 GAME_INITIALIZING = 1
uint8 GAME_WAITING_HUMAN = 2
uint8 GAME_HUMAN_THINKING = 3
uint8 GAME_AI_THINKING = 4
uint8 GAME_EXECUTING_MOVE = 5
uint8 GAME_PAUSED = 6
uint8 GAME_FINISHED = 7
uint8 GAME_ERROR = 8

uint8 game_state

# 游戏结果
uint8 RESULT_ONGOING = 0
uint8 RESULT_HUMAN_WIN = 1
uint8 RESULT_AI_WIN = 2
uint8 RESULT_DRAW = 3
uint8 RESULT_ABORT = 4

uint8 game_result

# 当前轮到哪方
bool human_turn

# 回合数
int32 turn_number

# 移动历史数量
int32 moves_count

# 最后一步移动
ChessMove last_move

# AI思考时间 (秒)
float32 ai_thinking_time

# 人类思考时间 (秒)
float32 human_thinking_time

# 游戏总时长 (秒)
float32 total_game_time

# 当前棋局评估分数 (-1000到1000，正数表示AI优势)
float32 position_score

# AI引擎状态
string engine_status

# 系统状态信息
string[] system_messages

# 错误信息
string error_message

# 游戏难度级别 (1-10)
int32 difficulty_level

# 是否使用仿真模式
bool simulation_mode

# 安全检查状态
bool safety_ok

# 机械臂状态
string arm_status

# 相机状态
string camera_status