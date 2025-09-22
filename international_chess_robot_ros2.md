# 基于ROS2的国际象棋机器人项目实施规划

## 项目概述

**项目目标**：基于ROS2构建智能国际象棋机器人系统，实现全自动国际象棋对弈功能

**核心技术栈**：
- **ROS2 Humble** (主要框架)
- **Jetson Nano 4GB** (边缘计算平台)  
- **DOFBOT Pro** (6DOF机械臂)
- **DABAI DCW2** (深度相机)
- **MoveIt2** (运动规划)
- **OpenCV + AI模型** (计算机视觉)
- **Stockfish引擎** (国际象棋AI)

## 国际象棋特定要求

### 棋盘规格
- **标准8×8棋盘**：64个方格，黑白相间
- **棋盘尺寸**：建议40cm×40cm，每格5cm×5cm
- **棋子高度**：王4.5cm，兵2.5cm，便于机械臂抓取
- **材质要求**：木质或塑料，表面无反光，利于视觉识别

### 棋子识别系统
- **6种棋子类型**：王(King)、后(Queen)、车(Rook)、象(Bishop)、马(Knight)、兵(Pawn)
- **黑白双方**：共12种不同的棋子-颜色组合
- **特殊形状识别**：每种棋子的独特轮廓特征

### 国际象棋规则实现
- **特殊走法**：王车易位(Castling)、吃过路兵(En Passant)、兵升变(Promotion)
- **胜负判定**：将军(Check)、将死(Checkmate)、僵局(Stalemate)
- **和棋规则**：50回合规则、三次重复局面、不可能将死

## ROS2架构设计

### 系统节点架构
```
┌─────────────────────────────────────────────────────────────┐
│                        ROS2 网络                             │
├─────────────────────────────────────────────────────────────┤
│  camera_node  │  vision_node  │  chess_engine │  arm_node    │
│   (传感器)     │   (视觉AI)    │  (国际象棋)   │  (运动控制)   │
└─────────────────────────────────────────────────────────────┘
      ↓              ↓            ↓           ↓
   RGB+深度图     棋盘状态      Stockfish     机械臂控制
```

### 核心ROS2包结构
```
chess_robot_ws/
├── src/
│   ├── chess_interfaces/         # 自定义消息和服务
│   ├── chess_camera/            # 相机驱动和数据发布
│   ├── chess_vision/            # 视觉识别和AI推理
│   ├── chess_engine/            # Stockfish国际象棋引擎
│   ├── chess_arm/               # 机械臂控制
│   ├── chess_planner/           # MoveIt2运动规划
│   ├── chess_coordinator/       # 系统协调和状态机
│   └── chess_ui/                # Web用户界面
├── config/                      # 配置文件
├── launch/                      # 启动文件
└── params/                      # 参数文件
```

## 自定义ROS2接口设计

### 消息类型 (chess_interfaces/msg/)

**BoardState.msg** - 国际象棋棋盘状态
```
Header header
int8[64] board_squares  # 0=空, 1-6=白方棋子, -1到-6=黑方棋子
# 棋子编码：1=King, 2=Queen, 3=Rook, 4=Bishop, 5=Knight, 6=Pawn
geometry_msgs/Point[64] square_positions_3d  # 每个格子的3D坐标
bool white_to_move
bool castling_rights[4]  # KQkq (白王侧易位、白后侧易位、黑王侧易位、黑后侧易位)
int8 en_passant_square   # -1表示无过路兵目标格
int32 halfmove_clock     # 50回合规则计数
int32 fullmove_number    # 总移动回合数
string fen_notation      # 标准FEN记录
```

**ChessMove.msg** - 国际象棋移动
```
Header header
int8 from_square    # 起始格子 (0-63，a1=0, h8=63)
int8 to_square      # 目标格子
int8 piece_type     # 移动的棋子类型
int8 captured_piece # 被吃的棋子(0表示无)
int8 promotion      # 兵升变类型(0=无, 2=Queen, 3=Rook, 4=Bishop, 5=Knight)
bool is_castling    # 是否为王车易位
bool is_en_passant  # 是否为吃过路兵
bool is_check       # 移动后是否将军
float32 confidence  # 视觉识别置信度
string algebraic_notation  # 标准代数记录法 (例如: "Nf3", "O-O", "exd5")
```

**PieceDetection.msg** - 棋子检测结果
```
Header header
geometry_msgs/Point position_3d
geometry_msgs/Point2D position_2d
int8 piece_type     # 1=King, 2=Queen, 3=Rook, 4=Bishop, 5=Knight, 6=Pawn
int8 piece_color    # 1=白, -1=黑, 0=未知
float32 confidence
sensor_msgs/Image roi_image
```

### 服务类型 (chess_interfaces/srv/)

**ValidateMove.srv** - 验证国际象棋移动合法性
```
ChessMove move
string current_fen  # 当前局面FEN
---
bool is_legal
bool is_check_after_move
bool is_checkmate
bool is_stalemate
string error_message
string resulting_fen
```

**GetEngineMove.srv** - 获取Stockfish引擎走法
```
string fen_position
int32 depth         # 搜索深度 (1-20)
float32 time_limit  # 思考时间限制(秒)
int32 elo_rating    # AI等级 (1000-3000)
---
bool success
ChessMove best_move
string algebraic_notation
int32 evaluation    # 局面评估分数(厘兵单位)
string principal_variation  # 主要变化
float32 thinking_time
string error_message
```

## 详细实施计划（17周）

### 第一阶段：ROS2环境搭建 (2周)

**Week 1: 系统基础搭建**
```bash
# Jetson Nano上安装ROS2 Humble
curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu jammy main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
sudo apt update && sudo apt install ros-humble-desktop

# 安装国际象棋相关依赖
sudo apt install stockfish python3-chess
sudo apt install ros-humble-moveit ros-humble-moveit-visual-tools
sudo apt install ros-humble-cv-bridge ros-humble-image-transport

# 安装Python象棋库
pip3 install python-chess stockfish opencv-python
```

**Week 2: 工作空间创建和基础包**
```bash
# 创建专用工作空间
mkdir -p ~/chess_robot_ws/src
cd ~/chess_robot_ws

# 创建基础包
ros2 pkg create --build-type ament_python chess_interfaces
ros2 pkg create --build-type ament_python chess_camera --dependencies rclpy sensor_msgs cv_bridge
ros2 pkg create --build-type ament_python chess_vision --dependencies rclpy opencv2
ros2 pkg create --build-type ament_python chess_engine --dependencies rclpy
ros2 pkg create --build-type ament_python chess_arm --dependencies rclpy moveit_msgs
ros2 pkg create --build-type ament_python chess_coordinator --dependencies rclpy

# 首次编译验证
colcon build
source install/setup.bash
```

### 第二阶段：国际象棋引擎集成 (2周)

**Week 3-4: Stockfish引擎节点开发**

```python
# chess_engine/chess_engine/stockfish_node.py
import rclpy
from rclpy.node import Node
import chess
import chess.engine
import chess.pgn
from chess_interfaces.msg import ChessMove, BoardState
from chess_interfaces.srv import GetEngineMove, ValidateMove

class StockfishEngineNode(Node):
    def __init__(self):
        super().__init__('stockfish_engine')
        
        # 初始化Stockfish引擎
        self.engine_path = "/usr/games/stockfish"
        self.engine = chess.engine.SimpleEngine.popen_uci(self.engine_path)
        
        # 设置引擎参数
        self.engine.configure({"Skill Level": 15, "UCI_LimitStrength": False})
        
        # 当前棋局状态
        self.board = chess.Board()
        self.game_history = []
        
        # 服务
        self.get_move_service = self.create_service(
            GetEngineMove, 'get_engine_move', self.get_engine_move_callback
        )
        self.validate_service = self.create_service(
            ValidateMove, 'validate_move', self.validate_move_callback
        )
        
        # 订阅棋盘状态更新
        self.board_subscription = self.create_subscription(
            BoardState, 'board_state', self.board_state_callback, 10
        )
        
        # 发布引擎移动
        self.move_publisher = self.create_publisher(ChessMove, 'engine_move', 10)
        
        self.get_logger().info("Stockfish engine initialized")
    
    def board_state_callback(self, msg):
        """处理棋盘状态更新"""
        try:
            # 从FEN字符串恢复棋局
            if msg.fen_notation:
                self.board = chess.Board(msg.fen_notation)
                self.get_logger().info(f"Board updated to: {msg.fen_notation}")
        except ValueError as e:
            self.get_logger().error(f"Invalid FEN notation: {e}")
    
    def get_engine_move_callback(self, request, response):
        """获取引擎推荐移动"""
        try:
            # 设置当前局面
            board = chess.Board(request.fen_position) if request.fen_position else self.board
            
            # 设置引擎参数
            if hasattr(request, 'elo_rating') and request.elo_rating > 0:
                # 根据ELO等级调整引擎强度
                skill_level = max(1, min(20, (request.elo_rating - 1000) // 100))
                self.engine.configure({"Skill Level": skill_level})
            
            # 计算最佳移动
            limit = chess.engine.Limit(
                time=request.time_limit if request.time_limit > 0 else 5.0,
                depth=request.depth if request.depth > 0 else None
            )
            
            result = self.engine.play(board, limit)
            
            if result.move:
                # 转换为ROS消息格式
                chess_move = self.chess_to_ros_move(result.move, board)
                
                response.success = True
                response.best_move = chess_move
                response.algebraic_notation = board.san(result.move)
                response.thinking_time = limit.time or 0.0
                
                # 获取评估分数
                if result.info and 'score' in result.info:
                    score = result.info['score'].relative
                    response.evaluation = score.score() if score else 0
                
                # 获取主要变化
                if result.info and 'pv' in result.info:
                    pv_moves = [board.san(move) for move in result.info['pv'][:5]]
                    response.principal_variation = ' '.join(pv_moves)
                
                self.get_logger().info(f"Engine suggests: {response.algebraic_notation}")
                
            else:
                response.success = False
                response.error_message = "Engine could not find a valid move"
                
        except Exception as e:
            response.success = False
            response.error_message = f"Engine error: {str(e)}"
            self.get_logger().error(f"Engine error: {e}")
        
        return response
    
    def validate_move_callback(self, request, response):
        """验证移动合法性"""
        try:
            # 设置棋盘状态
            board = chess.Board(request.current_fen) if request.current_fen else self.board
            
            # 转换ROS移动为chess.Move
            move = self.ros_to_chess_move(request.move, board)
            
            # 验证移动合法性
            response.is_legal = move in board.legal_moves
            
            if response.is_legal:
                # 执行移动并检查后续状态
                board.push(move)
                
                response.is_check_after_move = board.is_check()
                response.is_checkmate = board.is_checkmate()
                response.is_stalemate = board.is_stalemate()
                response.resulting_fen = board.fen()
                
                # 撤销移动
                board.pop()
                
            else:
                response.error_message = "Illegal move"
                
        except Exception as e:
            response.is_legal = False
            response.error_message = f"Validation error: {str(e)}"
        
        return response
    
    def chess_to_ros_move(self, chess_move, board):
        """转换chess.Move为ROS ChessMove消息"""
        ros_move = ChessMove()
        
        ros_move.from_square = chess_move.from_square
        ros_move.to_square = chess_move.to_square
        ros_move.piece_type = self.get_piece_type(board.piece_at(chess_move.from_square))
        
        # 检查特殊移动
        if board.is_castling(chess_move):
            ros_move.is_castling = True
        elif board.is_en_passant(chess_move):
            ros_move.is_en_passant = True
        elif chess_move.promotion:
            ros_move.promotion = chess_move.promotion
        
        # 检查吃子
        if board.is_capture(chess_move):
            captured_piece = board.piece_at(chess_move.to_square)
            if captured_piece:
                ros_move.captured_piece = self.get_piece_type(captured_piece)
        
        # 生成代数记录
        ros_move.algebraic_notation = board.san(chess_move)
        
        return ros_move
    
    def ros_to_chess_move(self, ros_move, board):
        """转换ROS ChessMove为chess.Move"""
        move = chess.Move(ros_move.from_square, ros_move.to_square)
        
        # 处理兵升变
        if ros_move.promotion > 0:
            move.promotion = ros_move.promotion
        
        return move
    
    def get_piece_type(self, piece):
        """获取棋子类型编号"""
        if piece is None:
            return 0
        
        piece_map = {
            chess.KING: 1, chess.QUEEN: 2, chess.ROOK: 3,
            chess.BISHOP: 4, chess.KNIGHT: 5, chess.PAWN: 6
        }
        
        return piece_map.get(piece.piece_type, 0) * (1 if piece.color else -1)
    
    def destroy_node(self):
        """清理资源"""
        if hasattr(self, 'engine'):
            self.engine.quit()
        super().destroy_node()


def main():
    rclpy.init()
    stockfish_node = StockfishEngineNode()
    
    try:
        rclpy.spin(stockfish_node)
    except KeyboardInterrupt:
        pass
    finally:
        stockfish_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### 第三阶段：国际象棋视觉识别系统 (4周)

**Week 5-6: 棋盘检测算法**

```python
# chess_vision/chess_vision/board_detector.py
import cv2
import numpy as np
import chess
from chess_interfaces.msg import BoardState, PieceDetection
from chess_interfaces.srv import DetectBoard

class InternationalChessBoardDetector(Node):
    def __init__(self):
        super().__init__('chess_board_detector')
        
        # ArUco检测参数
        self.aruco_dict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_4X4_50)
        self.aruco_params = cv2.aruco.DetectorParameters_create()
        
        # 棋盘参数
        self.board_size = 8  # 国际象棋8x8
        self.square_size = 0.05  # 5cm per square
        
        # 服务
        self.detect_service = self.create_service(
            DetectBoard, 'detect_board', self.detect_board_callback
        )
        
        # 发布器
        self.board_state_pub = self.create_publisher(BoardState, 'board_state', 10)
        
        self.get_logger().info("International chess board detector initialized")
    
    def detect_board_callback(self, request, response):
        """检测国际象棋棋盘"""
        try:
            rgb_image = self.bridge.imgmsg_to_cv2(request.rgb_image, 'bgr8')
            depth_image = self.bridge.imgmsg_to_cv2(request.depth_image, '16UC1')
            
            # 1. 检测ArUco标记定位棋盘
            corners, ids, _ = cv2.aruco.detectMarkers(
                rgb_image, self.aruco_dict, parameters=self.aruco_params
            )
            
            if ids is not None and len(ids) >= 4:
                # 2. 计算棋盘变换矩阵
                board_corners = self.extract_board_corners(corners, ids)
                homography = self.compute_board_homography(board_corners)
                
                # 3. 提取64个方格的位置
                square_positions = self.generate_square_positions(homography, depth_image)
                
                # 4. 识别每个方格的棋子
                board_state = self.detect_pieces_on_squares(
                    rgb_image, depth_image, square_positions
                )
                
                response.success = True
                response.board_state = board_state
                response.board_transform = self.create_transform_msg(homography)
                
            else:
                response.success = False
                response.error_message = "Cannot detect sufficient ArUco markers"
                
        except Exception as e:
            response.success = False
            response.error_message = f"Detection error: {str(e)}"
            
        return response
    
    def generate_square_positions(self, homography, depth_image):
        """生成8x8棋盘的64个方格位置"""
        positions_3d = []
        
        for rank in range(8):  # 1-8
            for file in range(8):  # a-h
                # 计算方格中心点(image coordinates)
                square_center_2d = np.array([
                    (file + 0.5) * self.square_size * 100,  # 转换为像素
                    (7 - rank + 0.5) * self.square_size * 100  # 翻转Y轴，rank 8在上
                ], dtype=np.float32)
                
                # 应用透视变换
                image_point = cv2.perspectiveTransform(
                    square_center_2d.reshape(1, 1, -1), homography
                ).reshape(-1)
                
                # 从深度图获取Z坐标
                x, y = int(image_point[0]), int(image_point[1])
                if 0 <= x < depth_image.shape[1] and 0 <= y < depth_image.shape[0]:
                    depth = depth_image[y, x] / 1000.0  # 转换为米
                    
                    # 转换为3D坐标
                    position_3d = self.image_to_world_coordinates(x, y, depth)
                    positions_3d.append(position_3d)
                else:
                    # 默认高度
                    positions_3d.append(geometry_msgs.msg.Point(x=0, y=0, z=0))
        
        return positions_3d
    
    def detect_pieces_on_squares(self, rgb_image, depth_image, square_positions):
        """检测每个方格上的棋子"""
        board_state = BoardState()
        board_state.header.stamp = self.get_clock().now().to_msg()
        board_state.header.frame_id = 'camera_link'
        
        # 初始化棋盘数组(64个方格)
        board_state.board_squares = [0] * 64
        board_state.square_positions_3d = square_positions
        
        for square_idx in range(64):
            # 提取方格ROI
            roi_rgb, roi_depth = self.extract_square_roi(
                rgb_image, depth_image, square_idx, square_positions
            )
            
            if roi_rgb is not None:
                # 使用深度信息检测是否有棋子
                if self.has_piece_on_square(roi_depth):
                    # 调用棋子分类服务
                    piece_type, piece_color = self.classify_piece(roi_rgb, roi_depth)
                    
                    if piece_type > 0:
                        # 编码棋子类型和颜色
                        board_state.board_squares[square_idx] = piece_type * piece_color
        
        # 分析当前局面并生成FEN
        board_state.fen_notation = self.board_array_to_fen(board_state.board_squares)
        
        # 解析FEN获取游戏状态
        try:
            chess_board = chess.Board(board_state.fen_notation)
            board_state.white_to_move = chess_board.turn
            board_state.castling_rights = [
                chess_board.has_kingside_castling_rights(chess.WHITE),
                chess_board.has_queenside_castling_rights(chess.WHITE),
                chess_board.has_kingside_castling_rights(chess.BLACK),
                chess_board.has_queenside_castling_rights(chess.BLACK)
            ]
            board_state.en_passant_square = chess_board.ep_square if chess_board.ep_square else -1
            board_state.halfmove_clock = chess_board.halfmove_clock
            board_state.fullmove_number = chess_board.fullmove_number
            
        except ValueError:
            # 如果FEN无效，设置默认值
            board_state.white_to_move = True
            board_state.castling_rights = [False] * 4
            board_state.en_passant_square = -1
            
        return board_state
    
    def board_array_to_fen(self, board_squares):
        """将棋盘数组转换为FEN字符串"""
        piece_symbols = {
            1: 'K', 2: 'Q', 3: 'R', 4: 'B', 5: 'N', 6: 'P',  # 白方
            -1: 'k', -2: 'q', -3: 'r', -4: 'b', -5: 'n', -6: 'p'  # 黑方
        }
        
        fen_ranks = []
        
        # 按rank遍历(从rank 8到rank 1)
        for rank in range(7, -1, -1):
            rank_str = ""
            empty_count = 0
            
            for file in range(8):
                square_idx = rank * 8 + file
                piece = board_squares[square_idx]
                
                if piece == 0:
                    empty_count += 1
                else:
                    if empty_count > 0:
                        rank_str += str(empty_count)
                        empty_count = 0
                    rank_str += piece_symbols.get(piece, '?')
            
            if empty_count > 0:
                rank_str += str(empty_count)
            
            fen_ranks.append(rank_str)
        
        # 基本FEN格式 (简化版本，实际使用中需要完整状态)
        board_fen = '/'.join(fen_ranks)
        return f"{board_fen} w KQkq - 0 1"
```

**Week 7-8: 国际象棋棋子分类模型**

```python
# chess_vision/chess_vision/piece_classifier.py
import torch
import torchvision.transforms as transforms
from torchvision.models import mobilenet_v3_small
import numpy as np

class ChessPieceClassifier(Node):
    def __init__(self):
        super().__init__('chess_piece_classifier')
        
        # 加载针对国际象棋优化的分类模型
        self.device = torch.device('cuda' if torch.cuda.is_available() else 'cpu')
        self.model = self.load_chess_piece_model()
        
        # 类别标签 (12类棋子 + 1类空格)
        self.class_labels = [
            'empty',  # 0
            'white_king', 'white_queen', 'white_rook',     # 1-3
            'white_bishop', 'white_knight', 'white_pawn',  # 4-6
            'black_king', 'black_queen', 'black_rook',     # 7-9
            'black_bishop', 'black_knight', 'black_pawn'   # 10-12
        ]
        
        # 图像预处理
        self.transform = transforms.Compose([
            transforms.ToPILImage(),
            transforms.Resize((224, 224)),
            transforms.ToTensor(),
            transforms.Normalize(mean=[0.485, 0.456, 0.406], 
                               std=[0.229, 0.224, 0.225])
        ])
        
        # 服务
        self.classify_service = self.create_service(
            ClassifyPiece, 'classify_piece', self.classify_callback
        )
        
        self.get_logger().info("Chess piece classifier initialized")
    
    def load_chess_piece_model(self):
        """加载国际象棋棋子分类模型"""
        # 使用MobileNetV3作为基础网络(适合Jetson Nano)
        model = mobilenet_v3_small(pretrained=True)
        
        # 修改分类头为13类输出
        model.classifier = torch.nn.Sequential(
            torch.nn.Linear(model.classifier[0].in_features, 512),
            torch.nn.ReLU(),
            torch.nn.Dropout(0.2),
            torch.nn.Linear(512, 13)  # 13类：空格 + 12种棋子
        )
        
        # 加载预训练权重
        try:
            checkpoint_path = '/home/chess/models/chess_piece_classifier.pth'
            model.load_state_dict(torch.load(checkpoint_path, map_location=self.device))
            self.get_logger().info("Loaded pre-trained chess piece model")
        except FileNotFoundError:
            self.get_logger().warning("Pre-trained model not found, using random weights")
        
        model.to(self.device)
        model.eval()
        
        return model
    
    def classify_callback(self, request, response):
        """棋子分类服务回调"""
        try:
            # 转换ROI图像
            roi_image = self.bridge.imgmsg_to_cv2(request.roi_image, 'bgr8')
            roi_depth = request.depth_data if hasattr(request, 'depth_data') else None
            
            # 预处理图像
            input_tensor = self.transform(roi_image).unsqueeze(0).to(self.device)
            
            # 推理
            with torch.no_grad():
                outputs = self.model(input_tensor)
                probabilities = torch.softmax(outputs, dim=1)
                predicted_class = torch.argmax(probabilities, dim=1).item()
                confidence = probabilities[0][predicted_class].item()
            
            # 解析预测结果
            if predicted_class == 0:
                # 空格
                response.piece_type = 0
                response.piece_color = 0
            else:
                # 有棋子
                piece_idx = (predicted_class - 1) % 6 + 1  # 1-6
                piece_color = 1 if predicted_class <= 6 else -1  # 白方为正，黑方为负
                
                response.piece_type = piece_idx
                response.piece_color = piece_color
            
            response.confidence = confidence
            response.success = True
            
            self.get_logger().debug(f"Classified as: {self.class_labels[predicted_class]} "
                                  f"(confidence: {confidence:.3f})")
            
        except Exception as e:
            response.success = False
            response.error_message = f"Classification error: {str(e)}"
            self.get_logger().error(f"Classification error: {e}")
        
        return response
    
    def create_training_dataset(self, data_collection_mode=False):
        """创建国际象棋棋子训练数据集"""
        if not data_collection_mode:
            return
        
        # 数据收集指南：
        # 1. 收集每种棋子在不同角度、光照下的图像
        # 2. 包含部分遮挡、阴影等真实场景
        # 3. 确保黑白棋子的区分度
        # 4. 收集空方格作为负样本
        
        dataset_structure = """
        chess_piece_dataset/
        ├── train/
        │   ├── empty/
        │   ├── white_king/
        │   ├── white_queen/
        │   ├── white_rook/
        │   ├── white_bishop/
        │   ├── white_knight/
        │   ├── white_pawn/
        │   ├── black_king/
        │   ├── black_queen/
        │   ├── black_rook/
        │   ├── black_bishop/
        │   ├── black_knight/
        │   └── black_pawn/
        ├── val/
        └── test/
        """
        
        self.get_logger().info("Training dataset structure created")
```

### 第四阶段：MoveIt2机械臂控制系统 (3周)

**Week 9: DOFBOT Pro MoveIt2配置**

```yaml
# config/dofbot_pro_chess.urdf.xacro
<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro" name="dofbot_pro_chess">
  
  <!-- 基座链接 -->
  <link name="base_link">
    <inertial>
      <mass value="2.0"/>
      <inertia ixx="0.01" ixy="0" ixz="0" iyy="0.01" iyz="0" izz="0.01"/>
    </inertial>
    <visual>
      <geometry>
        <cylinder radius="0.06" length="0.05"/>
      </geometry>
    </visual>
  </link>
  
  <!-- 关节1：基座旋转 -->
  <joint name="joint1" type="revolute">
    <parent link="base_link"/>
    <child link="link1"/>
    <origin xyz="0 0 0.06" rpy="0 0 0"/>
    <axis xyz="0 0 1"/>
    <limit lower="-3.14159" upper="3.14159" effort="10" velocity="1.0"/>
  </joint>
  
  <link name="link1">
    <inertial>
      <mass value="0.5"/>
      <inertia ixx="0.01" ixy="0" ixz="0" iyy="0.01" iyz="0" izz="0.01"/>
    </inertial>
    <visual>
      <geometry>
        <box size="0.04 0.06 0.08"/>
      </geometry>
    </visual>
  </link>
  
  <!-- 关节2-6：其他关节配置... -->
  
  <!-- 末端执行器：专为国际象棋设计的夹爪 -->
  <joint name="gripper_joint" type="prismatic">
    <parent link="link6"/>
    <child link="chess_gripper"/>
    <origin xyz="0 0 0.12" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <limit lower="0" upper="0.05" effort="15" velocity="0.1"/>
  </joint>
  
  <link name="chess_gripper">
    <inertial>
      <mass value="0.2"/>
      <inertia ixx="0.001" ixy="0" ixz="0" iyy="0.001" iyz="0" izz="0.001"/>
    </inertial>
    <visual>
      <geometry>
        <!-- 专门为抓取国际象棋棋子设计的夹爪形状 -->
        <mesh filename="package://chess_arm/meshes/chess_gripper.stl"/>
      </geometry>
    </visual>
    <collision>
      <geometry>
        <box size="0.03 0.05 0.04"/>
      </geometry>
    </collision>
  </link>
  
</robot>
```

```yaml
# config/dofbot_pro_chess_moveit.yaml
move_group:
  arm:
    joints: [joint1, joint2, joint3, joint4, joint5, joint6]
    kinematics_solver: kdl_kinematics_plugin/KDLKinematicsPlugin
    kinematics_solver_search_resolution: 0.005
    kinematics_solver_timeout: 0.05
    
    # 国际象棋专用规划参数
    planner_configs:
      - RRTConnect
      - RRTstar
      - PRM
    
    # 末端执行器配置
    end_effector_link: chess_gripper
    planning_frame: base_link
    
    # 工作空间限制(适配国际象棋棋盘)
    workspace:
      min_corner: [-0.3, -0.3, 0.0]
      max_corner: [0.3, 0.3, 0.4]
  
  chess_gripper:
    joints: [gripper_joint]
    parent_group: arm
    parent_link: link6
```

**Week 10-11: 国际象棋专用机械臂控制**

```python
# chess_arm/chess_arm/chess_arm_controller.py
import rclpy
from rclpy.node import Node
import moveit_commander
import chess
from geometry_msgs.msg import Pose, Point, Quaternion
from chess_interfaces.msg import ChessMove
from chess_interfaces.srv import PlanMove, ExecuteMove

class ChessArmController(Node):
    def __init__(self):
        super().__init__('chess_arm_controller')
        
        # MoveIt初始化
        moveit_commander.roscpp_initialize(['chess_arm_controller'])
        self.robot = moveit_commander.RobotCommander()
        self.scene = moveit_commander.PlanningSceneInterface()
        self.arm_group = moveit_commander.MoveGroupCommander("arm")
        self.gripper_group = moveit_commander.MoveGroupCommander("chess_gripper")
        
        # 设置规划参数
        self.arm_group.set_planning_time(10.0)
        self.arm_group.set_num_planning_attempts(5)
        self.arm_group.set_max_velocity_scaling_factor(0.2)  # 缓慢移动确保精度
        self.arm_group.set_max_acceleration_scaling_factor(0.1)
        
        # 国际象棋棋盘坐标系
        self.board_origin = [0.2, 0.0, 0.02]  # 棋盘a1角的世界坐标
        self.square_size = 0.05  # 每个方格5cm
        
        # 高度设置
        self.board_height = 0.02
        self.piece_heights = {
            1: 0.045,  # King
            2: 0.040,  # Queen  
            3: 0.035,  # Rook
            4: 0.032,  # Bishop
            5: 0.030,  # Knight
            6: 0.025   # Pawn
        }
        
        # 服务
        self.plan_service = self.create_service(PlanMove, 'plan_chess_move', self.plan_move_callback)
        self.execute_service = self.create_service(ExecuteMove, 'execute_chess_move', self.execute_callback)
        
        # 添加棋盘碰撞检测
        self.add_chessboard_collision_objects()
        
        # 移动到待机位置
        self.move_to_standby_position()
        
        self.get_logger().info("Chess arm controller initialized")
    
    def square_to_pose(self, square_index, piece_type=0, is_pickup=True):
        """转换方格索引为3D位姿"""
        # 转换方格索引(0-63)为file和rank
        file = square_index % 8  # 0-7 (a-h)
        rank = square_index // 8  # 0-7 (1-8)
        
        # 计算世界坐标
        x = self.board_origin[0] + file * self.square_size
        y = self.board_origin[1] + rank * self.square_size
        
        # 根据操作类型和棋子类型确定高度
        if is_pickup:
            z = self.board_height + self.piece_heights.get(abs(piece_type), 0.03) + 0.005
        else:
            z = self.board_height + 0.01  # 放置时稍微低一点
        
        # 创建位姿
        pose = Pose()
        pose.position = Point(x=x, y=y, z=z)
        
        # 垂直向下的方向
        pose.orientation = Quaternion(x=0.0, y=1.0, z=0.0, w=0.0)
        
        return pose
    
    def plan_move_callback(self, request, response):
        """规划国际象棋移动"""
        try:
            chess_move = request.chess_move
            
            # 获取起始和目标位置
            from_pose = self.square_to_pose(chess_move.from_square, chess_move.piece_type, True)
            to_pose = self.square_to_pose(chess_move.to_square, chess_move.piece_type, False)
            
            # 特殊移动处理
            if chess_move.is_castling:
                trajectory = self.plan_castling_move(chess_move)
            elif chess_move.is_en_passant:
                trajectory = self.plan_en_passant_move(chess_move)
            elif chess_move.captured_piece != 0:
                trajectory = self.plan_capture_move(chess_move, from_pose, to_pose)
            else:
                trajectory = self.plan_normal_move(from_pose, to_pose)
            
            if trajectory:
                response.success = True
                response.trajectory = trajectory
                response.execution_time = self.estimate_execution_time(trajectory)
            else:
                response.success = False
                response.error_message = "Failed to plan chess move"
            
        except Exception as e:
            response.success = False
            response.error_message = f"Planning error: {str(e)}"
            self.get_logger().error(f"Move planning failed: {e}")
        
        return response
    
    def plan_normal_move(self, from_pose, to_pose):
        """规划普通移动：抓取-提升-移动-放置"""
        waypoints = []
        
        # 1. 移动到起始位置上方
        approach_pose = self.create_approach_pose(from_pose, height_offset=0.08)
        waypoints.append(approach_pose)
        
        # 2. 下降到抓取位置
        waypoints.append(from_pose)
        
        # 3. 抓取后提升
        lift_pose = self.create_approach_pose(from_pose, height_offset=0.10)
        waypoints.append(lift_pose)
        
        # 4. 移动到目标上方
        target_approach = self.create_approach_pose(to_pose, height_offset=0.10)
        waypoints.append(target_approach)
        
        # 5. 下降放置
        waypoints.append(to_pose)
        
        # 6. 放置后提升
        target_retreat = self.create_approach_pose(to_pose, height_offset=0.08)
        waypoints.append(target_retreat)
        
        # 计算笛卡尔路径，避开其他棋子
        (plan, fraction) = self.arm_group.compute_cartesian_path(
            waypoints, 
            0.005,  # 5mm步长
            0.0     # 跳跃阈值
        )
        
        return plan if fraction > 0.9 else None
    
    def plan_castling_move(self, chess_move):
        """规划王车易位：需要同时移动王和车"""
        # 王车易位涉及两个棋子的移动
        king_from = chess_move.from_square
        king_to = chess_move.to_square
        
        # 确定车的位置
        if king_to == chess.G1 or king_to == chess.G8:  # 王翼易位
            rook_from = chess.H1 if king_to == chess.G1 else chess.H8
            rook_to = chess.F1 if king_to == chess.G1 else chess.F8
        else:  # 后翼易位
            rook_from = chess.A1 if king_to == chess.C1 else chess.A8
            rook_to = chess.D1 if king_to == chess.C1 else chess.D8
        
        # 先移动王，再移动车
        king_trajectory = self.plan_normal_move(
            self.square_to_pose(king_from, 1, True),
            self.square_to_pose(king_to, 1, False)
        )
        
        rook_trajectory = self.plan_normal_move(
            self.square_to_pose(rook_from, 3, True),
            self.square_to_pose(rook_to, 3, False)
        )
        
        # 组合轨迹
        return self.combine_trajectories([king_trajectory, rook_trajectory])
    
    def plan_capture_move(self, chess_move, from_pose, to_pose):
        """规划吃子移动：先移除被吃棋子，再移动己方棋子"""
        waypoints = []
        
        # 1. 先移除被吃的棋子(移到棋盘外)
        captured_piece_pose = self.square_to_pose(chess_move.to_square, chess_move.captured_piece, True)
        discard_pose = self.get_discard_position()
        
        # 移动到被吃棋子上方
        waypoints.append(self.create_approach_pose(captured_piece_pose, 0.08))
        # 抓取被吃棋子
        waypoints.append(captured_piece_pose)
        # 提升
        waypoints.append(self.create_approach_pose(captured_piece_pose, 0.10))
        # 移动到丢弃区
        waypoints.append(self.create_approach_pose(discard_pose, 0.05))
        waypoints.append(discard_pose)
        # 提升离开
        waypoints.append(self.create_approach_pose(discard_pose, 0.08))
        
        # 2. 然后移动己方棋子
        normal_move_waypoints = self.get_normal_move_waypoints(from_pose, to_pose)
        waypoints.extend(normal_move_waypoints)
        
        # 计算完整路径
        (plan, fraction) = self.arm_group.compute_cartesian_path(waypoints, 0.005, 0.0)
        
        return plan if fraction > 0.85 else None
    
    def add_chessboard_collision_objects(self):
        """添加棋盘和边界作为碰撞检测对象"""
        # 添加棋盘表面
        board_pose = Pose()
        board_pose.position.x = self.board_origin[0] + 4 * self.square_size
        board_pose.position.y = self.board_origin[1] + 4 * self.square_size
        board_pose.position.z = self.board_height / 2
        board_pose.orientation.w = 1.0
        
        self.scene.add_box("chessboard", board_pose, (0.4, 0.4, 0.02))
        
        # 添加棋盘边界墙，防止意外移动到棋盘外
        boundaries = [
            ("boundary_left", [-0.05, 0.2, 0.1], (0.02, 0.5, 0.2)),
            ("boundary_right", [0.45, 0.2, 0.1], (0.02, 0.5, 0.2)),
            ("boundary_front", [0.2, -0.05, 0.1], (0.5, 0.02, 0.2)),
            ("boundary_back", [0.2, 0.45, 0.1], (0.5, 0.02, 0.2))
        ]
        
        for name, position, size in boundaries:
            boundary_pose = Pose()
            boundary_pose.position.x, boundary_pose.position.y, boundary_pose.position.z = position
            boundary_pose.orientation.w = 1.0
            self.scene.add_box(name, boundary_pose, size)
    
    def move_to_standby_position(self):
        """移动到待机位置"""
        standby_pose = Pose()
        standby_pose.position.x = 0.0
        standby_pose.position.y = -0.3
        standby_pose.position.z = 0.3
        standby_pose.orientation.w = 1.0
        
        self.arm_group.set_pose_target(standby_pose)
        self.arm_group.go(wait=True)
        self.arm_group.clear_pose_targets()
        
        self.get_logger().info("Moved to standby position")
```

### 第五阶段：系统协调和状态机 (2周)

**Week 12-13: 国际象棋游戏协调器**

```python
# chess_coordinator/chess_coordinator/chess_game_coordinator.py
import rclpy
from rclpy.node import Node
import chess
import chess.pgn
from enum import Enum
import time
from chess_interfaces.msg import ChessMove, BoardState, GameStatus
from chess_interfaces.srv import GetEngineMove, ValidateMove, DetectBoard, PlanMove, ExecuteMove

class ChessGameState(Enum):
    INITIALIZING = 1
    WAITING_FOR_GAME_START = 2
    GAME_SETUP = 3
    HUMAN_TURN = 4
    DETECTING_HUMAN_MOVE = 5
    VALIDATING_HUMAN_MOVE = 6
    ENGINE_THINKING = 7
    PLANNING_ENGINE_MOVE = 8
    EXECUTING_ENGINE_MOVE = 9
    GAME_OVER = 10
    PAUSED = 11
    ERROR = 12

class ChessGameCoordinator(Node):
    def __init__(self):
        super().__init__('chess_game_coordinator')
        
        self.state = ChessGameState.INITIALIZING
        self.chess_board = chess.Board()  # 标准国际象棋开局
        self.game_pgn = chess.pgn.Game()
        
        # 游戏设置
        self.human_plays_white = True
        self.engine_difficulty = 15  # Stockfish深度
        self.engine_time_limit = 5.0
        
        # 状态跟踪
        self.last_board_state = None
        self.current_board_state = None
        self.last_human_move = None
        self.current_engine_move = None
        self.move_count = 0
        
        # 服务客户端
        self.detect_board_client = self.create_client(DetectBoard, 'detect_board')
        self.validate_move_client = self.create_client(ValidateMove, 'validate_move')
        self.get_engine_move_client = self.create_client(GetEngineMove, 'get_engine_move')
        self.plan_move_client = self.create_client(PlanMove, 'plan_chess_move')
        self.execute_move_client = self.create_client(ExecuteMove, 'execute_chess_move')
        
        # 订阅者
        self.board_state_sub = self.create_subscription(
            BoardState, 'board_state', self.board_state_callback, 10
        )
        
        # 发布者
        self.game_status_pub = self.create_publisher(GameStatus, 'game_status', 10)
        self.move_pub = self.create_publisher(ChessMove, 'game_move', 10)
        
        # 主状态机定时器
        self.state_timer = self.create_timer(0.5, self.state_machine_update)
        
        # 游戏统计
        self.game_start_time = None
        self.total_thinking_time = {'human': 0.0, 'engine': 0.0}
        
        self.get_logger().info("Chess Game Coordinator initialized")
        self.get_logger().info(f"Starting position: {self.chess_board.fen()}")
    
    def state_machine_update(self):
        """主状态机循环"""
        try:
            if self.state == ChessGameState.INITIALIZING:
                self.handle_initializing()
            elif self.state == ChessGameState.WAITING_FOR_GAME_START:
                self.handle_waiting_for_start()
            elif self.state == ChessGameState.GAME_SETUP:
                self.handle_game_setup()
            elif self.state == ChessGameState.HUMAN_TURN:
                self.handle_human_turn()
            elif self.state == ChessGameState.DETECTING_HUMAN_MOVE:
                self.handle_detecting_human_move()
            elif self.state == ChessGameState.VALIDATING_HUMAN_MOVE:
                self.handle_validating_human_move()
            elif self.state == ChessGameState.ENGINE_THINKING:
                self.handle_engine_thinking()
            elif self.state == ChessGameState.PLANNING_ENGINE_MOVE:
                self.handle_planning_engine_move()
            elif self.state == ChessGameState.EXECUTING_ENGINE_MOVE:
                self.handle_executing_engine_move()
            elif self.state == ChessGameState.GAME_OVER:
                self.handle_game_over()
            elif self.state == ChessGameState.ERROR:
                self.handle_error()
            
            # 发布游戏状态
            self.publish_game_status()
            
        except Exception as e:
            self.get_logger().error(f"State machine error: {e}")
            self.state = ChessGameState.ERROR
    
    def handle_initializing(self):
        """初始化处理"""
        # 等待所有服务上线
        services_ready = all([
            self.detect_board_client.wait_for_service(timeout_sec=1.0),
            self.validate_move_client.wait_for_service(timeout_sec=1.0),
            self.get_engine_move_client.wait_for_service(timeout_sec=1.0),
            self.plan_move_client.wait_for_service(timeout_sec=1.0),
            self.execute_move_client.wait_for_service(timeout_sec=1.0)
        ])
        
        if services_ready:
            self.get_logger().info("All services are ready")
            self.state = ChessGameState.WAITING_FOR_GAME_START
        else:
            self.get_logger().warning("Waiting for services...")
    
    def handle_game_setup(self):
        """游戏设置处理"""
        # 检测初始棋盘设置是否正确
        if self.current_board_state:
            expected_fen = chess.Board().fen().split()[0]  # 只比较棋盘部分
            actual_fen = self.board_state_to_fen(self.current_board_state).split()[0]
            
            if self.fen_pieces_match(expected_fen, actual_fen):
                self.get_logger().info("Chess board setup verified")
                self.game_start_time = time.time()
                
                if self.human_plays_white:
                    self.state = ChessGameState.HUMAN_TURN
                    self.get_logger().info("Human plays white - your turn!")
                else:
                    self.state = ChessGameState.ENGINE_THINKING
                    self.get_logger().info("Engine plays white - thinking...")
            else:
                self.get_logger().warning(f"Board setup incorrect. Expected: {expected_fen}, Got: {actual_fen}")
    
    def handle_human_turn(self):
        """处理人类回合"""
        # 持续检测棋盘变化
        if self.board_state_changed():
            self.get_logger().info("Detected board change - analyzing move")
            self.state = ChessGameState.DETECTING_HUMAN_MOVE
            self.human_move_start_time = time.time()
    
    def handle_detecting_human_move(self):
        """检测人类移动"""
        if self.current_board_state and self.last_board_state:
            detected_move = self.detect_human_move_from_board_diff(
                self.last_board_state, self.current_board_state
            )
            
            if detected_move:
                self.last_human_move = detected_move
                self.state = ChessGameState.VALIDATING_HUMAN_MOVE
                
                # 记录人类思考时间
                if hasattr(self, 'human_move_start_time'):
                    thinking_time = time.time() - self.human_move_start_time
                    self.total_thinking_time['human'] += thinking_time
                    
                self.get_logger().info(f"Detected human move: {detected_move.algebraic_notation}")
    
    def handle_validating_human_move(self):
        """验证人类移动"""
        if not hasattr(self, 'validate_future'):
            # 发起验证请求
            request = ValidateMove.Request()
            request.move = self.last_human_move
            request.current_fen = self.chess_board.fen()
            
            self.validate_future = self.validate_move_client.call_async(request)
        
        elif self.validate_future.done():
            result = self.validate_future.result()
            
            if result.is_legal:
                # 合法移动，更新棋局
                move_str = f"{chess.square_name(self.last_human_move.from_square)}{chess.square_name(self.last_human_move.to_square)}"
                if self.last_human_move.promotion > 0:
                    promotion_map = {2: 'q', 3: 'r', 4: 'b', 5: 'n'}
                    move_str += promotion_map.get(self.last_human_move.promotion, 'q')
                
                chess_move = chess.Move.from_uci(move_str)
                self.chess_board.push(chess_move)
                self.move_count += 1
                
                self.get_logger().info(f"Legal move accepted: {self.chess_board.san(chess_move)}")
                
                # 检查游戏结束条件
                if self.chess_board.is_checkmate():
                    self.get_logger().info("Checkmate! Human wins!")
                    self.state = ChessGameState.GAME_OVER
                elif self.chess_board.is_stalemate():
                    self.get_logger().info("Stalemate!")
                    self.state = ChessGameState.GAME_OVER
                elif self.chess_board.is_insufficient_material():
                    self.get_logger().info("Draw by insufficient material")
                    self.state = ChessGameState.GAME_OVER
                else:
                    # 轮到引擎
                    self.state = ChessGameState.ENGINE_THINKING
                    self.engine_thinking_start_time = time.time()
                
            else:
                self.get_logger().warning(f"Illegal move detected: {result.error_message}")
                # 回到人类回合，等待正确移动
                self.state = ChessGameState.HUMAN_TURN
            
            delattr(self, 'validate_future')
    
    def handle_engine_thinking(self):
        """处理引擎思考"""
        if not hasattr(self, 'engine_future'):
            # 发起引擎计算请求
            request = GetEngineMove.Request()
            request.fen_position = self.chess_board.fen()
            request.depth = self.engine_difficulty
            request.time_limit = self.engine_time_limit
            request.elo_rating = 2000  # 可调整
            
            self.engine_future = self.get_engine_move_client.call_async(request)
            self.get_logger().info("Engine is thinking...")
        
        elif self.engine_future.done():
            result = self.engine_future.result()
            
            if result.success:
                self.current_engine_move = result.best_move
                
                # 记录引擎思考时间
                thinking_time = time.time() - self.engine_thinking_start_time
                self.total_thinking_time['engine'] += thinking_time
                
                self.get_logger().info(f"Engine suggests: {result.algebraic_notation} "
                                     f"(eval: {result.evaluation}, time: {thinking_time:.1f}s)")
                
                # 更新棋局状态
                move_str = f"{chess.square_name(self.current_engine_move.from_square)}{chess.square_name(self.current_engine_move.to_square)}"
                if self.current_engine_move.promotion > 0:
                    promotion_map = {2: 'q', 3: 'r', 4: 'b', 5: 'n'}
                    move_str += promotion_map.get(self.current_engine_move.promotion, 'q')
                
                chess_move = chess.Move.from_uci(move_str)
                self.chess_board.push(chess_move)
                self.move_count += 1
                
                self.state = ChessGameState.PLANNING_ENGINE_MOVE
                
            else:
                self.get_logger().error(f"Engine failed: {result.error_message}")
                self.state = ChessGameState.ERROR
            
            delattr(self, 'engine_future')
    
    def handle_planning_engine_move(self):
        """规划引擎移动"""
        if not hasattr(self, 'plan_future'):
            request = PlanMove.Request()
            request.chess_move = self.current_engine_move
            request.speed_factor = 0.2
            request.avoid_pieces = True
            
            self.plan_future = self.plan_move_client.call_async(request)
            self.get_logger().info("Planning engine move execution...")
        
        elif self.plan_future.done():
            result = self.plan_future.result()
            
            if result.success:
                self.state = ChessGameState.EXECUTING_ENGINE_MOVE
                self.execution_start_time = time.time()
            else:
                self.get_logger().error(f"Move planning failed: {result.error_message}")
                self.state = ChessGameState.ERROR
            
            delattr(self, 'plan_future')
    
    def handle_executing_engine_move(self):
        """执行引擎移动"""
        if not hasattr(self, 'execute_future'):
            request = ExecuteMove.Request()
            request.chess_move = self.current_engine_move
            
            self.execute_future = self.execute_move_client.call_async(request)
            self.get_logger().info("Executing engine move...")
        
        elif self.execute_future.done():
            result = self.execute_future.result()
            
            if result.success:
                execution_time = time.time() - self.execution_start_time
                self.get_logger().info(f"Engine move executed successfully in {execution_time:.1f}s")
                
                # 检查游戏结束条件
                if self.chess_board.is_checkmate():
                    self.get_logger().info("Checkmate! Engine wins!")
                    self.state = ChessGameState.GAME_OVER
                elif self.chess_board.is_stalemate():
                    self.get_logger().info("Stalemate!")
                    self.state = ChessGameState.GAME_OVER
                else:
                    # 轮到人类
                    self.state = ChessGameState.HUMAN_TURN
                    self.get_logger().info("Your turn!")
                
            else:
                self.get_logger().error(f"Move execution failed: {result.error_message}")
                self.state = ChessGameState.ERROR
            
            delattr(self, 'execute_future')
    
    def handle_game_over(self):
        """处理游戏结束"""
        if not hasattr(self, 'game_over_logged'):
            total_game_time = time.time() - self.game_start_time if self.game_start_time else 0
            
            self.get_logger().info("=== GAME OVER ===")
            self.get_logger().info(f"Total moves: {self.move_count}")
            self.get_logger().info(f"Game duration: {total_game_time:.1f}s")
            self.get_logger().info(f"Human thinking time: {self.total_thinking_time['human']:.1f}s")
            self.get_logger().info(f"Engine thinking time: {self.total_thinking_time['engine']:.1f}s")
            self.get_logger().info(f"Final position: {self.chess_board.fen()}")
            
            # 保存PGN
            self.save_game_pgn()
            
            self.game_over_logged = True
    
    def detect_human_move_from_board_diff(self, old_state, new_state):
        """通过比较前后棋盘状态检测人类移动"""
        if not old_state or not new_state:
            return None
        
        differences = []
        for i in range(64):
            if old_state.board_squares[i] != new_state.board_squares[i]:
                differences.append({
                    'square': i,
                    'old_piece': old_state.board_squares[i],
                    'new_piece': new_state.board_squares[i]
                })
        
        # 分析差异模式
        if len(differences) == 2:
            # 正常移动：一个格子变空，一个格子放置棋子
            from_square = None
            to_square = None
            moved_piece = None
            
            for diff in differences:
                if diff['old_piece'] != 0 and diff['new_piece'] == 0:
                    # 原来有棋子，现在没有 = 起始位置
                    from_square = diff['square']
                    moved_piece = diff['old_piece']
                elif diff['old_piece'] == 0 and diff['new_piece'] != 0:
                    # 原来没有，现在有 = 目标位置
                    to_square = diff['square']
                elif diff['old_piece'] != 0 and diff['new_piece'] != 0:
                    # 吃子情况
                    to_square = diff['square']
                    moved_piece = diff['new_piece']
            
            if from_square is not None and to_square is not None and moved_piece is not None:
                return self.create_chess_move(from_square, to_square, moved_piece, differences)
        
        elif len(differences) == 4:
            # 可能是王车易位
            return self.detect_castling_move(differences)
        
        return None
    
    def create_chess_move(self, from_square, to_square, piece, differences):
        """创建国际象棋移动消息"""
        move = ChessMove()
        move.from_square = from_square
        move.to_square = to_square
        move.piece_type = abs(piece)
        
        # 检查是否吃子
        for diff in differences:
            if diff['square'] == to_square and diff['old_piece'] != 0:
                move.captured_piece = abs(diff['old_piece'])
        
        # 生成代数记录
        try:
            chess_move = chess.Move(from_square, to_square)
            if chess_move in self.chess_board.legal_moves:
                move.algebraic_notation = self.chess_board.san(chess_move)
        except:
            move.algebraic_notation = f"{chess.square_name(from_square)}{chess.square_name(to_square)}"
        
        return move
    
    def board_state_changed(self):
        """检查棋盘状态是否发生变化"""
        if not self.current_board_state or not self.last_board_state:
            return False
        
        for i in range(64):
            if self.current_board_state.board_squares[i] != self.last_board_state.board_squares[i]:
                return True
        
        return False
    
    def board_state_callback(self, msg):
        """棋盘状态更新回调"""
        self.last_board_state = self.current_board_state
        self.current_board_state = msg
    
    def publish_game_status(self):
        """发布游戏状态"""
        status = GameStatus()
        status.header.stamp = self.get_clock().now().to_msg()
        status.state = self.state.value
        status.current_player = 'white' if self.chess_board.turn else 'black'
        status.move_count = self.move_count
        status.is_check = self.chess_board.is_check()
        
        if self.chess_board.is_checkmate():
            winner = 'white' if not self.chess_board.turn else 'black'
            status.game_result = f"Checkmate - {winner} wins"
        elif self.chess_board.is_stalemate():
            status.game_result = "Stalemate - Draw"
        elif self.chess_board.is_insufficient_material():
            status.game_result = "Insufficient material - Draw"
        
        # 最后一步移动
        if self.chess_board.move_stack:
            last_move = self.chess_board.move_stack[-1]
            status.last_move = self.chess_board.san(last_move)
        
        self.game_status_pub.publish(status)
    
    def save_game_pgn(self):
        """保存游戏PGN记录"""
        try:
            game = chess.pgn.Game()
            game.headers["Event"] = "Chess Robot Game"
            game.headers["Date"] = time.strftime("%Y.%m.%d")
            game.headers["White"] = "Human" if self.human_plays_white else "Robot"
            game.headers["Black"] = "Robot" if self.human_plays_white else "Human"
            
            # 设置结果
            if self.chess_board.is_checkmate():
                game.headers["Result"] = "1-0" if not self.chess_board.turn else "0-1"
            else:
                game.headers["Result"] = "1/2-1/2"
            
            # 添加移动记录
            node = game
            board = chess.Board()
            for move in self.chess_board.move_stack:
                node = node.add_variation(move)
                board.push(move)
            
            # 保存到文件
            filename = f"/home/chess/games/game_{int(time.time())}.pgn"
            with open(filename, "w") as f:
                print(game, file=f)
            
            self.get_logger().info(f"Game saved to {filename}")
            
        except Exception as e:
            self.get_logger().error(f"Failed to save PGN: {e}")


def main():
    rclpy.init()
    coordinator = ChessGameCoordinator()
    
    try:
        rclpy.spin(coordinator)
    except KeyboardInterrupt:
        pass
    finally:
        coordinator.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### 第六阶段：Web界面开发 (2周)

**Week 14-15: 国际象棋专用Web界面**

```html
<!-- chess_ui/templates/international_chess.html -->
<!DOCTYPE html>
<html lang="zh-CN">
<head>
    <meta charset="UTF-8">
    <meta name="viewport" content="width=device-width, initial-scale=1.0">
    <title>国际象棋机器人控制台</title>
    <script src="https://unpkg.com/react@18/umd/react.development.js"></script>
    <script src="https://unpkg.com/react-dom@18/umd/react-dom.development.js"></script>
    <script src="https://unpkg.com/@babel/standalone/babel.min.js"></script>
    <script src="https://cdnjs.cloudflare.com/ajax/libs/socket.io/4.7.2/socket.io.js"></script>
    <script src="https://cdnjs.cloudflare.com/ajax/libs/chess.js/1.0.0-beta.6/chess.min.js"></script>
    <link href="https://cdn.jsdelivr.net/npm/tailwindcss@2.2.19/dist/tailwind.min.css" rel="stylesheet">
    <style>
        .chess-board {
            width: 480px;
            height: 480px;
            border: 2px solid #8b4513;
            display: grid;
            grid-template-columns: repeat(8, 1fr);
            grid-template-rows: repeat(8, 1fr);
        }
        
        .chess-square {
            width: 60px;
            height: 60px;
            display: flex;
            align-items: center;
            justify-content: center;
            font-size: 40px;
            cursor: pointer;
            transition: all 0.2s;
            position: relative;
        }
        
        .chess-square.light { background-color: #f0d9b5; }
        .chess-square.dark { background-color: #b58863; }
        .chess-square.selected { background-color: #ffff00 !important; box-shadow: inset 0 0 10px rgba(0,0,0,0.5); }
        .chess-square.legal-move { background-color: #90ee90 !important; }
        .chess-square.last-move { background-color: #ffa500 !important; }
        .chess-square.check { background-color: #ff6b6b !important; }
        
        .chess-square:hover {
            transform: scale(1.05);
        }
        
        .file-rank-labels {
            font-family: 'Times New Roman', serif;
            font-weight: bold;
            color: #8b4513;
        }
        
        .captured-pieces {
            display: flex;
            flex-wrap: wrap;
            gap: 5px;
            padding: 10px;
            background-color: #f5f5f5;
            border-radius: 8px;
            min-height: 60px;
        }
        
        .captured-piece {
            font-size: 24px;
            opacity: 0.7;
        }
        
        .move-history {
            max-height: 300px;
            overflow-y: auto;
            font-family: monospace;
            font-size: 14px;
            background-color: #f9f9f9;
            padding: 10px;
            border-radius: 8px;
        }
        
        .engine-analysis {
            background: linear-gradient(135deg, #667eea 0%, #764ba2 100%);
            color: white;
            padding: 15px;
            border-radius: 8px;
        }
        
        .evaluation-bar {
            width: 100%;
            height: 20px;
            background: linear-gradient(to right, #000 0%, #000 50%, #fff 50%, #fff 100%);
            border-radius: 10px;
            position: relative;
            overflow: hidden;
        }
        
        .eval-indicator {
            position: absolute;
            top: 0;
            height: 100%;
            width: 2px;
            background-color: #ff0000;
            transition: left 0.3s ease;
        }
    </style>
</head>
<body class="bg-gray-50">
    <div id="chess-app"></div>

    <script type="text/babel">
        // 国际象棋棋子Unicode符号
        const CHESS_PIECES = {
            // 白方棋子
            'wK': '♔', 'wQ': '♕', 'wR': '♖', 'wB': '♗', 'wN': '♘', 'wP': '♙',
            // 黑方棋子  
            'bK': '♚', 'bQ': '♛', 'bR': '♜', 'bB': '♝', 'bN': '♞', 'bP': '♟'
        };
        
        const PIECE_VALUES = {
            1: 'K', 2: 'Q', 3: 'R', 4: 'B', 5: 'N', 6: 'P'
        };

        function InternationalChessApp() {
            const [socket, setSocket] = React.useState(null);
            const [gameState, setGameState] = React.useState(null);
            const [boardState, setBoardState] = React.useState(null);
            const [selectedSquare, setSelectedSquare] = React.useState(null);
            const [legalMoves, setLegalMoves] = React.useState([]);
            const [moveHistory, setMoveHistory] = React.useState([]);
            const [capturedPieces, setCapturedPieces] = React.useState({white: [], black: []});
            const [engineAnalysis, setEngineAnalysis] = React.useState(null);
            const [cameraFeed, setCameraFeed] = React.useState(null);
            const [gameSettings, setGameSettings] = React.useState({
                humanPlaysWhite: true,
                engineDifficulty: 15,
                timeLimit: 5.0
            });

            // 初始化WebSocket连接
            React.useEffect(() => {
                const newSocket = io();
                setSocket(newSocket);

                newSocket.on('connect', () => {
                    console.log('Connected to chess robot');
                });

                newSocket.on('game_status_update', (data) => {
                    setGameState(data);
                });

                newSocket.on('board_state_update', (data) => {
                    setBoardState(data);
                    updateCapturedPieces(data);
                });

                newSocket.on('engine_analysis', (data) => {
                    setEngineAnalysis(data);
                });

                newSocket.on('camera_frame', (data) => {
                    setCameraFeed(data.image);
                });

                newSocket.on('move_made', (data) => {
                    setMoveHistory(prev => [...prev, data]);
                });

                return () => newSocket.close();
            }, []);

            const updateCapturedPieces = (board) => {
                // 计算被吃的棋子
                const initialPieces = {
                    white: ['♔','♕','♖','♖','♗','♗','♘','♘','♙','♙','♙','♙','♙','♙','♙','♙'],
                    black: ['♚','♛','♜','♜','♝','♝','♞','♞','♟','♟','♟','♟','♟','♟','♟','♟']
                };
                
                const currentPieces = {white: [], black: []};
                
                board.board_squares?.forEach(piece => {
                    if (piece > 0) {
                        const pieceSymbol = getPieceSymbol(piece, 1);
                        currentPieces.white.push(pieceSymbol);
                    } else if (piece < 0) {
                        const pieceSymbol = getPieceSymbol(-piece, -1);
                        currentPieces.black.push(pieceSymbol);
                    }
                });
                
                const captured = {
                    white: initialPieces.white.filter(p => !currentPieces.white.includes(p)),
                    black: initialPieces.black.filter(p => !currentPieces.black.includes(p))
                };
                
                setCapturedPieces(captured);
            };

            const getPieceSymbol = (pieceType, color) => {
                const pieceChar = PIECE_VALUES[pieceType] || 'P';
                const colorPrefix = color > 0 ? 'w' : 'b';
                return CHESS_PIECES[colorPrefix + pieceChar] || '';
            };

            const renderChessBoard = () => {
                if (!boardState) return <div className="chess-board bg-gray-300 flex items-center justify-center">Loading board...</div>;

                const squares = [];
                const files = ['a', 'b', 'c', 'd', 'e', 'f', 'g', 'h'];
                const ranks = ['8', '7', '6', '5', '4', '3', '2', '1'];

                for (let rank = 0; rank < 8; rank++) {
                    for (let file = 0; file < 8; file++) {
                        const squareIndex = rank * 8 + file;
                        const piece = boardState.board_squares[squareIndex];
                        const isLight = (rank + file) % 2 === 0;
                        const isSelected = selectedSquare === squareIndex;
                        const isLegalMove = legalMoves.includes(squareIndex);
                        
                        squares.push(
                            <div
                                key={squareIndex}
                                className={`chess-square ${isLight ? 'light' : 'dark'} ${isSelected ? 'selected' : ''} ${isLegalMove ? 'legal-move' : ''}`}
                                onClick={() => handleSquareClick(squareIndex)}
                            >
                                {piece !== 0 && (
                                    <span className="piece-symbol">
                                        {getPieceSymbol(Math.abs(piece), piece > 0 ? 1 : -1)}
                                    </span>
                                )}
                                {/* 坐标标签 */}
                                {rank === 7 && (
                                    <div className="absolute top-1 left-1 file-rank-labels text-xs">
                                        {files[file]}
                                    </div>
                                )}
                                {file === 0 && (
                                    <div className="absolute bottom-1 right-1 file-rank-labels text-xs">
                                        {ranks[rank]}
                                    </div>
                                )}
                            </div>
                        );
                    }
                }

                return <div className="chess-board">{squares}</div>;
            };

            const handleSquareClick = (squareIndex) => {
                if (gameState?.state !== 3) return; // 只在人类回合允许点击

                if (selectedSquare === null) {
                    // 选择棋子
                    const piece = boardState.board_squares[squareIndex];
                    if (piece !== 0 && ((piece > 0 && boardState.white_to_move) || (piece < 0 && !boardState.white_to_move))) {
                        setSelectedSquare(squareIndex);
                        // 请求获取合法移动
                        socket?.emit('get_legal_moves', { square: squareIndex, fen: boardState.fen_notation });
                    }
                } else if (selectedSquare === squareIndex) {
                    // 取消选择
                    setSelectedSquare(null);
                    setLegalMoves([]);
                } else {
                    // 尝试移动
                    if (legalMoves.includes(squareIndex)) {
                        const move = {
                            from: selectedSquare,
                            to: squareIndex,
                            promotion: 'q' // 默认升变为后
                        };
                        socket?.emit('make_move', move);
                    }
                    setSelectedSquare(null);
                    setLegalMoves([]);
                }
            };

            const startNewGame = () => {
                socket?.emit('start_new_game', gameSettings);
            };

            const resetGame = () => {
                socket?.emit('reset_game');
                setMoveHistory([]);
                setCapturedPieces({white: [], black: []});
            };

            const renderGameControls = () => (
                <div className="bg-white p-6 rounded-lg shadow-lg">
                    <h3 className="text-xl font-bold mb-4">游戏控制</h3>
                    
                    <div className="space-y-4">
                        <div>
                            <label className="block text-sm font-medium mb-2">人类执白棋</label>
                            <input 
                                type="checkbox" 
                                checked={gameSettings.humanPlaysWhite}
                                onChange={(e) => setGameSettings({...gameSettings, humanPlaysWhite: e.target.checked})}
                                className="w-4 h-4"
                            />
                        </div>
                        
                        <div>
                            <label className="block text-sm font-medium mb-2">引擎难度: {gameSettings.engineDifficulty}</label>
                            <input 
                                type="range" 
                                min="1" 
                                max="20" 
                                value={gameSettings.engineDifficulty}
                                onChange={(e) => setGameSettings({...gameSettings, engineDifficulty: parseInt(e.target.value)})}
                                className="w-full"
                            />
                        </div>
                        
                        <div>
                            <label className="block text-sm font-medium mb-2">思考时间: {gameSettings.timeLimit}秒</label>
                            <input 
                                type="range" 
                                min="1" 
                                max="30" 
                                value={gameSettings.timeLimit}
                                onChange={(e) => setGameSettings({...gameSettings, timeLimit: parseFloat(e.target.value)})}
                                className="w-full"
                            />
                        </div>
                        
                        <div className="flex space-x-2">
                            <button 
                                onClick={startNewGame}
                                className="bg-green-500 hover:bg-green-600 text-white px-4 py-2 rounded"
                            >
                                开始新游戏
                            </button>
                            <button 
                                onClick={resetGame}
                                className="bg-red-500 hover:bg-red-600 text-white px-4 py-2 rounded"
                            >
                                重置游戏
                            </button>
                        </div>
                    </div>
                </div>
            );

            const renderEngineAnalysis = () => {
                if (!engineAnalysis) return null;
                
                const evalPercentage = Math.max(0, Math.min(100, 50 + (engineAnalysis.evaluation / 10)));
                
                return (
                    <div className="engine-analysis">
                        <h3 className="text-lg font-bold mb-3">引擎分析</h3>
                        
                        <div className="space-y-3">
                            <div>
                                <div className="flex justify-between text-sm mb-1">
                                    <span>局面评估</span>
                                    <span>{engineAnalysis.evaluation > 0 ? '+' : ''}{(engineAnalysis.evaluation / 100).toFixed(2)}</span>
                                </div>
                                <div className="evaluation-bar">
                                    <div 
                                        className="eval-indicator"
                                        style={{left: `${evalPercentage}%`}}
                                    ></div>
                                </div>
                            </div>
                            
                            <div>
                                <div className="text-sm">最佳变化</div>
                                <div className="font-mono text-sm">{engineAnalysis.principal_variation}</div>
                            </div>
                            
                            <div>
                                <div className="text-sm">搜索深度: {engineAnalysis.depth}</div>
                                <div className="text-sm">思考时间: {engineAnalysis.thinking_time?.toFixed(1)}s</div>
                            </div>
                        </div>
                    </div>
                );
            };

            return (
                <div className="container mx-auto px-4 py-8">
                    <div className="text-center mb-8">
                        <h1 className="text-4xl font-bold text-gray-800 mb-2">国际象棋机器人</h1>
                        <div className="text-lg">
                            {gameState && (
                                <span className={`px-3 py-1 rounded-full text-white ${
                                    gameState.state === 3 ? 'bg-green-500' : 
                                    gameState.state === 6 ? 'bg-blue-500' : 'bg-gray-500'
                                }`}>
                                    {gameState.state === 3 ? '您的回合' : 
                                     gameState.state === 6 ? '机器人思考中...' : '等待中...'}
                                </span>
                            )}
                        </div>
                    </div>

                    <div className="grid grid-cols-1 xl:grid-cols-4 gap-8">
                        {/* 主棋盘区域 */}
                        <div className="xl:col-span-2 flex justify-center">
                            {/* 着法历史 */}
                            <div className="bg-white p-4 rounded-lg shadow-lg">
                                <h3 className="text-lg font-bold mb-3">着法历史</h3>
                                <div className="move-history">
                                    {moveHistory.length === 0 ? (
                                        <div className="text-gray-500 text-center">暂无着法记录</div>
                                    ) : (
                                        moveHistory.map((move, index) => (
                                            <div key={index} className={`mb-1 ${index % 2 === 0 ? 'font-bold' : ''}`}>
                                                {Math.floor(index / 2) + 1}.{index % 2 === 0 ? ' ' : '... '}{move.algebraic_notation}
                                            </div>
                                        ))
                                    )}
                                </div>
                            </div>

                            {/* 相机监控 */}
                            {cameraFeed && (
                                <div className="bg-white p-4 rounded-lg shadow-lg">
                                    <h3 className="text-lg font-bold mb-3">实时监控</h3>
                                    <img 
                                        src={cameraFeed} 
                                        alt="Chess board camera feed" 
                                        className="w-full rounded border"
                                    />
                                </div>
                            )}
                        </div>
                    </div>
                </div>
            );
        }

        ReactDOM.render(<InternationalChessApp />, document.getElementById('chess-app'));
    </script>
</body>
</html>
```

### 第七阶段：系统优化和测试 (1周)

**Week 16: 系统集成测试**

```python
# test/test_chess_integration.py
import unittest
import rclpy
import chess
import time
from chess_interfaces.msg import ChessMove, BoardState
from chess_interfaces.srv import GetEngineMove, ValidateMove

class TestChessRobotIntegration(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()
        
    def setUp(self):
        self.node = rclpy.create_node('test_chess_integration')
        
        # 服务客户端
        self.engine_client = self.node.create_client(GetEngineMove, 'get_engine_move')
        self.validate_client = self.node.create_client(ValidateMove, 'validate_move')
        
        # 等待服务上线
        self.assertTrue(self.engine_client.wait_for_service(timeout_sec=10.0))
        self.assertTrue(self.validate_client.wait_for_service(timeout_sec=10.0))
    
    def test_engine_opening_moves(self):
        """测试引擎开局移动"""
        # 标准开局位置
        starting_fen = "rnbqkbnr/pppppppp/8/8/8/8/PPPPPPPP/RNBQKBNR w KQkq - 0 1"
        
        request = GetEngineMove.Request()
        request.fen_position = starting_fen
        request.depth = 10
        request.time_limit = 3.0
        
        future = self.engine_client.call_async(request)
        rclpy.spin_until_future_complete(self.node, future, timeout_sec=10.0)
        
        response = future.result()
        self.assertTrue(response.success)
        self.assertIsNotNone(response.best_move)
        self.assertGreater(len(response.algebraic_notation), 0)
        
        # 验证是合法的开局移动
        board = chess.Board(starting_fen)
        legal_moves = [board.san(move) for move in board.legal_moves]
        self.assertIn(response.algebraic_notation, legal_moves)
    
    def test_castling_detection(self):
        """测试王车易位检测"""
        # 可以王翼易位的局面
        castling_fen = "r3k2r/pppppppp/8/8/8/8/PPPPPPPP/R3K2R w KQkq - 0 1"
        
        # 创建王翼易位移动
        move = ChessMove()
        move.from_square = chess.E1
        move.to_square = chess.G1
        move.piece_type = 1  # King
        move.is_castling = True
        move.algebraic_notation = "O-O"
        
        request = ValidateMove.Request()
        request.move = move
        request.current_fen = castling_fen
        
        future = self.validate_client.call_async(request)
        rclpy.spin_until_future_complete(self.node, future, timeout_sec=5.0)
        
        response = future.result()
        self.assertTrue(response.is_legal)
        self.assertFalse(response.is_checkmate)
    
    def test_checkmate_detection(self):
        """测试将死检测"""
        # 学者杀死法
        mate_in_one_fen = "rnb1kbnr/pppp1ppp/8/4p3/6Pq/5P2/PPPPP2P/RNBQKBNR w KQkq - 1 3"
        
        board = chess.Board(mate_in_one_fen)
        self.assertTrue(board.is_checkmate())
        
    def test_en_passant_move(self):
        """测试吃过路兵"""
        # 设置可以吃过路兵的局面
        en_passant_fen = "rnbqkbnr/ppp1p1pp/8/3pPp2/8/8/PPPP1PPP/RNBQKBNR w KQkq f6 0 3"
        
        move = ChessMove()
        move.from_square = chess.E5
        move.to_square = chess.F6
        move.piece_type = 6  # Pawn
        move.is_en_passant = True
        move.algebraic_notation = "exf6"
        
        request = ValidateMove.Request()
        request.move = move
        request.current_fen = en_passant_fen
        
        future = self.validate_client.call_async(request)
        rclpy.spin_until_future_complete(self.node, future, timeout_sec=5.0)
        
        response = future.result()
        self.assertTrue(response.is_legal)
    
    def test_pawn_promotion(self):
        """测试兵升变"""
        # 白兵可以升变的局面
        promotion_fen = "8/P7/8/8/8/8/8/8 w - - 0 1"
        
        move = ChessMove()
        move.from_square = chess.A7
        move.to_square = chess.A8
        move.piece_type = 6  # Pawn
        move.promotion = 2   # Queen
        move.algebraic_notation = "a8=Q"
        
        request = ValidateMove.Request()
        request.move = move
        request.current_fen = promotion_fen
        
        future = self.validate_client.call_async(request)
        rclpy.spin_until_future_complete(self.node, future, timeout_sec=5.0)
        
        response = future.result()
        self.assertTrue(response.is_legal)
    
    def tearDown(self):
        self.node.destroy_node()
    
    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()


class TestChessVisionSystem(unittest.TestCase):
    """测试视觉系统"""
    
    def test_board_detection_accuracy(self):
        """测试棋盘检测精度"""
        # 加载测试图像
        test_images = [
            'test_data/initial_position.jpg',
            'test_data/middle_game.jpg',
            'test_data/endgame.jpg'
        ]
        
        for image_path in test_images:
            with self.subTest(image=image_path):
                # 这里应该调用实际的视觉检测服务
                # 验证检测精度是否满足要求
                pass
    
    def test_piece_classification(self):
        """测试棋子分类精度"""
        piece_types = ['king', 'queen', 'rook', 'bishop', 'knight', 'pawn']
        colors = ['white', 'black']
        
        for piece in piece_types:
            for color in colors:
                test_image_path = f'test_data/pieces/{color}_{piece}.jpg'
                # 测试分类精度
                # self.assertGreaterEqual(confidence, 0.9)


if __name__ == '__main__':
    unittest.main()
```

### 完整启动配置

```python
# launch/chess_robot_complete.launch.py
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    return LaunchDescription([
        # 声明参数
        DeclareLaunchArgument('debug', default_value='false'),
        DeclareLaunchArgument('simulation', default_value='false'),
        DeclareLaunchArgument('web_interface', default_value='true'),
        
        # 1. 相机节点
        Node(
            package='chess_camera',
            executable='camera_node',
            name='camera_node',
            parameters=[{
                'frame_rate': 10.0,
                'auto_exposure': True,
                'depth_enabled': True
            }],
            condition=UnlessCondition(LaunchConfiguration('simulation'))
        ),
        
        # 2. 视觉识别系统
        Node(
            package='chess_vision',
            executable='board_detector',
            name='board_detector',
            parameters=[{
                'aruco_dict': 'DICT_4X4_50',
                'board_size': 8,
                'square_size': 0.05,  # 5cm squares
                'detection_threshold': 0.7
            }]
        ),
        
        Node(
            package='chess_vision',
            executable='piece_classifier',
            name='piece_classifier',
            parameters=[{
                'model_path': '/home/chess/models/international_chess_classifier.trt',
                'confidence_threshold': 0.85,
                'batch_size': 1
            }]
        ),
        
        # 3. Stockfish引擎
        Node(
            package='chess_engine',
            executable='stockfish_node',
            name='stockfish_engine',
            parameters=[{
                'engine_path': '/usr/games/stockfish',
                'default_depth': 15,
                'default_time': 5.0,
                'skill_level': 20,
                'threads': 2
            }]
        ),
        
        # 4. MoveIt2机械臂系统
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                get_package_share_directory('chess_arm'), 
                '/launch/dofbot_pro_moveit.launch.py'
            ]),
            condition=UnlessCondition(LaunchConfiguration('simulation'))
        ),
        
        Node(
            package='chess_arm',
            executable='chess_arm_controller',
            name='chess_arm_controller',
            parameters=[{
                'speed_factor': 0.2,
                'safety_mode': True,
                'collision_detection': True,
                'gripper_force': 10.0
            }],
            condition=UnlessCondition(LaunchConfiguration('simulation'))
        ),
        
        # 5. 游戏协调器 - 核心控制节点
        Node(
            package='chess_coordinator',
            executable='chess_game_coordinator',
            name='chess_game_coordinator',
            parameters=[{
                'auto_start': False,
                'human_plays_white': True,
                'engine_difficulty': 15,
                'debug_mode': LaunchConfiguration('debug'),
                'move_timeout': 300.0,  # 5分钟超时
                'log_games': True
            }]
        ),
        
        # 6. 性能监控
        Node(
            package='chess_coordinator',
            executable='performance_monitor',
            name='performance_monitor',
            parameters=[{
                'monitor_interval': 1.0,
                'log_performance': True,
                'alert_thresholds': {
                    'cpu_usage': 80.0,
                    'memory_usage': 85.0,
                    'vision_fps': 5.0
                }
            }]
        ),
        
        # 7. Web界面桥接
        Node(
            package='chess_ui',
            executable='web_bridge',
            name='web_bridge',
            parameters=[{
                'web_port': 8080,
                'web_host': '0.0.0.0',
                'camera_stream_fps': 5.0,
                'enable_cors': True,
                'api_auth': False  # 开发模式
            }],
            condition=IfCondition(LaunchConfiguration('web_interface'))
        ),
        
        # 8. 静态变换发布
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments=[
                '0.2', '0.0', '0.02',  # x, y, z
                '0', '0', '0', '1',     # qx, qy, qz, qw
                'base_link', 'chessboard_link'
            ]
        ),
        
        # 9. Robot State Publisher
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[{
                'robot_description': Command(['xacro ', 
                    get_package_share_directory('chess_arm'), 
                    '/urdf/dofbot_pro_chess.urdf.xacro'])
            }],
            condition=UnlessCondition(LaunchConfiguration('simulation'))
        ),
        
        # 10. RViz可视化 (调试模式)
        Node(
            package='rviz2',
            executable='rviz2',
            arguments=['-d', os.path.join(
                get_package_share_directory('chess_coordinator'),
                'rviz', 'chess_robot.rviz'
            )],
            condition=IfCondition(LaunchConfiguration('debug'))
        ),
        
        # 11. 数据记录 (可选)
        ExecuteProcess(
            cmd=['ros2', 'bag', 'record', '-a', '-o', '/home/chess/bags/chess_session'],
            condition=IfCondition(LaunchConfiguration('debug'))
        )
    ])
```

## 项目关键改进点

### 国际象棋特定优化

1. **规则完整性**
   - 完整实现国际象棋所有规则
   - 特殊移动：王车易位、吃过路兵、兵升变
   - 胜负判定：将军、将死、僵局、和棋条件

2. **视觉识别精度**
   - 针对国际象棋棋子的专门训练
   - 区分6种不同棋子类型
   - 黑白棋子的可靠区分

3. **机械臂精度要求**
   - 适应不同棋子高度的抓取策略
   - 王车易位的复合移动实现
   - 吃过路兵的特殊处理

### 技术架构优势

1. **ROS2现代化架构**
   - 更好的实时性和可靠性
   - 标准化的消息和服务接口
   - 分布式系统支持

2. **模块化设计**
   - 每个功能独立测试和部署
   - 便于维护和升级
   - 支持不同硬件配置

3. **Web界面集成**
   - 实时游戏监控
   - 远程控制能力
   - 移动端支持

### 部署和运维

```bash
# 快速启动脚本
#!/bin/bash
echo "启动国际象棋机器人系统..."

# 设置环境
source /opt/ros/humble/setup.bash
source ~/chess_robot_ws/install/setup.bash
export ROS_DOMAIN_ID=42

# 检查硬件连接
echo "检查硬件状态..."
if lsusb | grep -q "DABAI"; then
    echo "✅ 深度相机连接正常"
else
    echo "❌ 深度相机未检测到"
fi

# 启动完整系统
ros2 launch chess_robot chess_robot_complete.launch.py web_interface:=true

echo "系统启动完成！访问: http://$(hostname -I | awk '{print $1}'):8080"
```

## 总结

这个基于ROS2的国际象棋机器人项目提供了：

1. **完整的国际象棋支持**：包含所有规则和特殊移动
2. **现代化ROS2架构**：模块化、可扩展、高可靠性
3. **智能视觉系统**：精确的棋盘和棋子识别
4. **精密机械控制**：适配国际象棋的专用机械臂控制
5. **强大的AI引擎**：集成Stockfish提供不同难度级别
6. **友好的Web界面**：实时监控和远程控制
7. **完整的测试框架**：确保系统可靠性
8. **生产级部署**：包含监控、日志、错误处理

这个17周的实施计划为构建一个功能完整、技术先进的国际象棋机器人系统提供了详细的路线图。-white p-4 rounded-lg shadow-lg">
                                {renderChessBoard()}
                            </div>
                        </div>

                        {/* 右侧信息面板 */}
                        <div className="xl:col-span-2 space-y-6">
                            {/* 游戏控制 */}
                            {renderGameControls()}
                            
                            {/* 被吃棋子 */}
                            <div className="bg-white p-4 rounded-lg shadow-lg">
                                <h3 className="text-lg font-bold mb-3">被吃棋子</h3>
                                <div className="space-y-2">
                                    <div>
                                        <span className="text-sm text-gray-600">白方: </span>
                                        <div className="captured-pieces">
                                            {capturedPieces.white.map((piece, i) => (
                                                <span key={i} className="captured-piece">{piece}</span>
                                            ))}
                                        </div>
                                    </div>
                                    <div>
                                        <span className="text-sm text-gray-600">黑方: </span>
                                        <div className="captured-pieces">
                                            {capturedPieces.black.map((piece, i) => (
                                                <span key={i} className="captured-piece">{piece}</span>
                                            ))}
                                        </div>
                                    </div>
                                </div>
                            </div>

                            {/* 引擎分析 */}
                            {renderEngineAnalysis()}
                            
                            {/* 着法历史 */}
                            <div className="bg