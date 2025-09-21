# 基于ROS2的象棋机器人项目实施规划

## 项目概述

**项目目标**：基于ROS2构建智能象棋机器人系统，实现全自动象棋对弈功能

**核心技术栈**：
- **ROS2 Humble** (主要框架)
- **Jetson Nano 4GB** (边缘计算平台)  
- **DOFBOT Pro** (6DOF机械臂)
- **DABAI DCW2** (深度相机)
- **MoveIt2** (运动规划)
- **OpenCV + AI模型** (计算机视觉)

## ROS2架构设计

### 系统节点架构
```
┌─────────────────────────────────────────────────────────────┐
│                        ROS2 网络                             │
├─────────────────────────────────────────────────────────────┤
│  camera_node  │  vision_node  │  game_node  │  arm_node    │
│   (传感器)     │   (视觉AI)    │  (游戏逻辑)  │  (运动控制)   │
└─────────────────────────────────────────────────────────────┘
      ↓              ↓            ↓           ↓
   RGB+深度图     棋盘状态      AI决策      机械臂控制
```

### 核心ROS2包结构
```
chess_robot_ws/
├── src/
│   ├── chess_interfaces/         # 自定义消息和服务
│   ├── chess_camera/            # 相机驱动和数据发布
│   ├── chess_vision/            # 视觉识别和AI推理
│   ├── chess_game/              # 游戏逻辑和引擎
│   ├── chess_arm/               # 机械臂控制
│   ├── chess_planner/           # MoveIt2运动规划
│   ├── chess_coordinator/       # 系统协调和状态机
│   └── chess_ui/                # 用户界面和监控
├── config/                      # 配置文件
├── launch/                      # 启动文件
└── params/                      # 参数文件
```

## 自定义ROS2接口设计

### 消息类型 (chess_interfaces/msg/)

**BoardState.msg** - 棋盘状态
```
Header header
int8[64] board_squares  # 0=空, 1-6=白方棋子, -1到-6=黑方棋子
geometry_msgs/Point[64] square_positions_3d  # 每个格子的3D坐标
bool white_to_move
bool castling_rights[4]  # KQkq
int8 en_passant_square   # -1表示无
int32 halfmove_clock
int32 fullmove_number
```

**ChessMove.msg** - 象棋移动
```
Header header
int8 from_square    # 0-63
int8 to_square      # 0-63
int8 piece_type     # 移动的棋子类型
int8 captured_piece # 被吃的棋子(0表示无)
int8 promotion      # 兵升变类型(0表示无)
bool is_castling    # 是否为易位
bool is_en_passant  # 是否为吃过路兵
float32 confidence  # 识别置信度
```

**PieceDetection.msg** - 棋子检测结果
```
Header header
geometry_msgs/Point position_3d
geometry_msgs/Point2D position_2d
int8 piece_type     # 1=王, 2=后, 3=车, 4=象, 5=马, 6=兵
int8 piece_color    # 1=白, -1=黑, 0=未知
float32 confidence
sensor_msgs/Image roi_image
```

### 服务类型 (chess_interfaces/srv/)

**DetectBoard.srv** - 棋盘检测服务
```
sensor_msgs/Image rgb_image
sensor_msgs/Image depth_image
---
bool success
string error_message
BoardState board_state
geometry_msgs/TransformStamped board_transform
```

**PlanMove.srv** - 运动规划服务
```
ChessMove chess_move
float32 speed_factor    # 0.1-1.0
bool avoid_pieces       # 是否避开其他棋子
---
bool success
string error_message
moveit_msgs/RobotTrajectory trajectory
float32 execution_time
```

**ExecuteMove.srv** - 执行移动服务
```
ChessMove chess_move
moveit_msgs/RobotTrajectory trajectory
---
bool success
string error_message
float32 actual_execution_time
```

## 详细实施计划（16周）

### 第一阶段：ROS2环境搭建 (2周)

**Week 1: 系统基础搭建**
```bash
# Jetson Nano上安装ROS2 Humble
curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu jammy main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
sudo apt update && sudo apt install ros-humble-desktop

# 安装关键依赖
sudo apt install python3-colcon-common-extensions
sudo apt install ros-humble-moveit ros-humble-moveit-visual-tools
sudo apt install ros-humble-robot-state-publisher ros-humble-joint-state-publisher
sudo apt install ros-humble-cv-bridge ros-humble-image-transport
```

**Week 2: 工作空间创建和基础包**
```bash
# 创建工作空间
mkdir -p ~/chess_robot_ws/src
cd ~/chess_robot_ws

# 创建基础包
ros2 pkg create --build-type ament_python chess_interfaces
ros2 pkg create --build-type ament_python chess_camera --dependencies rclpy sensor_msgs cv_bridge
ros2 pkg create --build-type ament_python chess_vision --dependencies rclpy opencv2
ros2 pkg create --build-type ament_python chess_game --dependencies rclpy
ros2 pkg create --build-type ament_python chess_arm --dependencies rclpy moveit_msgs
ros2 pkg create --build-type ament_python chess_coordinator --dependencies rclpy

# 首次编译
colcon build
source install/setup.bash
```

**验收标准**：
- ROS2系统正常运行
- 所有基础包编译成功
- DOFBOT Pro在ROS2下正常通信

### 第二阶段：相机驱动和数据流 (2周)

**Week 3: DABAI DCW2 ROS2驱动开发**

```python
# chess_camera/chess_camera/camera_node.py
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge
import cv2
import numpy as np

class CameraNode(Node):
    def __init__(self):
        super().__init__('camera_node')
        
        # 发布器
        self.rgb_pub = self.create_publisher(Image, 'camera/rgb/image_raw', 10)
        self.depth_pub = self.create_publisher(Image, 'camera/depth/image_raw', 10)
        self.info_pub = self.create_publisher(CameraInfo, 'camera/camera_info', 10)
        
        # 定时器
        self.timer = self.create_timer(0.1, self.capture_and_publish)  # 10Hz
        
        # CV Bridge
        self.bridge = CvBridge()
        
        # 初始化相机
        self.init_camera()
    
    def init_camera(self):
        """初始化DABAI DCW2深度相机"""
        # 这里添加具体的相机初始化代码
        pass
    
    def capture_and_publish(self):
        """采集并发布图像数据"""
        # 获取RGB和深度图像
        rgb_image = self.get_rgb_image()
        depth_image = self.get_depth_image()
        
        if rgb_image is not None and depth_image is not None:
            # 转换为ROS消息
            rgb_msg = self.bridge.cv2_to_imgmsg(rgb_image, 'bgr8')
            depth_msg = self.bridge.cv2_to_imgmsg(depth_image, '16UC1')
            
            # 添加时间戳
            now = self.get_clock().now().to_msg()
            rgb_msg.header.stamp = now
            depth_msg.header.stamp = now
            rgb_msg.header.frame_id = 'camera_link'
            depth_msg.header.frame_id = 'camera_link'
            
            # 发布
            self.rgb_pub.publish(rgb_msg)
            self.depth_pub.publish(depth_msg)
```

**Week 4: 相机标定和坐标变换**

```python
# chess_camera/chess_camera/calibration_node.py
class CalibrationNode(Node):
    def __init__(self):
        super().__init__('calibration_node')
        
        # 服务
        self.calibrate_srv = self.create_service(
            Trigger, 'calibrate_camera', self.calibrate_callback
        )
        
        # TF广播器
        self.tf_broadcaster = TransformBroadcaster(self)
    
    def calibrate_callback(self, request, response):
        """执行相机标定"""
        # 9x6棋盘格标定
        success = self.perform_calibration()
        
        if success:
            # 发布camera_link到base_link的变换
            self.publish_camera_transform()
            response.success = True
            response.message = "Camera calibration successful"
        else:
            response.success = False
            response.message = "Camera calibration failed"
        
        return response
```

### 第三阶段：视觉识别系统 (4周)

**Week 5-6: 棋盘检测算法**

```python
# chess_vision/chess_vision/board_detector.py
class BoardDetector(Node):
    def __init__(self):
        super().__init__('board_detector')
        
        # 订阅相机数据
        self.rgb_sub = self.create_subscription(
            Image, 'camera/rgb/image_raw', self.image_callback, 10
        )
        self.depth_sub = self.create_subscription(
            Image, 'camera/depth/image_raw', self.depth_callback, 10
        )
        
        # 服务
        self.detect_srv = self.create_service(
            DetectBoard, 'detect_board', self.detect_board_callback
        )
        
        # 发布棋盘状态
        self.board_pub = self.create_publisher(BoardState, 'board_state', 10)
        
        self.bridge = CvBridge()
        self.latest_rgb = None
        self.latest_depth = None
    
    def detect_board_callback(self, request, response):
        """检测棋盘服务回调"""
        rgb_image = self.bridge.imgmsg_to_cv2(request.rgb_image, 'bgr8')
        depth_image = self.bridge.imgmsg_to_cv2(request.depth_image, '16UC1')
        
        # 检测ArUco标记
        aruco_corners = self.detect_aruco_markers(rgb_image)
        
        if len(aruco_corners) >= 4:
            # 计算棋盘变换矩阵
            board_transform = self.compute_board_transform(aruco_corners, depth_image)
            
            # 分割64个格子
            square_positions = self.extract_square_positions(board_transform)
            
            # 检测每个格子的棋子
            board_state = self.detect_pieces_on_squares(rgb_image, depth_image, square_positions)
            
            response.success = True
            response.board_state = board_state
            response.board_transform = board_transform
        else:
            response.success = False
            response.error_message = "Cannot detect board corners"
        
        return response
```

**Week 7-8: 棋子识别和分类**

```python
# chess_vision/chess_vision/piece_classifier.py
class PieceClassifier(Node):
    def __init__(self):
        super().__init__('piece_classifier')
        
        # 加载训练好的CNN模型
        self.model = self.load_piece_classification_model()
        
        # 服务
        self.classify_srv = self.create_service(
            ClassifyPiece, 'classify_piece', self.classify_callback
        )
    
    def load_piece_classification_model(self):
        """加载棋子分类模型"""
        # 加载TensorRT优化的模型（用于Jetson Nano）
        import tensorrt as trt
        import pycuda.driver as cuda
        
        # 这里加载预训练的棋子识别模型
        # 模型输入：RGB图像 + 深度信息
        # 模型输出：12类（6种棋子 × 2种颜色）+ 置信度
        pass
    
    def classify_callback(self, request, response):
        """棋子分类服务"""
        roi_image = self.bridge.imgmsg_to_cv2(request.roi_image, 'bgr8')
        
        # 预处理图像
        processed_image = self.preprocess_image(roi_image)
        
        # 推理
        piece_type, piece_color, confidence = self.model.predict(processed_image)
        
        response.piece_type = piece_type
        response.piece_color = piece_color
        response.confidence = confidence
        
        return response
```

### 第四阶段：MoveIt2机械臂控制 (3周)

**Week 9: DOFBOT Pro MoveIt2配置**

```yaml
# config/dofbot_pro.urdf.xacro
<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro" name="dofbot_pro">
  
  <!-- 基础参数 -->
  <xacro:property name="base_width" value="0.1"/>
  <xacro:property name="base_height" value="0.05"/>
  
  <!-- 关节定义 -->
  <joint name="joint1" type="revolute">
    <parent link="base_link"/>
    <child link="link1"/>
    <origin xyz="0 0 0.06" rpy="0 0 0"/>
    <axis xyz="0 0 1"/>
    <limit lower="-3.14159" upper="3.14159" effort="10" velocity="1.0"/>
  </joint>
  
  <!-- 其他5个关节... -->
  
  <!-- 末端执行器 -->
  <joint name="gripper_joint" type="prismatic">
    <parent link="link6"/>
    <child link="gripper_link"/>
    <origin xyz="0 0 0.1" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <limit lower="0" upper="0.04" effort="10" velocity="0.1"/>
  </joint>
  
</robot>
```

```yaml
# config/dofbot_pro_moveit.yaml
move_group:
  arm:
    joints: [joint1, joint2, joint3, joint4, joint5, joint6]
    kinematics_solver: kdl_kinematics_plugin/KDLKinematicsPlugin
    kinematics_solver_search_resolution: 0.005
    kinematics_solver_timeout: 0.05
  
  gripper:
    joints: [gripper_joint]
    kinematics_solver: kdl_kinematics_plugin/KDLKinematicsPlugin
```

**Week 10-11: 机械臂控制节点**

```python
# chess_arm/chess_arm/arm_controller.py
import moveit_commander
from moveit_msgs.msg import RobotTrajectory
from geometry_msgs.msg import Pose, Point, Quaternion

class ArmController(Node):
    def __init__(self):
        super().__init__('arm_controller')
        
        # MoveIt初始化
        moveit_commander.roscpp_initialize(sys.argv)
        self.robot = moveit_commander.RobotCommander()
        self.scene = moveit_commander.PlanningSceneInterface()
        self.arm_group = moveit_commander.MoveGroupCommander("arm")
        self.gripper_group = moveit_commander.MoveGroupCommander("gripper")
        
        # 设置规划参数
        self.arm_group.set_planning_time(5.0)
        self.arm_group.set_num_planning_attempts(10)
        self.arm_group.set_max_velocity_scaling_factor(0.5)
        self.arm_group.set_max_acceleration_scaling_factor(0.5)
        
        # 服务
        self.plan_srv = self.create_service(PlanMove, 'plan_move', self.plan_move_callback)
        self.execute_srv = self.create_service(ExecuteMove, 'execute_move', self.execute_move_callback)
        
        # 添加棋盘碰撞物体
        self.add_chessboard_collision_objects()
    
    def plan_move_callback(self, request, response):
        """规划机械臂运动"""
        chess_move = request.chess_move
        
        # 转换为机械臂坐标
        from_pose = self.square_to_pose(chess_move.from_square)
        to_pose = self.square_to_pose(chess_move.to_square)
        
        # 规划抓取-移动-放置轨迹
        trajectory = self.plan_pick_and_place(from_pose, to_pose, request.avoid_pieces)
        
        if trajectory:
            response.success = True
            response.trajectory = trajectory
        else:
            response.success = False
            response.error_message = "Motion planning failed"
        
        return response
    
    def plan_pick_and_place(self, from_pose, to_pose, avoid_pieces):
        """规划抓取和放置轨迹"""
        waypoints = []
        
        # 1. 移动到抓取位置上方
        pre_grasp_pose = copy.deepcopy(from_pose)
        pre_grasp_pose.position.z += 0.05
        waypoints.append(pre_grasp_pose)
        
        # 2. 下降到抓取位置
        waypoints.append(from_pose)
        
        # 3. 抓取后上升
        post_grasp_pose = copy.deepcopy(from_pose)
        post_grasp_pose.position.z += 0.08
        waypoints.append(post_grasp_pose)
        
        # 4. 移动到放置位置上方
        pre_place_pose = copy.deepcopy(to_pose)
        pre_place_pose.position.z += 0.08
        waypoints.append(pre_place_pose)
        
        # 5. 下降到放置位置
        waypoints.append(to_pose)
        
        # 6. 放置后上升
        post_place_pose = copy.deepcopy(to_pose)
        post_place_pose.position.z += 0.05
        waypoints.append(post_place_pose)
        
        # 计算笛卡尔路径
        (plan, fraction) = self.arm_group.compute_cartesian_path(waypoints, 0.01, 0.0)
        
        return plan if fraction > 0.8 else None
```

### 第五阶段：游戏逻辑和引擎 (2周)

**Week 12-13: 象棋引擎集成**

```python
# chess_game/chess_game/game_engine.py
import chess
import chess.engine
import chess.pgn

class GameEngine(Node):
    def __init__(self):
        super().__init__('game_engine')
        
        # 象棋引擎
        self.engine = chess.engine.SimpleEngine.popen_uci("/usr/local/bin/stockfish")
        self.board = chess.Board()
        
        # 订阅棋盘状态
        self.board_sub = self.create_subscription(
            BoardState, 'board_state', self.board_state_callback, 10
        )
        
        # 发布AI移动
        self.move_pub = self.create_publisher(ChessMove, 'ai_move', 10)
        
        # 服务
        self.validate_srv = self.create_service(ValidateMove, 'validate_move', self.validate_callback)
        self.get_move_srv = self.create_service(GetAIMove, 'get_ai_move', self.get_ai_move_callback)
        
        # 游戏状态
        self.current_board_state = None
        self.game_phase = "waiting"  # waiting, human_turn, ai_thinking, ai_moving
    
    def board_state_callback(self, msg):
        """处理棋盘状态更新"""
        self.current_board_state = msg
        
        # 检测人类走法
        if self.game_phase == "human_turn":
            detected_move = self.detect_human_move(msg)
            if detected_move:
                self.process_human_move(detected_move)
    
    def detect_human_move(self, new_board_state):
        """通过比较前后棋盘状态检测人类走法"""
        if self.previous_board_state is None:
            return None
        
        # 比较两个棋盘状态
        differences = self.find_board_differences(self.previous_board_state, new_board_state)
        
        # 解析为合法走法
        if len(differences) == 2:  # 正常移动：一个位置清空，一个位置放置
            from_square, to_square = self.parse_move_from_differences(differences)
            
            # 验证走法合法性
            move = chess.Move(from_square, to_square)
            if move in self.board.legal_moves:
                return move
        
        return None
    
    def get_ai_move_callback(self, request, response):
        """获取AI走法"""
        self.game_phase = "ai_thinking"
        
        # 使用Stockfish计算最佳走法
        result = self.engine.play(
            self.board, 
            chess.engine.Limit(time=request.thinking_time, depth=request.depth)
        )
        
        if result.move:
            # 转换为ROS消息格式
            chess_move = self.chess_move_to_ros_msg(result.move)
            response.success = True
            response.move = chess_move
            
            # 发布AI移动
            self.move_pub.publish(chess_move)
            self.game_phase = "ai_moving"
        else:
            response.success = False
            response.error_message = "AI engine failed to find a move"
        
        return response
```

### 第六阶段：系统协调和状态机 (2周)

**Week 14-15: 主协调器开发**

```python
# chess_coordinator/chess_coordinator/game_coordinator.py
from enum import Enum
import rclpy
from rclpy.node import Node

class GameState(Enum):
    INITIALIZING = 1
    WAITING_FOR_GAME_START = 2
    HUMAN_TURN = 3
    DETECTING_HUMAN_MOVE = 4
    VALIDATING_HUMAN_MOVE = 5
    AI_THINKING = 6
    AI_MOVING = 7
    GAME_OVER = 8
    ERROR = 9

class GameCoordinator(Node):
    def __init__(self):
        super().__init__('game_coordinator')
        
        self.state = GameState.INITIALIZING
        self.last_board_state = None
        
        # 服务客户端
        self.detect_board_client = self.create_client(DetectBoard, 'detect_board')
        self.validate_move_client = self.create_client(ValidateMove, 'validate_move')
        self.get_ai_move_client = self.create_client(GetAIMove, 'get_ai_move')
        self.plan_move_client = self.create_client(PlanMove, 'plan_move')
        self.execute_move_client = self.create_client(ExecuteMove, 'execute_move')
        
        # 订阅者
        self.board_sub = self.create_subscription(
            BoardState, 'board_state', self.board_callback, 10
        )
        
        # 发布者
        self.status_pub = self.create_publisher(GameStatus, 'game_status', 10)
        
        # 主状态机定时器
        self.state_machine_timer = self.create_timer(0.5, self.state_machine_update)
        
        self.get_logger().info("Game Coordinator initialized")
    
    def state_machine_update(self):
        """主状态机更新"""
        if self.state == GameState.INITIALIZING:
            self.handle_initializing()
        elif self.state == GameState.WAITING_FOR_GAME_START:
            self.handle_waiting_for_start()
        elif self.state == GameState.HUMAN_TURN:
            self.handle_human_turn()
        elif self.state == GameState.DETECTING_HUMAN_MOVE:
            self.handle_detecting_human_move()
        elif self.state == GameState.AI_THINKING:
            self.handle_ai_thinking()
        elif self.state == GameState.AI_MOVING:
            self.handle_ai_moving()
        elif self.state == GameState.GAME_OVER:
            self.handle_game_over()
        elif self.state == GameState.ERROR:
            self.handle_error()
    
    def handle_human_turn(self):
        """处理人类回合"""
        # 持续检测棋盘状态变化
        self.request_board_detection()
        
        # 如果检测到变化，切换到验证状态
        if self.board_changed():
            self.state = GameState.DETECTING_HUMAN_MOVE
    
    def handle_ai_thinking(self):
        """处理AI思考"""
        # 异步请求AI走法
        if not hasattr(self, 'ai_move_future'):
            self.ai_move_future = self.get_ai_move_client.call_async(GetAIMove.Request())
        
        # 检查是否完成
        if self.ai_move_future.done():
            result = self.ai_move_future.result()
            if result.success:
                self.current_ai_move = result.move
                self.state = GameState.AI_MOVING
            else:
                self.get_logger().error(f"AI move failed: {result.error_message}")
                self.state = GameState.ERROR
            
            delattr(self, 'ai_move_future')
    
    def handle_ai_moving(self):
        """处理AI移动执行"""
        # 1. 规划运动
        if not hasattr(self, 'plan_future'):
            plan_request = PlanMove.Request()
            plan_request.chess_move = self.current_ai_move
            plan_request.speed_factor = 0.3
            plan_request.avoid_pieces = True
            self.plan_future = self.plan_move_client.call_async(plan_request)
        
        # 2. 执行运动
        elif self.plan_future.done() and not hasattr(self, 'execute_future'):
            plan_result = self.plan_future.result()
            if plan_result.success:
                execute_request = ExecuteMove.Request()
                execute_request.chess_move = self.current_ai_move
                execute_request.trajectory = plan_result.trajectory
                self.execute_future = self.execute_move_client.call_async(execute_request)
            else:
                self.get_logger().error("Motion planning failed")
                self.state = GameState.ERROR
        
        # 3. 检查执行完成
        elif hasattr(self, 'execute_future') and self.execute_future.done():
            execute_result = self.execute_future.result()
            if execute_result.success:
                self.get_logger().info("AI move executed successfully")
                self.state = GameState.HUMAN_TURN
                # 清理futures
                delattr(self, 'plan_future')
                delattr(self, 'execute_future')
            else:
                self.get_logger().error("Move execution failed")
                self.state = GameState.ERROR
```

### 第七阶段：Web前端界面开发 (2周)

**Week 16: 后端Web服务器**

```python
# chess_ui/chess_ui/web_bridge.py
import rclpy
from rclpy.node import Node
from flask import Flask, render_template, jsonify, request
from flask_socketio import SocketIO, emit
import threading
import json
import base64
import cv2
from cv_bridge import CvBridge

class WebBridge(Node):
    """ROS2与Web前端的桥接节点"""
    
    def __init__(self):
        super().__init__('web_bridge')
        
        # Flask应用和SocketIO
        self.app = Flask(__name__, template_folder='templates', static_folder='static')
        self.app.config['SECRET_KEY'] = 'chess_robot_secret'
        self.socketio = SocketIO(self.app, cors_allowed_origins="*")
        
        # CV Bridge
        self.bridge = CvBridge()
        
        # ROS2订阅者
        self.board_sub = self.create_subscription(
            BoardState, 'board_state', self.board_state_callback, 10
        )
        self.status_sub = self.create_subscription(
            GameStatus, 'game_status', self.game_status_callback, 10
        )
        self.camera_sub = self.create_subscription(
            Image, 'camera/rgb/image_raw', self.camera_callback, 10
        )
        self.perf_sub = self.create_subscription(
            PerformanceMetrics, 'performance_metrics', self.performance_callback, 10
        )
        
        # ROS2服务客户端
        self.start_game_client = self.create_client(Trigger, 'start_game')
        self.reset_game_client = self.create_client(Trigger, 'reset_game')
        self.set_difficulty_client = self.create_client(SetDifficulty, 'set_difficulty')
        
        # 设置Web路由和Socket事件
        self.setup_routes()
        self.setup_socket_events()
        
        # 系统状态
        self.current_board_state = None
        self.current_game_status = None
        self.current_performance = None
        
        self.get_logger().info("Web Bridge initialized")
    
    def setup_routes(self):
        """设置HTTP路由"""
        
        @self.app.route('/')
        def index():
            return render_template('chess_game.html')
        
        @self.app.route('/monitor')
        def monitor():
            return render_template('system_monitor.html')
        
        @self.app.route('/api/game_status')
        def get_game_status():
            if self.current_game_status:
                return jsonify({
                    'state': self.current_game_status.state,
                    'current_player': self.current_game_status.current_player,
                    'move_count': self.current_game_status.move_count,
                    'last_move': self.current_game_status.last_move,
                    'is_check': self.current_game_status.is_check,
                    'game_result': self.current_game_status.game_result,
                    'ai_thinking_time': self.current_game_status.ai_thinking_time
                })
            return jsonify({'error': 'No game status available'})
        
        @self.app.route('/api/board_state')
        def get_board_state():
            if self.current_board_state:
                return jsonify({
                    'fen': self.board_state_to_fen(self.current_board_state),
                    'squares': self.current_board_state.board_squares,
                    'white_to_move': self.current_board_state.white_to_move,
                    'castling_rights': self.current_board_state.castling_rights
                })
            return jsonify({'error': 'No board state available'})
        
        @self.app.route('/api/start_game', methods=['POST'])
        def start_game():
            try:
                request_msg = Trigger.Request()
                future = self.start_game_client.call_async(request_msg)
                # 注意：这里应该使用异步处理，简化为同步演示
                return jsonify({'success': True, 'message': 'Game start requested'})
            except Exception as e:
                return jsonify({'success': False, 'error': str(e)})
        
        @self.app.route('/api/reset_game', methods=['POST'])
        def reset_game():
            try:
                request_msg = Trigger.Request()
                future = self.reset_game_client.call_async(request_msg)
                return jsonify({'success': True, 'message': 'Game reset requested'})
            except Exception as e:
                return jsonify({'success': False, 'error': str(e)})
        
        @self.app.route('/api/set_difficulty', methods=['POST'])
        def set_difficulty():
            try:
                data = request.get_json()
                difficulty = data.get('difficulty', 5)
                
                request_msg = SetDifficulty.Request()
                request_msg.level = difficulty
                future = self.set_difficulty_client.call_async(request_msg)
                
                return jsonify({'success': True, 'message': f'Difficulty set to {difficulty}'})
            except Exception as e:
                return jsonify({'success': False, 'error': str(e)})
    
    def setup_socket_events(self):
        """设置WebSocket事件"""
        
        @self.socketio.on('connect')
        def handle_connect():
            emit('status', {'message': 'Connected to chess robot'})
            self.get_logger().info('Web client connected')
        
        @self.socketio.on('disconnect')
        def handle_disconnect():
            self.get_logger().info('Web client disconnected')
        
        @self.socketio.on('request_camera_stream')
        def handle_camera_request():
            # 客户端请求相机流
            emit('camera_stream_started', {'status': 'Camera stream active'})
    
    def board_state_callback(self, msg):
        """棋盘状态回调"""
        self.current_board_state = msg
        
        # 实时推送到Web客户端
        board_data = {
            'fen': self.board_state_to_fen(msg),
            'squares': msg.board_squares,
            'white_to_move': msg.white_to_move,
            'timestamp': msg.header.stamp.sec
        }
        self.socketio.emit('board_update', board_data)
    
    def game_status_callback(self, msg):
        """游戏状态回调"""
        self.current_game_status = msg
        
        # 实时推送到Web客户端
        status_data = {
            'state': msg.state,
            'current_player': msg.current_player,
            'move_count': msg.move_count,
            'last_move': msg.last_move,
            'is_check': msg.is_check,
            'game_result': msg.game_result
        }
        self.socketio.emit('game_status_update', status_data)
    
    def camera_callback(self, msg):
        """相机图像回调"""
        try:
            # 转换为OpenCV格式
            cv_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
            
            # 压缩为JPEG
            _, buffer = cv2.imencode('.jpg', cv_image, [cv2.IMWRITE_JPEG_QUALITY, 80])
            
            # Base64编码
            img_base64 = base64.b64encode(buffer).decode('utf-8')
            
            # 推送到Web客户端（限制频率，避免过载）
            self.socketio.emit('camera_frame', {
                'image': f'data:image/jpeg;base64,{img_base64}',
                'timestamp': msg.header.stamp.sec
            })
        except Exception as e:
            self.get_logger().error(f'Camera callback error: {e}')
    
    def performance_callback(self, msg):
        """性能指标回调"""
        self.current_performance = msg
        
        # 推送性能数据到监控页面
        perf_data = {
            'vision_fps': msg.vision_fps,
            'cpu_usage': msg.cpu_usage,
            'memory_usage': msg.memory_usage,
            'gpu_usage': msg.gpu_usage,
            'planning_time': msg.planning_time,
            'execution_time': msg.execution_time
        }
        self.socketio.emit('performance_update', perf_data)
    
    def board_state_to_fen(self, board_state):
        """转换棋盘状态为FEN字符串"""
        # 简化的FEN转换逻辑
        piece_map = {
            1: 'K', 2: 'Q', 3: 'R', 4: 'B', 5: 'N', 6: 'P',  # 白方
            -1: 'k', -2: 'q', -3: 'r', -4: 'b', -5: 'n', -6: 'p'  # 黑方
        }
        
        fen_parts = []
        for row in range(8):
            fen_row = ""
            empty_count = 0
            for col in range(8):
                square_idx = row * 8 + col
                piece = board_state.board_squares[square_idx]
                
                if piece == 0:  # 空格
                    empty_count += 1
                else:
                    if empty_count > 0:
                        fen_row += str(empty_count)
                        empty_count = 0
                    fen_row += piece_map.get(piece, '?')
            
            if empty_count > 0:
                fen_row += str(empty_count)
            
            fen_parts.append(fen_row)
        
        board_fen = '/'.join(fen_parts)
        active_color = 'w' if board_state.white_to_move else 'b'
        
        # 简化的FEN，实际项目中需要完整实现
        return f"{board_fen} {active_color} KQkq - 0 1"
    
    def run_web_server(self):
        """运行Web服务器"""
        self.socketio.run(self.app, host='0.0.0.0', port=8080, debug=False)


def main():
    rclpy.init()
    
    # 创建Web桥接节点
    web_bridge = WebBridge()
    
    # 在单独线程中运行ROS2
    ros_thread = threading.Thread(target=lambda: rclpy.spin(web_bridge))
    ros_thread.daemon = True
    ros_thread.start()
    
    # 运行Web服务器（主线程）
    web_bridge.run_web_server()
    
    web_bridge.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

**Week 17: 前端React应用开发**

```html
<!-- chess_ui/templates/chess_game.html -->
<!DOCTYPE html>
<html lang="zh-CN">
<head>
    <meta charset="UTF-8">
    <meta name="viewport" content="width=device-width, initial-scale=1.0">
    <title>象棋机器人控制台</title>
    <script src="https://unpkg.com/react@18/umd/react.development.js"></script>
    <script src="https://unpkg.com/react-dom@18/umd/react-dom.development.js"></script>
    <script src="https://unpkg.com/@babel/standalone/babel.min.js"></script>
    <script src="https://cdnjs.cloudflare.com/ajax/libs/socket.io/4.7.2/socket.io.js"></script>
    <link href="https://cdn.jsdelivr.net/npm/tailwindcss@2.2.19/dist/tailwind.min.css" rel="stylesheet">
    <style>
        .chess-square {
            width: 60px;
            height: 60px;
            display: flex;
            align-items: center;
            justify-content: center;
            font-size: 36px;
            cursor: pointer;
            transition: background-color 0.2s;
        }
        .chess-square.light { background-color: #f0d9b5; }
        .chess-square.dark { background-color: #b58863; }
        .chess-square.highlight { background-color: #ffff00 !important; }
        .chess-square.last-move { background-color: #90ee90 !important; }
        
        .camera-feed {
            max-width: 100%;
            border-radius: 8px;
            box-shadow: 0 4px 6px rgba(0, 0, 0, 0.1);
        }
        
        .status-indicator {
            width: 12px;
            height: 12px;
            border-radius: 50%;
            display: inline-block;
            margin-right: 8px;
        }
        .status-online { background-color: #10b981; }
        .status-thinking { background-color: #f59e0b; }
        .status-moving { background-color: #3b82f6; }
        .status-error { background-color: #ef4444; }
    </style>
</head>
<body class="bg-gray-100">
    <div id="chess-app"></div>

    <script type="text/babel">
        // 棋子Unicode符号映射
        const PIECE_SYMBOLS = {
            1: '♔', 2: '♕', 3: '♖', 4: '♗', 5: '♘', 6: '♙',  // 白方
            '-1': '♚', '-2': '♛', '-3': '♜', '-4': '♝', '-5': '♞', '-6': '♟'  // 黑方
        };

        // 游戏状态映射
        const GAME_STATES = {
            1: { text: '初始化中', status: 'thinking' },
            2: { text: '等待开始', status: 'online' },
            3: { text: '人类回合', status: 'online' },
            4: { text: '检测移动', status: 'thinking' },
            5: { text: 'AI思考中', status: 'thinking' },
            6: { text: 'AI移动中', status: 'moving' },
            7: { text: '游戏结束', status: 'online' },
            8: { text: '系统错误', status: 'error' }
        };

        // 主应用组件
        function ChessApp() {
            const [socket, setSocket] = React.useState(null);
            const [boardState, setBoardState] = React.useState(null);
            const [gameStatus, setGameStatus] = React.useState(null);
            const [cameraImage, setCameraImage] = React.useState(null);
            const [performance, setPerformance] = React.useState(null);
            const [connectionStatus, setConnectionStatus] = React.useState('disconnected');
            const [selectedSquare, setSelectedSquare] = React.useState(null);
            const [difficulty, setDifficulty] = React.useState(5);

            // 初始化Socket连接
            React.useEffect(() => {
                const newSocket = io();
                setSocket(newSocket);

                newSocket.on('connect', () => {
                    setConnectionStatus('connected');
                    console.log('Connected to chess robot');
                });

                newSocket.on('disconnect', () => {
                    setConnectionStatus('disconnected');
                });

                newSocket.on('board_update', (data) => {
                    setBoardState(data);
                });

                newSocket.on('game_status_update', (data) => {
                    setGameStatus(data);
                });

                newSocket.on('camera_frame', (data) => {
                    setCameraImage(data.image);
                });

                newSocket.on('performance_update', (data) => {
                    setPerformance(data);
                });

                // 请求相机流
                newSocket.emit('request_camera_stream');

                return () => newSocket.close();
            }, []);

            // 开始游戏
            const handleStartGame = async () => {
                try {
                    const response = await fetch('/api/start_game', { method: 'POST' });
                    const result = await response.json();
                    if (!result.success) {
                        alert('启动游戏失败: ' + result.error);
                    }
                } catch (error) {
                    alert('启动游戏失败: ' + error.message);
                }
            };

            // 重置游戏
            const handleResetGame = async () => {
                try {
                    const response = await fetch('/api/reset_game', { method: 'POST' });
                    const result = await response.json();
                    if (!result.success) {
                        alert('重置游戏失败: ' + result.error);
                    }
                } catch (error) {
                    alert('重置游戏失败: ' + error.message);
                }
            };

            // 设置难度
            const handleSetDifficulty = async (level) => {
                try {
                    const response = await fetch('/api/set_difficulty', {
                        method: 'POST',
                        headers: { 'Content-Type': 'application/json' },
                        body: JSON.stringify({ difficulty: level })
                    });
                    const result = await response.json();
                    if (result.success) {
                        setDifficulty(level);
                    } else {
                        alert('设置难度失败: ' + result.error);
                    }
                } catch (error) {
                    alert('设置难度失败: ' + error.message);
                }
            };

            // 棋盘组件
            const ChessBoard = () => {
                if (!boardState) return <div>加载棋盘中...</div>;

                const squares = [];
                for (let row = 0; row < 8; row++) {
                    for (let col = 0; col < 8; col++) {
                        const squareIndex = row * 8 + col;
                        const piece = boardState.squares[squareIndex];
                        const isLight = (row + col) % 2 === 0;
                        const isSelected = selectedSquare === squareIndex;

                        squares.push(
                            <div
                                key={squareIndex}
                                className={`chess-square ${isLight ? 'light' : 'dark'} ${isSelected ? 'highlight' : ''}`}
                                onClick={() => setSelectedSquare(isSelected ? null : squareIndex)}
                            >
                                {piece !== 0 && PIECE_SYMBOLS[piece]}
                            </div>
                        );
                    }
                }

                return (
                    <div className="inline-block border-2 border-gray-800">
                        <div className="grid grid-cols-8 gap-0">
                            {squares}
                        </div>
                    </div>
                );
            };

            // 状态指示器
            const StatusIndicator = () => {
                if (!gameStatus) return null;

                const stateInfo = GAME_STATES[gameStatus.state] || { text: '未知状态', status: 'error' };
                
                return (
                    <div className="flex items-center">
                        <span className={`status-indicator status-${stateInfo.status}`}></span>
                        <span className="font-medium">{stateInfo.text}</span>
                    </div>
                );
            };

            // 性能监控组件
            const PerformanceMonitor = () => {
                if (!performance) return null;

                return (
                    <div className="bg-white p-4 rounded-lg shadow">
                        <h3 className="text-lg font-semibold mb-3">系统性能</h3>
                        <div className="space-y-2">
                            <div className="flex justify-between">
                                <span>视觉处理:</span>
                                <span>{performance.vision_fps.toFixed(1)} FPS</span>
                            </div>
                            <div className="flex justify-between">
                                <span>CPU使用率:</span>
                                <span>{performance.cpu_usage.toFixed(1)}%</span>
                            </div>
                            <div className="flex justify-between">
                                <span>内存使用率:</span>
                                <span>{performance.memory_usage.toFixed(1)}%</span>
                            </div>
                            <div className="flex justify-between">
                                <span>GPU使用率:</span>
                                <span>{performance.gpu_usage.toFixed(1)}%</span>
                            </div>
                            <div className="flex justify-between">
                                <span>规划时间:</span>
                                <span>{performance.planning_time.toFixed(2)}s</span>
                            </div>
                        </div>
                    </div>
                );
            };

            return (
                <div className="container mx-auto px-4 py-8">
                    <div className="text-center mb-8">
                        <h1 className="text-4xl font-bold text-gray-800 mb-2">象棋机器人控制台</h1>
                        <div className="flex justify-center items-center space-x-4">
                            <StatusIndicator />
                            <div className="text-sm text-gray-600">
                                连接状态: 
                                <span className={connectionStatus === 'connected' ? 'text-green-600' : 'text-red-600'}>
                                    {connectionStatus === 'connected' ? '已连接' : '未连接'}
                                </span>
                            </div>
                        </div>
                    </div>

                    <div className="grid grid-cols-1 lg:grid-cols-3 gap-8">
                        {/* 棋盘区域 */}
                        <div className="lg:col-span-2">
                            <div className="bg-white p-6 rounded-lg shadow text-center">
                                <h2 className="text-2xl font-semibold mb-4">棋盘</h2>
                                <ChessBoard />
                                
                                {/* 控制按钮 */}
                                <div className="flex justify-center space-x-4 mt-6">
                                    <button 
                                        onClick={handleStartGame}
                                        className="bg-green-500 hover:bg-green-600 text-white px-6 py-2 rounded-lg"
                                    >
                                        开始游戏
                                    </button>
                                    <button 
                                        onClick={handleResetGame}
                                        className="bg-red-500 hover:bg-red-600 text-white px-6 py-2 rounded-lg"
                                    >
                                        重置游戏
                                    </button>
                                </div>

                                {/* 难度设置 */}
                                <div className="mt-4">
                                    <label className="block text-sm font-medium text-gray-700 mb-2">
                                        AI难度: {difficulty}
                                    </label>
                                    <input
                                        type="range"
                                        min="1"
                                        max="10"
                                        value={difficulty}
                                        onChange={(e) => handleSetDifficulty(parseInt(e.target.value))}
                                        className="w-full"
                                    />
                                </div>
                            </div>
                        </div>

                        {/* 右侧信息面板 */}
                        <div className="space-y-6">
                            {/* 相机视频流 */}
                            <div className="bg-white p-4 rounded-lg shadow">
                                <h3 className="text-lg font-semibold mb-3">实时视频</h3>
                                {cameraImage ? (
                                    <img src={cameraImage} alt="Camera feed" className="camera-feed" />
                                ) : (
                                    <div className="bg-gray-200 h-48 flex items-center justify-center rounded">
                                        <span className="text-gray-500">等待相机数据...</span>
                                    </div>
                                )}
                            </div>

                            {/* 游戏信息 */}
                            <div className="bg-white p-4 rounded-lg shadow">
                                <h3 className="text-lg font-semibold mb-3">游戏信息</h3>
                                {gameStatus ? (
                                    <div className="space-y-2 text-sm">
                                        <div className="flex justify-between">
                                            <span>当前玩家:</span>
                                            <span>{gameStatus.current_player === 'white' ? '白方' : '黑方'}</span>
                                        </div>
                                        <div className="flex justify-between">
                                            <span>移动次数:</span>
                                            <span>{gameStatus.move_count}</span>
                                        </div>
                                        <div className="flex justify-between">
                                            <span>最后移动:</span>
                                            <span>{gameStatus.last_move || '无'}</span>
                                        </div>
                                        <div className="flex justify-between">
                                            <span>将军状态:</span>
                                            <span>{gameStatus.is_check ? '是' : '否'}</span>
                                        </div>
                                        {gameStatus.game_result && (
                                            <div className="flex justify-between">
                                                <span>游戏结果:</span>
                                                <span className="font-semibold">{gameStatus.game_result}</span>
                                            </div>
                                        )}
                                    </div>
                                ) : (
                                    <div className="text-gray-500">等待游戏数据...</div>
                                )}
                            </div>

                            {/* 性能监控 */}
                            <PerformanceMonitor />
                        </div>
                    </div>
                </div>
            );
        }

        // 渲染应用
        ReactDOM.render(<ChessApp />, document.getElementById('chess-app'));
    </script>
</body>
</html>
```

### 移动端监控App (可选扩展)

```javascript
// chess_ui/mobile_app/ChessRobotApp.js (React Native)
import React, { useState, useEffect } from 'react';
import { View, Text, TouchableOpacity, Image, ScrollView, Alert } from 'react-native';
import io from 'socket.io-client';

const ChessRobotApp = () => {
    const [socket, setSocket] = useState(null);
    const [gameStatus, setGameStatus] = useState(null);
    const [cameraImage, setCameraImage] = useState(null);
    const [performance, setPerformance] = useState(null);

    useEffect(() => {
        // 连接到Jetson Nano的Web服务器
        const newSocket = io('http://192.168.1.100:8080');  // 替换为实际IP
        setSocket(newSocket);

        newSocket.on('connect', () => {
            console.log('Connected to chess robot');
        });

        newSocket.on('game_status_update', setGameStatus);
        newSocket.on('camera_frame', (data) => setCameraImage(data.image));
        newSocket.on('performance_update', setPerformance);

        return () => newSocket.close();
    }, []);

    const handleEmergencyStop = () => {
        Alert.alert(
            "紧急停止",
            "确认要紧急停止机器人吗？",
            [
                { text: "取消", style: "cancel" },
                { text: "确认", onPress: () => {
                    fetch('http://192.168.1.100:8080/api/emergency_stop', { method: 'POST' });
                }}
            ]
        );
    };

    return (
        <ScrollView style={{ flex: 1, backgroundColor: '#f5f5f5' }}>
            {/* 顶部状态栏 */}
            <View style={{ padding: 20, backgroundColor: '#2563eb' }}>
                <Text style={{ color: 'white', fontSize: 24, fontWeight: 'bold' }}>
                    象棋机器人监控
                </Text>
                {gameStatus && (
                    <Text style={{ color: 'white', marginTop: 5 }}>
                        状态: {gameStatus.state} | 移动: {gameStatus.move_count}
                    </Text>
                )}
            </View>

            {/* 相机视频 */}
            <View style={{ margin: 20 }}>
                <Text style={{ fontSize: 18, fontWeight: 'bold', marginBottom: 10 }}>
                    实时视频
                </Text>
                {cameraImage ? (
                    <Image 
                        source={{ uri: cameraImage }} 
                        style={{ width: '100%', height: 200, borderRadius: 10 }}
                    />
                ) : (
                    <View style={{ 
                        width: '100%', height: 200, backgroundColor: '#e5e5e5', 
                        justifyContent: 'center', alignItems: 'center', borderRadius: 10 
                    }}>
                        <Text>等待相机数据...</Text>
                    </View>
                )}
            </View>

            {/* 性能监控 */}
            {performance && (
                <View style={{ margin: 20, backgroundColor: 'white', padding: 15, borderRadius: 10 }}>
                    <Text style={{ fontSize: 18, fontWeight: 'bold', marginBottom: 10 }}>
                        系统性能
                    </Text>
                    <Text>CPU: {performance.cpu_usage.toFixed(1)}%</Text>
                    <Text>内存: {performance.memory_usage.toFixed(1)}%</Text>
                    <Text>视觉处理: {performance.vision_fps.toFixed(1)} FPS</Text>
                </View>
            )}

            {/* 控制按钮 */}
            <View style={{ margin: 20 }}>
                <TouchableOpacity 
                    style={{ 
                        backgroundColor: '#dc2626', padding: 15, borderRadius: 10, alignItems: 'center' 
                    }}
                    onPress={handleEmergencyStop}
                >
                    <Text style={{ color: 'white', fontSize: 18, fontWeight: 'bold' }}>
                        紧急停止
                    </Text>
                </TouchableOpacity>
            </View>
        </ScrollView>
    );
};

export default ChessRobotApp;
```

### 部署配置

**Docker容器化部署 (可选)**

```dockerfile
# chess_ui/Dockerfile
FROM node:16-alpine

# 安装Python和Flask依赖
RUN apk add --no-cache python3 py3-pip

# 设置工作目录
WORKDIR /app

# 复制Python依赖文件
COPY requirements.txt .
RUN pip3 install -r requirements.txt

# 复制应用代码
COPY . .

# 暴露端口
EXPOSE 8080

# 启动命令
CMD ["python3", "web_bridge.py"]
```

```yaml
# docker-compose.yml
version: '3.8'
services:
  chess-web-ui:
    build: ./chess_ui
    ports:
      - "8080:8080"
    volumes:
      - /tmp/.X11-unix:/tmp/.X11-unix:rw
    environment:
      - DISPLAY=${DISPLAY}
      - ROS_DOMAIN_ID=42
    network_mode: host
    depends_on:
      - chess-robot-core
```

### 网络安全配置

```python
# chess_ui/chess_ui/security.py
from flask_limiter import Limiter
from flask_limiter.util import get_remote_address
import hashlib
import jwt
import datetime

class SecurityManager:
    def __init__(self, app):
        self.app = app
        self.limiter = Limiter(
            app,
            key_func=get_remote_address,
            default_limits=["200 per day", "50 per hour"]
        )
        
        # API密钥认证
        self.api_keys = {
            'admin': 'chess_robot_admin_key_2025',
            'viewer': 'chess_robot_viewer_key_2025'
        }
    
    def require_auth(self, permission_level='viewer'):
        """API认证装饰器"""
        def decorator(f):
            def wrapper(*args, **kwargs):
                auth_header = request.headers.get('Authorization')
                if not auth_header or not self.validate_api_key(auth_header, permission_level):
                    return jsonify({'error': 'Unauthorized'}), 401
                return f(*args, **kwargs)
            return wrapper
        return decorator
    
    def validate_api_key(self, auth_header, required_level):
        """验证API密钥"""
        try:
            key = auth_header.replace('Bearer ', '')
            return key in self.api_keys.values()
        except:
            return False
```

## 启动配置

### 完整启动文件（包含Web界面）

```python
# launch/chess_robot_full_web.launch.py
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription, ExecuteProcess

def generate_launch_description():
    return LaunchDescription([
        # 1. 相机节点
        Node(
            package='chess_camera',
            executable='camera_node',
            name='camera_node',
            parameters=[{'frame_rate': 10.0, 'auto_exposure': True}]
        ),
        
        # 2. 视觉识别节点
        Node(
            package='chess_vision',
            executable='board_detector',
            name='board_detector',
            parameters=[{'aruco_dict': 'DICT_4X4_50', 'board_size': 8}]
        ),
        
        Node(
            package='chess_vision',
            executable='piece_classifier',
            name='piece_classifier',
            parameters=[{'model_path': '/home/chess/models/piece_classifier.trt'}]
        ),
        
        # 3. 游戏引擎
        Node(
            package='chess_game',
            executable='game_engine',
            name='game_engine',
            parameters=[{
                'engine_path': '/usr/local/bin/stockfish',
                'default_depth': 15,
                'thinking_time': 5.0
            }]
        ),
        
        # 4. MoveIt2机械臂控制
        IncludeLaunchDescription(
            'moveit_config/launch/dofbot_pro_moveit.launch.py'
        ),
        
        Node(
            package='chess_arm',
            executable='arm_controller',
            name='arm_controller',
            parameters=[{'speed_factor': 0.3, 'safety_mode': True}]
        ),
        
        # 5. 主协调器
        Node(
            package='chess_coordinator',
            executable='game_coordinator',
            name='game_coordinator',
            parameters=[{'auto_start': False, 'debug_mode': True}]
        ),
        
        # 6. 性能监控
        Node(
            package='chess_coordinator',
            executable='performance_monitor',
            name='performance_monitor',
            parameters=[{'monitor_interval': 1.0}]
        ),
        
        # 7. Web桥接节点 - 核心Web服务
        Node(
            package='chess_ui',
            executable='web_bridge',
            name='web_bridge',
            parameters=[{
                'web_port': 8080,
                'web_host': '0.0.0.0',
                'camera_stream_fps': 5.0,
                'enable_cors': True
            }]
        ),
        
        # 8. Robot State Publisher
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[{'robot_description': '/path/to/dofbot_pro.urdf'}]
        ),
        
        # 9. 可选：启动浏览器（仅开发模式）
        ExecuteProcess(
            cmd=['python3', '-c', '''
import time
import webbrowser
time.sleep(10)  # 等待服务器启动
webbrowser.open("http://localhost:8080")
            '''],
            name='open_browser'
        ),
    ])
```

### 快速启动脚本

```bash
#!/bin/bash
# chess_robot_web_startup.sh

echo "启动象棋机器人Web系统..."

# 检查网络连接
if ! ping -c 1 google.com &> /dev/null; then
    echo "警告: 网络连接不可用，某些功能可能受限"
fi

# 设置环境变量
export ROS_DOMAIN_ID=42
export DISPLAY=:0
export CHESS_ROBOT_CONFIG_PATH=/home/chess/config

# 设置ROS2环境
source /opt/ros/humble/setup.bash
source ~/chess_robot_ws/install/setup.bash

# 创建日志目录
mkdir -p ~/chess_logs/$(date +%Y-%m-%d)

# 启动完整系统（包含Web界面）
echo "启动ROS2节点..."
ros2 launch chess_robot chess_robot_full_web.launch.py > ~/chess_logs/$(date +%Y-%m-%d)/system.log 2>&1 &

# 等待系统启动
echo "等待系统启动..."
sleep 15

# 检查关键节点状态
echo "检查系统状态..."
ros2 node list | grep -E "(camera_node|game_coordinator|web_bridge)" > /dev/null
if [ $? -eq 0 ]; then
    echo "✅ 核心节点启动成功"
else
    echo "❌ 核心节点启动失败，检查日志文件"
    exit 1
fi

# 检查Web服务
sleep 5
curl -s http://localhost:8080/api/game_status > /dev/null
if [ $? -eq 0 ]; then
    echo "✅ Web服务启动成功"
    echo "🌐 访问地址: http://$(hostname -I | awk '{print $1}'):8080"
else
    echo "❌ Web服务启动失败"
fi

echo "象棋机器人系统启动完成！"
echo ""
echo "访问方式："
echo "  本地访问: http://localhost:8080"
echo "  局域网访问: http://$(hostname -I | awk '{print $1}'):8080"
echo "  移动端: 下载Chess Robot App并连接到上述地址"
echo ""
echo "停止系统: Ctrl+C 或运行 'pkill -f chess_robot'"
```

## 调试和测试

### 单元测试框架

```python
# test/test_board_detection.py
import unittest
import rclpy
from chess_vision.board_detector import BoardDetector

class TestBoardDetection(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.node = BoardDetector()
    
    def test_aruco_detection(self):
        """测试ArUco标记检测"""
        # 加载测试图像
        test_image = cv2.imread('test_data/test_board.jpg')
        
        # 检测标记
        corners = self.node.detect_aruco_markers(test_image)
        
        # 验证结果
        self.assertEqual(len(corners), 4)
    
    def test_square_extraction(self):
        """测试格子位置提取"""
        # 测试棋盘变换
        pass
    
    @classmethod
    def tearDownClass(cls):
        cls.node.destroy_node()
        rclpy.shutdown()
```

### 性能监控

```python
# chess_coordinator/chess_coordinator/performance_monitor.py
class PerformanceMonitor(Node):
    def __init__(self):
        super().__init__('performance_monitor')
        
        # 性能指标发布器
        self.perf_pub = self.create_publisher(PerformanceMetrics, 'performance_metrics', 10)
        
        # 定时器
        self.timer = self.create_timer(1.0, self.publish_metrics)
        
        self.metrics = {
            'vision_fps': 0.0,
            'planning_time': 0.0,
            'execution_time': 0.0,
            'cpu_usage': 0.0,
            'memory_usage': 0.0,
            'gpu_usage': 0.0
        }
    
    def publish_metrics(self):
        """发布性能指标"""
        msg = PerformanceMetrics()
        msg.vision_fps = self.get_vision_fps()
        msg.cpu_usage = self.get_cpu_usage()
        msg.memory_usage = self.get_memory_usage()
        msg.gpu_usage = self.get_gpu_usage()
        
        self.perf_pub.publish(msg)
```

## 关键技术指标

### 性能目标
- **视觉识别频率**: 10Hz
- **棋子识别准确率**: ≥95%
- **机械臂定位精度**: ≤2mm
- **单步操作时间**: ≤30秒
- **端到端延迟**: ≤5秒

### 系统资源
- **CPU使用率**: ≤70%
- **内存使用率**: ≤80%
- **GPU使用率**: ≤90%
- **网络延迟**: ≤10ms (ROS2内部)

## 部署和运维

### 自动启动脚本

```bash
#!/bin/bash
# chess_robot_startup.sh

# 等待系统完全启动
sleep 10

# 设置ROS2环境
source /opt/ros/humble/setup.bash
source ~/chess_robot_ws/install/setup.bash

# 启动系统
ros2 launch chess_robot chess_robot_full.launch.py

# 启动Web界面
cd ~/chess_robot_ws/src/chess_ui
python3 app.py &

# 启动系统监控
ros2 run chess_coordinator performance_monitor &
```

### 系统服务配置

```ini
# /etc/systemd/system/chess-robot.service
[Unit]
Description=Chess Robot ROS2 System
After=network.target

[Service]
Type=simple
User=chess
WorkingDirectory=/home/chess
ExecStart=/home/chess/chess_robot_startup.sh
Restart=always
RestartSec=10

[Install]
WantedBy=multi-user.target
```

## 项目里程碑与验收

| 阶段 | 时间 | 关键交付物 | 验收标准 |
|------|------|------------|----------|
| ROS2环境 | Week 1-2 | 工作空间+基础包 | 编译成功，通信正常 |
| 相机集成 | Week 3-4 | 相机驱动+标定 | 稳定的RGB+深度数据流 |
| 视觉系统 | Week 5-8 | 检测+识别算法 | 识别准确率>90% |
| MoveIt2集成 | Week 9-11 | 运动规划+控制 | 精确路径执行 |
| 游戏引擎 | Week 12-13 | 逻辑+AI引擎 | 完整象棋对弈功能 |
| 系统协调 | Week 14-15 | 状态机+协调器 | 端到端自动对弈 |
| 完善优化 | Week 16 | UI+优化+文档 | 可演示的完整系统 |

## Web前端技术特性

### 实时通信架构
- **WebSocket连接**：使用Socket.IO实现ROS2与Web前端的实时双向通信
- **数据流优化**：相机图像压缩传输，性能数据实时推送
- **自动重连**：网络断开时自动重连，保证系统稳定性

### 响应式界面设计
- **多设备适配**：支持桌面、平板、手机等不同屏幕尺寸
- **实时棋盘显示**：3D棋盘渲染，实时反映物理棋盘状态
- **视觉反馈**：移动高亮、状态指示、动画效果

### 高级功能
- **远程控制**：通过Web界面远程启动、停止、重置游戏
- **系统监控**：实时显示CPU、内存、GPU使用率和处理性能
- **日志查看**：Web界面查看系统日志和错误信息
- **设置调整**：在线调整AI难度、相机参数、机械臂速度等

### 安全性保障
- **API认证**：基于密钥的API访问控制
- **访问限制**：IP白名单和访问频率限制
- **安全传输**：支持HTTPS加密传输（生产环境）

## 项目里程碑与验收（更新）

| 阶段 | 时间 | 关键交付物 | 验收标准 |
|------|------|------------|----------|
| ROS2环境 | Week 1-2 | 工作空间+基础包 | 编译成功，通信正常 |
| 相机集成 | Week 3-4 | 相机驱动+标定 | 稳定的RGB+深度数据流 |
| 视觉系统 | Week 5-8 | 检测+识别算法 | 识别准确率>90% |
| MoveIt2集成 | Week 9-11 | 运动规划+控制 | 精确路径执行 |
| 游戏引擎 | Week 12-13 | 逻辑+AI引擎 | 完整象棋对弈功能 |
| 系统协调 | Week 14-15 | 状态机+协调器 | 端到端自动对弈 |
| **Web前端** | **Week 16-17** | **Web界面+移动端** | **远程控制和监控** |

## Web前端开发要点

### 技术栈选择理由
1. **React + Socket.IO**：
   - 实时数据绑定，UI自动更新
   - 组件化开发，代码复用性高
   - 丰富的生态系统和社区支持

2. **Flask-SocketIO后端**：
   - 与ROS2完美集成
   - 轻量级，资源占用少
   - 支持异步处理，不阻塞机器人控制

3. **TailwindCSS样式**：
   - 快速原型开发
   - 响应式设计
   - 现代化UI组件

### 关键实现细节

**实时数据同步**
```python
# 关键：ROS2数据到Web的实时推送
def board_state_callback(self, msg):
    # ROS2回调 -> WebSocket推送
    self.socketio.emit('board_update', board_data)
```

**相机视频流**
```python
# 图像压缩和Base64编码，降低网络负载
_, buffer = cv2.imencode('.jpg', cv_image, [cv2.IMWRITE_JPEG_QUALITY, 80])
img_base64 = base64.b64encode(buffer).decode('utf-8')
```

**移动端优化**
- 图像分辨率自适应
- 触摸操作优化
- 离线状态处理

## 部署和访问

### 本地访问
```
http://localhost:8080
```

### 局域网访问
```
http://[Jetson-Nano-IP]:8080
```

### 移动端App
- 开发React Native应用
- 或使用浏览器直接访问Web版本
- 支持紧急停止等安全功能

### ROS2优势
1. **更好的实时性**：DDS中间件提供确定性通信
2. **更强的安全性**：内置安全框架
3. **跨平台支持**：Windows、macOS、嵌入式系统
4. **更好的多机器人支持**：原生分布式架构
5. **现代化工具**：colcon构建系统、launch文件Python API
6. **活跃的社区**：持续更新和长期支持

### 关键改进
- **节点生命周期管理**：更可靠的节点启动和关闭
- **参数系统**：动态参数重配置
- **QoS配置**：灵活的通信质量控制
- **组合节点**：单进程多节点，降低资源消耗

## 总结

基于ROS2的象棋机器人项目具有以下特点：

1. **现代化架构**：采用ROS2分布式实时系统，提供更好的可靠性和扩展性
2. **模块化设计**：每个功能模块独立开发和测试，便于维护和升级
3. **标准化接口**：使用MoveIt2和标准ROS2消息，便于集成其他组件
4. **完整的生态**：从底层驱动到上层应用的完整解决方案
5. **优秀的可观测性**：内置性能监控和调试工具
6. **生产级质量**：包含完整的测试、部署和运维方案

## Gazebo仿真环境补充

### 仿真环境的重要性

在机器人开发中，仿真环境是必不可少的组成部分，特别是对于象棋机器人这种需要精确控制的项目。仿真环境的优势包括：

1. **降低硬件风险**：在真实硬件上直接测试可能导致机械臂碰撞或损坏
2. **加速开发进程**：并行开发，无需等待硬件到位
3. **算法验证**：快速验证运动规划和视觉算法
4. **批量测试**：自动化测试不同场景和边界条件
5. **成本节约**：减少硬件损耗和维护成本

### 扩展的ROS2包结构（包含仿真）

```
chess_robot_ws/
├── src/
│   ├── chess_interfaces/         # 自定义消息和服务
│   ├── chess_camera/            # 相机驱动和数据发布
│   ├── chess_vision/            # 视觉识别和AI推理
│   ├── chess_game/              # 游戏逻辑和引擎
│   ├── chess_arm/               # 机械臂控制
│   ├── chess_planner/           # MoveIt2运动规划
│   ├── chess_coordinator/       # 系统协调和状态机
│   ├── chess_ui/                # 用户界面和监控
│   ├── chess_simulation/        # 仿真环境包（新增）
│   │   ├── worlds/              # Gazebo世界文件
│   │   ├── models/              # 3D模型文件
│   │   ├── plugins/             # Gazebo插件
│   │   ├── launch/              # 仿真启动文件
│   │   └── config/              # 仿真配置文件
│   └── chess_description/       # 机器人描述文件（新增）
│       ├── urdf/                # URDF和Xacro文件
│       ├── meshes/              # 3D网格文件
│       └── config/              # 机器人配置
```

### 仿真核心组件

#### 1. 象棋世界模型 (chess_simulation/worlds/chess_world.world)

```xml
<?xml version="1.0" ?>
<sdf version="1.6">
  <world name="chess_world">
    <!-- 光照设置 -->
    <include>
      <uri>model://sun</uri>
    </include>

    <!-- 地面 -->
    <include>
      <uri>model://ground_plane</uri>
    </include>

    <!-- 象棋桌 -->
    <model name="chess_table">
      <static>true</static>
      <link name="table_link">
        <visual name="table_visual">
          <geometry>
            <box>
              <size>1.2 0.8 0.75</size>
            </box>
          </geometry>
          <material>
            <ambient>0.8 0.6 0.4 1</ambient>
            <diffuse>0.8 0.6 0.4 1</diffuse>
          </material>
        </visual>
        <collision name="table_collision">
          <geometry>
            <box>
              <size>1.2 0.8 0.75</size>
            </box>
          </geometry>
        </collision>
      </link>
      <pose>0 0 0.375 0 0 0</pose>
    </model>

    <!-- 象棋棋盘 -->
    <model name="chess_board">
      <static>true</static>
      <link name="board_link">
        <visual name="board_visual">
          <geometry>
            <box>
              <size>0.64 0.64 0.01</size>
            </box>
          </geometry>
          <material>
            <script>
              <uri>file://chess_board_texture</uri>
              <name>ChessBoard</name>
            </script>
          </material>
        </visual>
        <collision name="board_collision">
          <geometry>
            <box>
              <size>0.64 0.64 0.01</size>
            </box>
          </geometry>
        </collision>
      </link>
      <pose>0 0 0.755 0 0 0</pose>
    </model>

    <!-- 相机插件 -->
    <model name="chess_camera">
      <static>true</static>
      <pose>0.5 0 1.2 0 0.5 3.14159</pose>
      <link name="camera_link">
        <sensor name="camera" type="camera">
          <camera>
            <horizontal_fov>1.0472</horizontal_fov>
            <image>
              <width>1920</width>
              <height>1080</height>
              <format>R8G8B8</format>
            </image>
            <clip>
              <near>0.1</near>
              <far>100</far>
            </clip>
          </camera>
          <plugin name="camera_controller" filename="libgazebo_ros_camera.so">
            <ros>
              <namespace>chess_simulation</namespace>
              <argument>~/out:=image_raw</argument>
            </ros>
          </plugin>
        </sensor>
      </link>
    </model>
  </world>
</sdf>
```

#### 2. 仿真启动文件 (chess_simulation/launch/chess_simulation.launch.py)

```python
import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, ExecuteProcess, SetEnvironmentVariable
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    chess_simulation_path = get_package_share_directory('chess_simulation')
    chess_description_path = get_package_share_directory('chess_description')

    return LaunchDescription([
        # 设置环境变量
        SetEnvironmentVariable('GAZEBO_MODEL_PATH',
                             os.path.join(chess_simulation_path, 'models')),

        # 启动Gazebo服务器
        ExecuteProcess(
            cmd=['gzserver', '--verbose',
                 os.path.join(chess_simulation_path, 'worlds', 'chess_world.world')],
            output='screen'
        ),

        # 启动Gazebo客户端
        ExecuteProcess(
            cmd=['gzclient'],
            output='screen'
        ),

        # 发布机器人状态
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[{
                'robot_description': open(os.path.join(
                    chess_description_path, 'urdf', 'dofbot_pro.urdf')).read()
            }]
        ),

        # 在Gazebo中生成机器人
        Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            arguments=['-topic', '/robot_description',
                      '-entity', 'dofbot_pro',
                      '-x', '0.6', '-y', '0.0', '-z', '0.76'],
            output='screen'
        ),

        # 仿真相机节点
        Node(
            package='chess_simulation',
            executable='sim_camera_node',
            name='sim_camera_node',
            parameters=[{
                'camera_topic': '/chess_simulation/image_raw',
                'depth_topic': '/chess_simulation/depth_image_raw'
            }]
        ),
    ])
```

#### 3. 环境适配器 (chess_coordinator/chess_coordinator/environment_adapter.py)

```python
import rclpy
from rclpy.node import Node
from chess_interfaces.msg import BoardState, GameStatus

class EnvironmentAdapter(Node):
    """环境适配器：统一仿真和真实环境的接口"""

    def __init__(self):
        super().__init__('environment_adapter')

        # 参数：运行模式
        self.declare_parameter('simulation_mode', False)
        self.simulation_mode = self.get_parameter('simulation_mode').value

        # 发布器（统一接口）
        self.board_pub = self.create_publisher(BoardState, 'board_state', 10)
        self.status_pub = self.create_publisher(GameStatus, 'game_status', 10)

        if self.simulation_mode:
            self.setup_simulation_interface()
        else:
            self.setup_hardware_interface()

        self.get_logger().info(f"环境适配器启动 - 模式: {'仿真' if self.simulation_mode else '硬件'}")

    def setup_simulation_interface(self):
        """设置仿真环境接口"""
        # 订阅仿真数据
        self.sim_board_sub = self.create_subscription(
            BoardState, 'sim_board_state', self.sim_board_callback, 10)

    def setup_hardware_interface(self):
        """设置硬件环境接口"""
        # 订阅真实硬件数据
        self.hw_board_sub = self.create_subscription(
            BoardState, 'hw_board_state', self.hw_board_callback, 10)

    def sim_board_callback(self, msg):
        """仿真棋盘状态回调"""
        # 转发到统一接口
        self.board_pub.publish(msg)

    def hw_board_callback(self, msg):
        """硬件棋盘状态回调"""
        # 直接转发
        self.board_pub.publish(msg)
```

### 核心仿真命令

```bash
# 启动完整仿真环境
ros2 launch chess_simulation chess_simulation.launch.py

# 启动仿真模式的完整系统
ros2 launch chess_robot chess_robot_full.launch.py simulation:=true

# 切换仿真/硬件模式
ros2 param set /environment_adapter simulation_mode true

# 仿真移动命令
ros2 service call /simulate_move chess_interfaces/srv/SimulateMove \
  "{chess_move: {from_square: 52, to_square: 36}}"
```

### 更新后的项目时间线（18周）

| 阶段 | 周数 | 任务 | 交付物 | 验收标准 |
|------|------|------|--------|----------|
| **第一阶段** | Week 1-2 | ROS2环境+仿真基础 | 工作空间+Gazebo环境 | 仿真环境运行正常 |
| **第二阶段** | Week 3-4 | 相机集成+仿真相机 | 双模式相机系统 | 真实+仿真数据流 |
| **第三阶段** | Week 5-8 | 视觉系统+仿真适配 | 检测+识别算法 | 仿真环境95%准确率 |
| **第四阶段** | Week 9-11 | MoveIt2+仿真控制 | 运动规划+仿真执行 | 仿真环境精确控制 |
| **第五阶段** | Week 12-13 | 游戏引擎+测试框架 | 完整游戏逻辑 | 仿真环境完整对弈 |
| **第六阶段** | Week 14-15 | 系统协调+双模式 | 环境适配器 | 仿真/硬件无缝切换 |
| **第七阶段** | Week 16-17 | Web界面+移动端 | 完整用户界面 | 远程监控和控制 |
| **第八阶段** | Week 18 | 集成测试+优化 | 最终系统 | 硬件环境验证 |

## 总结

基于ROS2的象棋机器人项目现在具有以下增强特点：

1. **完整的仿真环境**：Gazebo物理仿真，降低开发风险
2. **双模式架构**：仿真和硬件环境无缝切换
3. **自动化测试**：完整的测试框架和性能基准
4. **Web监控界面**：实时远程监控和控制
5. **现代化架构**：ROS2分布式实时系统
6. **模块化设计**：独立开发和测试的功能模块
7. **标准化接口**：MoveIt2和标准ROS2消息
8. **生产级质量**：包含完整的测试、部署和运维方案

这个18周的实施计划通过引入仿真环境，大大提高了开发效率和系统可靠性，为构建一个功能完整、性能优异的象棋机器人系统提供了更加完善的技术路线图。