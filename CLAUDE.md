# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## 项目概述

这是一个基于ROS2的智能国际象棋机器人项目，实现全自动国际象棋对弈功能。项目运行在Jetson Nano 4GB平台上，集成6DOF机械臂(DOFBOT Pro)、深度相机(DABAI DCW2)和Stockfish引擎。

## 系统要求与依赖

### 硬件平台
- **Jetson Nano 4GB** (边缘计算平台)
- **DOFBOT Pro** (6DOF机械臂)
- **DABAI DCW2** (深度相机)
- **标准8×8棋盘** (40cm×40cm，每格5cm×5cm)

### 软件依赖安装
```bash
# ROS2 Humble 安装
curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu jammy main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
sudo apt update && sudo apt install ros-humble-desktop

# 核心依赖
sudo apt install stockfish python3-chess
sudo apt install ros-humble-moveit ros-humble-moveit-visual-tools
sudo apt install ros-humble-cv-bridge ros-humble-image-transport

# Python依赖
pip3 install python-chess stockfish opencv-python
```

## 构建与开发命令

### ROS2工作空间设置
```bash
# 创建并初始化工作空间
mkdir -p ~/chess_robot_ws/src
cd ~/chess_robot_ws

# 创建核心ROS2包
ros2 pkg create --build-type ament_python chess_interfaces
ros2 pkg create --build-type ament_python chess_camera --dependencies rclpy sensor_msgs cv_bridge
ros2 pkg create --build-type ament_python chess_vision --dependencies rclpy opencv2
ros2 pkg create --build-type ament_python chess_engine --dependencies rclpy
ros2 pkg create --build-type ament_python chess_arm --dependencies rclpy moveit_msgs
ros2 pkg create --build-type ament_python chess_coordinator --dependencies rclpy
```

### 构建与测试
```bash
# 构建整个工作空间
colcon build

# 设置环境
source install/setup.bash

# 运行单元测试
python3 -m pytest test/test_chess_integration.py -v

# 运行特定测试
python3 -m unittest test.test_chess_integration.TestChessRobotIntegration.test_engine_opening_moves
```

### 启动系统
```bash
# 快速启动完整系统
ros2 launch chess_robot chess_robot_complete.launch.py web_interface:=true

# 启动单个节点进行调试
ros2 run chess_camera camera_node
ros2 run chess_vision vision_node
ros2 run chess_engine stockfish_node
ros2 run chess_arm arm_node

# Web界面访问
echo "访问: http://$(hostname -I | awk '{print $1}'):8080"
```

## 系统架构

### 核心ROS2节点架构
```
┌─────────────────────────────────────────────────────────────┐
│                        ROS2 网络                             │
├─────────────────────────────────────────────────────────────┤
│  camera_node  │  vision_node  │  chess_engine │  arm_node    │
│   (传感器)     │   (视觉AI)    │  (国际象棋)   │  (运动控制)   │
└─────────────────────────────────────────────────────────────┘
```

### 包结构
```
chess_robot_ws/
src/
├── chess_interfaces/         # 消息、服务、动作定义
├── chess_camera/            # 相机驱动和数据发布
├── chess_vision/            # 视觉识别和AI推理
├── chess_engine/            # Stockfish国际象棋引擎
├── chess_arm/               # 机械臂控制
├── chess_planner/           # MoveIt2运动规划
├── chess_coordinator/       # 系统协调和状态机
└── chess_ui/                # Web用户界面
```

### 关键组件说明

#### 1. chess_engine/stockfish_node.py
- **功能**: 集成Stockfish引擎，提供AI对弈能力
- **关键服务**: `get_engine_move`, `validate_move`
- **配置**: 引擎强度、思考时间、ELO等级可调

#### 2. chess_coordinator
- **功能**: 游戏状态机，协调各节点交互
- **状态**: WAITING_FOR_PLAYER → VALIDATING_HUMAN_MOVE → ENGINE_THINKING → EXECUTING_ENGINE_MOVE
- **核心逻辑**: 处理人机轮流对弈流程

#### 3. chess_vision
- **功能**: 棋盘检测、棋子识别、状态分析
- **支持**: 6种棋子类型(王/后/车/象/马/兵) × 2种颜色
- **特殊处理**: 王车易位、吃过路兵、兵升变检测

#### 4. chess_arm + chess_planner
- **功能**: MoveIt2运动规划和机械臂控制
- **避障**: 智能路径规划，避开其他棋子
- **精确性**: 专为国际象棋优化的抓取和放置策略

## 国际象棋特定实现

### 棋子识别系统
- **棋盘规格**: 标准8×8棋盘，每格5cm×5cm
- **识别类型**: King/Queen/Rook/Bishop/Knight/Pawn × White/Black
- **深度学习**: 基于OpenCV和AI模型的实时识别

### 特殊规则处理
- **王车易位**: 自动检测并执行两步移动
- **吃过路兵**: 识别特殊吃兵情况
- **兵升变**: 支持升变为后/车/象/马
- **胜负判定**: 将军/将死/僵局检测

### 引擎集成
- **Stockfish引擎**: 可调节难度等级(1-20)
- **实时分析**: 提供局面评估和最佳变化
- **多难度**: 支持从初学者到大师级别

## Web界面功能

### 实时监控
- **棋盘状态**: 实时显示当前局面
- **移动历史**: 完整的对局记录
- **引擎分析**: 局面评估和最佳变化
- **相机视图**: 实时摄像头画面

### 游戏控制
- **难度调节**: 引擎强度1-20级可调
- **时间控制**: 可设置思考时间限制
- **游戏设置**: 人机对弈配置选项

## 调试与开发提示

### 常用调试命令
```bash
# 查看节点状态
ros2 node list
ros2 topic list
ros2 service list

# 监控消息
ros2 topic echo /board_state
ros2 topic echo /engine_move
ros2 topic echo /camera/image_raw

# 查看服务调用
ros2 service call /get_engine_move chess_interfaces/srv/GetEngineMove "{fen_position: 'rnbqkbnr/pppppppp/8/8/8/8/PPPPPPPP/RNBQKBNR w KQkq - 0 1', depth: 10, time_limit: 5.0}"
```

### 测试数据路径
- **棋子图像**: `test_data/pieces/{color}_{piece}.jpg`
- **棋盘状态**: `test_data/{position_name}.jpg`
- **单元测试**: `test/test_chess_integration.py`

### 配置文件位置
- **启动配置**: `launch/chess_robot_complete.launch.py`
- **节点参数**: `params/` 目录下各yaml文件
- **相机标定**: `config/camera_calibration.yaml`
- **机械臂配置**: `config/arm_parameters.yaml`

## 生产部署

### 系统启动脚本
```bash
#!/bin/bash
# 设置环境
source /opt/ros/humble/setup.bash
source ~/chess_robot_ws/install/setup.bash

# 硬件检查
if ! lsusb | grep -q "DABAI"; then
    echo "❌ 深度相机未检测到"
fi

# 启动完整系统
ros2 launch chess_robot chess_robot_complete.launch.py web_interface:=true
```

### 性能监控
- **CPU使用率**: 视觉处理和AI推理监控
- **内存占用**: Jetson Nano 4GB内存管理
- **温度监控**: 防止过热保护
- **网络延迟**: Web界面响应性能

## 注意事项

- 所有代码使用中文注释和文档
- 遵循ROS2编程规范和最佳实践
- 图像处理针对Jetson Nano进行优化
- 机械臂操作确保安全性和精确性
- 支持离线运行，不依赖外部网络服务