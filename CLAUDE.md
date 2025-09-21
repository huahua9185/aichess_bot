# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## 项目概述

这是一个基于ROS2的智能象棋机器人系统，实现全自动象棋对弈功能。系统运行在Jetson Nano平台上，集成DOFBOT Pro 6DOF机械臂、DABAI DCW2深度相机，通过MoveIt2进行运动规划。

## 核心技术栈

- **ROS2 Humble** - 主要通信框架
- **Gazebo** - 物理仿真环境
- **MoveIt2** - 机械臂运动规划
- **OpenCV + AI模型** - 计算机视觉
- **Stockfish** - 象棋引擎
- **Flask-SocketIO** - Web界面后端
- **React + TailwindCSS** - Web前端

## 常用命令

### 构建系统
```bash
# 创建ROS2工作空间
mkdir -p ~/chess_robot_ws/src
cd ~/chess_robot_ws

# 构建所有包
colcon build

# 设置环境
source install/setup.bash
```

### 运行系统
```bash
# 启动仿真环境（开发模式）
ros2 launch chess_simulation chess_simulation.launch.py

# 启动完整系统（包含Web界面）
ros2 launch chess_robot chess_robot_full_web.launch.py

# 启动核心机器人系统
ros2 launch chess_robot chess_robot_full.launch.py

# 启动单个节点进行测试
ros2 run chess_camera camera_node
ros2 run chess_vision board_detector
ros2 run chess_game game_engine
```

### 仿真环境
```bash
# 启动Gazebo仿真
ros2 launch chess_simulation gazebo_world.launch.py

# 启动仿真模式的完整系统
ros2 launch chess_simulation chess_sim_full.launch.py

# 加载测试用例
ros2 run chess_testing run_test_scenario.py --scenario basic_game

# 录制和回放仿真数据
ros2 bag record /chess/board_state /chess/moves
ros2 bag play chess_game_data.bag
```

### 测试和调试
```bash
# 运行单元测试
python3 -m pytest test/

# 检查节点状态
ros2 node list
ros2 topic list

# 监控性能
ros2 topic echo /performance_metrics

# 查看相机数据
ros2 topic echo /camera/rgb/image_raw
```

### Web界面
```bash
# 启动Web服务器
cd chess_ui
python3 web_bridge.py

# 访问地址
# 本地: http://localhost:8080
# 局域网: http://[JETSON_IP]:8080
```

## 系统架构

### ROS2包结构
```
chess_robot_ws/src/
├── chess_interfaces/     # 自定义消息和服务定义
├── chess_camera/        # DABAI DCW2相机驱动
├── chess_vision/        # 棋盘检测和棋子识别
├── chess_game/          # 象棋逻辑和Stockfish引擎集成
├── chess_arm/           # DOFBOT Pro机械臂控制
├── chess_planner/       # MoveIt2运动规划配置
├── chess_coordinator/   # 系统状态机和协调器
├── chess_ui/           # Web界面和移动端监控
├── chess_simulation/    # Gazebo仿真环境
├── chess_gazebo_plugins/ # 自定义Gazebo插件
└── chess_testing/       # 自动化测试和基准测试
```

### 核心节点通信
```
camera_node → vision_node → game_node → arm_node
     ↓            ↓           ↓          ↓
  RGB+深度图   棋盘状态    AI决策    机械臂控制
```

### 关键接口定义

**BoardState.msg** - 64格棋盘状态，包含棋子位置和3D坐标
**ChessMove.msg** - 象棋移动指令，包含起止位置和特殊规则
**DetectBoard.srv** - 棋盘检测服务
**PlanMove.srv** - 运动规划服务
**ExecuteMove.srv** - 移动执行服务

## 硬件集成要点

### 相机标定
- 使用9x6棋盘格进行相机标定
- 建立camera_link到base_link的TF变换
- ArUco标记用于棋盘定位

### 机械臂配置
- URDF模型路径: `config/dofbot_pro.urdf.xacro`
- MoveIt配置: `config/dofbot_pro_moveit.yaml`
- 安全限制: 速度系数0.3，避碰检测开启

### 坐标系关系
```
base_link → camera_link → board_frame → square_positions[64]
```

## 性能目标

- 视觉识别频率: 10Hz
- 棋子识别准确率: ≥95%
- 机械臂定位精度: ≤2mm
- 单步操作时间: ≤30秒
- CPU使用率: ≤70%
- 内存使用率: ≤80%

## 仿真环境

### Gazebo仿真优势
- **安全开发** - 避免损坏昂贵的机械臂和传感器
- **快速迭代** - 算法测试周期从小时缩短到分钟
- **并行开发** - 多人同时在不同模块工作
- **全面测试** - 涵盖各种场景和边缘情况

### 仿真组件
- **DOFBOT Pro精确模型** - 包含关节限制和动力学参数
- **虚拟深度相机** - 模拟DABAI DCW2的RGB+深度数据
- **物理棋盘和棋子** - 真实的重力、碰撞和摩擦力
- **光照仿真** - 可配置的照明条件和阴影

### 双模式架构
```bash
# 仿真模式 - 开发和测试阶段
export ROBOT_MODE=simulation
ros2 launch chess_simulation chess_sim_full.launch.py

# 真实硬件模式 - 部署阶段
export ROBOT_MODE=hardware
ros2 launch chess_robot chess_robot_full.launch.py
```

### 测试场景
- **基础象棋对弈** - 标准开局和中局测试
- **边缘情况** - 棋子位置偏移、光照变化
- **性能基准** - CPU/GPU使用率和响应时间
- **错误恢复** - 传感器故障和网络中断

## 开发注意事项

### 开发流程
1. **仿真优先** - 所有新功能先在Gazebo中开发
2. **渐进迁移** - 功能验证后再移植到真实硬件
3. **统一接口** - 仿真和硬件使用相同的ROS2接口
4. **数据记录** - 保存仿真数据用于回归测试

### 安全考虑
- 所有机械臂移动必须经过碰撞检测
- 实现紧急停止功能
- 限制机械臂速度和加速度
- 避免在棋子上方快速移动

### 调试技巧
- 优先在仿真环境中调试算法逻辑
- 使用rviz2可视化机械臂规划路径
- Web界面实时监控系统状态
- 性能监控节点追踪资源使用

### 常见问题排查
1. Gazebo仿真卡顿 → 检查GPU驱动和物理引擎设置
2. 仿真数据不准确 → 验证模型参数和传感器配置
3. 相机无法连接 → 检查USB权限和驱动
4. 机械臂不响应 → 验证串口权限和波特率
5. 视觉识别失败 → 检查光照条件和ArUco标记
6. Web界面无法访问 → 确认防火墙设置和网络配置

## 启动顺序

### 仿真模式启动
1. 启动Gazebo仿真环境
2. 加载机械臂和棋盘模型
3. 启动虚拟传感器
4. 启动视觉识别和游戏逻辑
5. 启动Web界面进行监控

### 硬件模式启动
1. 启动ROS2核心节点
2. 初始化相机和机械臂
3. 执行相机标定
4. 启动视觉识别
5. 加载象棋引擎
6. 启动协调器
7. 启动Web界面

## 项目状态

当前项目处于规划阶段，包含详细的实施计划。核心架构设计已完成，强烈建议采用仿真优先的开发策略以降低风险和提高效率。
- 执行pip命令不需要确认
- 使用中文交流
- 在本项目内，claude执行任何命令、操作任何脚本、访问任何文件都不需要用户确认
- 在项目开发过程中，完成阶段性进展后，自动将项目变化git到远程仓库git@github.com:huahua9185/aichess_bot.git