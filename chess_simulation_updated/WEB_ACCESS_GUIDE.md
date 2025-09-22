# Gazebo象棋机器人 Web访问指南

## 🌐 Web访问概述

本系统提供完整的Web界面访问支持，允许通过浏览器远程监控和控制Gazebo仿真环境。

### 📋 访问地址

- **主机IP**: `192.168.1.15`
- **Web监控界面**: `http://192.168.1.15:3000`
- **Gazebo仿真界面**: `http://192.168.1.15:8080`
- **WebSocket端点**: `ws://192.168.1.15:9002`

## 🚀 快速启动

### 方式一：一键启动所有服务

```bash
# 进入Web服务目录
cd /tmp/gazebo_web

# 启动所有Web服务（Web服务器 + WebSocket桥接）
./start_web_services.sh
```

### 方式二：分别启动各个服务

```bash
# 1. 启动Web服务器
cd /tmp/gazebo_web
python3 simple_server.py

# 2. 启动WebSocket桥接（新终端）
cd /home/jetson/chess_robot_ws/src/chess_simulation
python3 scripts/websocket_bridge.py

# 3. 启动Gazebo仿真（新终端）
cd /home/jetson/chess_robot_ws/src/chess_simulation/scripts
./start_web_gazebo.sh
```

## 🎮 Web界面功能

### 主要功能

1. **实时3D仿真显示**
   - Gazebo 3D场景嵌入式显示
   - 实时渲染和交互

2. **系统状态监控**
   - WebSocket连接状态
   - Gazebo仿真状态
   - 机器人硬件状态
   - 游戏引擎状态

3. **远程控制**
   - 开始/暂停游戏
   - 重置游戏
   - 紧急停止功能

4. **实时数据显示**
   - 棋盘状态可视化
   - 机械臂关节状态
   - 系统日志实时更新

5. **数据流监控**
   - ROS话题数据实时显示
   - 象棋移动历史
   - 性能监控指标

## 🔧 系统架构

```
┌─────────────────┐    ┌─────────────────┐    ┌─────────────────┐
│   Web Browser   │    │   Web Server    │    │   ROS2 System   │
│                 │◄──►│   (Port 3000)   │◄──►│                 │
│  - 监控界面     │    │  - Static Files │    │  - Gazebo       │
│  - 实时控制     │    │  - CORS Support │    │  - Nodes        │
└─────────────────┘    └─────────────────┘    │  - Topics       │
         ▲                                     └─────────────────┘
         │ WebSocket                                   ▲
         ▼                                             │
┌─────────────────┐    ┌─────────────────┐            │
│  WebSocket      │    │  Gazebo Web     │◄───────────┘
│  Client         │◄──►│  GUI Server     │
│  (Port 9002)    │    │  (Port 8080)    │
└─────────────────┘    └─────────────────┘
```

## 📱 支持的设备

### 桌面浏览器
- **Chrome/Edge**: 完全支持，推荐使用
- **Firefox**: 完全支持
- **Safari**: 基本支持

### 移动设备
- **Android Chrome**: 支持监控功能
- **iOS Safari**: 支持监控功能
- **平板设备**: 完全支持，界面自适应

## 🛠️ 配置文件

### Web服务器配置
- 文件位置: `/home/jetson/chess_robot_ws/src/chess_simulation/config/gazebo_web.yaml`
- 主要配置:
  ```yaml
  web_server:
    host: "0.0.0.0"
    port: 8080
  websocket:
    port: 9002
    allowed_origins: ["*"]
  ```

### 网络配置
- 防火墙状态: 未启用（简化配置）
- 开放端口: 3000 (Web), 8080 (Gazebo), 9002 (WebSocket)
- 跨域访问: 已启用CORS支持

## 🔍 故障排除

### 常见问题

#### 1. 无法访问Web界面
```bash
# 检查服务是否运行
netstat -tuln | grep -E ":(3000|8080|9002) "

# 检查IP地址
hostname -I

# 重启Web服务
cd /tmp/gazebo_web && ./start_web_services.sh
```

#### 2. WebSocket连接失败
```bash
# 检查WebSocket服务
ps aux | grep websocket_bridge

# 重启WebSocket桥接
cd /home/jetson/chess_robot_ws/src/chess_simulation
python3 scripts/websocket_bridge.py
```

#### 3. Gazebo无法显示
```bash
# 检查Gazebo进程
ps aux | grep gazebo

# 重启Gazebo仿真
cd /home/jetson/chess_robot_ws/src/chess_simulation/scripts
./start_web_gazebo.sh
```

### 日志查看

```bash
# Web服务器日志
tail -f /tmp/gazebo_web_logs/web_server.log

# ROS2节点日志
ros2 node list
ros2 topic echo /chess/system_status
```

## 🔒 安全注意事项

1. **网络访问**
   - 当前配置允许局域网内所有设备访问
   - 生产环境建议配置防火墙规则

2. **端口暴露**
   - 仅在可信网络中使用
   - 考虑使用VPN进行远程访问

3. **数据传输**
   - WebSocket连接未加密
   - 敏感数据不应通过Web界面传输

## 📈 性能优化

### 推荐设置

1. **帧率控制**
   - Gazebo渲染: 30 FPS
   - WebSocket更新: 10 Hz
   - 图像传输: 5 Hz

2. **网络优化**
   - 启用数据压缩
   - 限制并发连接数
   - 使用缓存策略

3. **硬件要求**
   - CPU: 多核处理器（推荐4核以上）
   - 内存: 4GB以上可用内存
   - 网络: 100Mbps以上带宽

## 🚀 高级功能

### 扩展开发

1. **自定义插件**
   - 位置: `/home/jetson/chess_robot_ws/src/chess_simulation/web/plugins/`
   - 支持JavaScript插件开发

2. **API接口**
   - RESTful API端点
   - 自定义数据格式支持

3. **多用户支持**
   - 会话管理
   - 权限控制

## 📞 技术支持

如遇到问题，请检查：

1. 网络连接状态
2. 服务运行状态
3. 端口占用情况
4. 系统资源使用情况

更多技术细节请参考项目文档或联系技术支持团队。