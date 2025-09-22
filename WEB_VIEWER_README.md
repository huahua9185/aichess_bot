# Gazebo象棋机器人Web查看器

## 🌟 功能概述

这是一个基于Flask的Web应用，允许您通过浏览器实时查看Gazebo仿真环境中的象棋机器人系统。

## 🚀 功能特性

- **🎥 实时视频流** - 30fps流畅的仿真画面传输
- **📱 跨平台支持** - 支持桌面、手机、平板浏览器访问
- **🎮 远程控制** - 通过Web界面启动/停止/重置仿真
- **📊 性能监控** - 实时显示FPS、延迟、系统状态
- **🖥️ 响应式界面** - 自适应不同屏幕尺寸
- **🔧 调试工具** - 系统日志查看、截图保存功能

## 📡 访问地址

系统配置完成后，可通过以下地址访问：

- **本地访问**: http://localhost:8080
- **局域网访问**:
  - http://192.168.8.88:8080
  - http://10.0.0.119:8080
  - http://198.18.0.1:8080

## 🔧 使用方法

### 方法1：快速启动脚本

```bash
cd /home/jetson/aichess
./start_web_viewer.sh
```

### 方法2：直接运行

```bash
cd /home/jetson/aichess
source install/setup.bash
python3 web_viewer/gazebo_web_server.py
```

### 方法3：使用Launch文件（推荐）

```bash
cd /home/jetson/aichess
source install/setup.bash

# 启动完整系统（含GUI）
ros2 launch chess_simulation chess_web_viewer.launch.py

# 无头模式（服务器模式）
ros2 launch chess_simulation chess_web_viewer.launch.py gui:=false
```

## 🖼️ 界面功能说明

### 主界面

- **视频显示区域** - 显示实时仿真画面
- **全屏按钮** - 点击右上角可全屏显示
- **性能统计** - 显示FPS、分辨率、延迟等指标

### 控制面板

- **仿真控制**
  - 🟢 启动仿真 - 启动Gazebo仿真环境
  - 🔴 停止仿真 - 关闭仿真进程
  - 🔄 重置仿真 - 重置仿真状态

- **系统状态**
  - Gazebo状态 - 显示仿真进程运行状态
  - ROS2连接 - 显示与ROS2系统的连接状态
  - 视频流 - 显示视频流传输状态

- **快速操作**
  - 🔄 刷新页面 - 重新加载界面
  - 📷 截图保存 - 下载当前画面

### 日志面板

实时显示系统运行日志，包括：
- 连接状态变化
- 操作结果反馈
- 错误信息提示

## ⌨️ 快捷键

- **Esc** - 退出全屏模式
- **F11** - 切换全屏显示
- **Ctrl+R** - 刷新页面

## 🔧 技术架构

### 后端 (Python/Flask)
- **Flask** - Web框架
- **Flask-SocketIO** - WebSocket实时通信
- **ROS2 rclpy** - ROS2节点通信
- **OpenCV** - 图像处理
- **cv_bridge** - ROS图像消息转换

### 前端 (HTML/JavaScript)
- **TailwindCSS** - UI样式框架
- **Socket.IO** - 客户端WebSocket
- **Vanilla JavaScript** - 交互逻辑

## 📊 系统要求

- **操作系统**: Ubuntu 22.04 LTS
- **ROS2版本**: Humble
- **Python版本**: 3.10+
- **浏览器**: Chrome 80+, Firefox 75+, Safari 13+

## 🛠️ 故障排除

### 常见问题

1. **无法访问Web界面**
   - 检查端口8080是否被占用: `netstat -tuln | grep 8080`
   - 确认Web服务器正在运行
   - 检查防火墙设置

2. **视频流无法显示**
   - 确认ROS2话题 `/camera/rgb/image_raw` 有数据发布
   - 检查摄像头节点是否正常运行: `ros2 node list`
   - 查看日志输出是否有错误信息

3. **Gazebo启动失败**
   - 检查显示环境变量: `echo $DISPLAY`
   - 尝试无头模式: `gui:=false`
   - 确认Gazebo版本兼容性

4. **性能问题**
   - 降低视频质量设置
   - 检查网络带宽
   - 监控系统资源使用

### 调试命令

```bash
# 检查ROS2话题
ros2 topic list
ros2 topic echo /camera/rgb/image_raw --field header

# 检查节点状态
ros2 node list
ros2 node info /gazebo_web_viewer

# 查看系统日志
journalctl -f | grep gazebo
tail -f ~/.ros/log/*/gazebo_web_server*
```

## 🚀 高级功能

### 自定义配置

可以通过修改 `gazebo_web_server.py` 中的参数来自定义：
- 视频分辨率和质量
- 帧率设置
- 端口号
- 网络接口

### 扩展开发

系统采用模块化设计，支持：
- 添加新的控制API
- 集成更多传感器数据
- 自定义UI组件
- 添加数据录制功能

## 📝 更新日志

- **v1.0.0** (2025-09-21)
  - 初始版本发布
  - 基础Web界面和视频流
  - 仿真控制功能
  - 性能监控和日志

## 📞 技术支持

如遇到问题，请检查：
1. 系统日志输出
2. 网络连接状态
3. ROS2环境配置
4. 依赖库版本

---

**注意**: 这是开发测试版本，不建议在生产环境中使用。如需生产部署，建议使用专业的WSGI服务器如Gunicorn。