#!/bin/bash

# Gazebo Web GUI启动脚本
# 允许通过浏览器远程访问Gazebo仿真环境

set -e

# 获取脚本目录
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
CONFIG_DIR="$(dirname "$SCRIPT_DIR")/config"

# 颜色输出
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

echo -e "${BLUE}=== Gazebo Web GUI启动脚本 ===${NC}"

# 检查是否已source ROS2环境
if [ -z "$ROS_DISTRO" ]; then
    echo -e "${YELLOW}警告: 未检测到ROS2环境，正在source...${NC}"
    source /opt/ros/humble/setup.bash
    if [ -f "/home/jetson/chess_robot_ws/install/setup.bash" ]; then
        source /home/jetson/chess_robot_ws/install/setup.bash
    fi
fi

# 设置Gazebo环境变量
export GZ_SIM_RESOURCE_PATH="/home/jetson/chess_robot_ws/src/chess_simulation/worlds:/home/jetson/chess_robot_ws/src/chess_simulation/models:$GZ_SIM_RESOURCE_PATH"

# 设置Web GUI环境变量
export GZ_SIM_SERVER_CONFIG_PATH="$CONFIG_DIR/gazebo_web.yaml"

# 获取本机IP地址
LOCAL_IP=$(hostname -I | awk '{print $1}')

echo -e "${GREEN}配置信息:${NC}"
echo -e "  本机IP: ${YELLOW}$LOCAL_IP${NC}"
echo -e "  Web访问地址: ${YELLOW}http://$LOCAL_IP:8080${NC}"
echo -e "  WebSocket端口: ${YELLOW}9002${NC}"
echo -e "  配置文件: ${YELLOW}$CONFIG_DIR/gazebo_web.yaml${NC}"

# 检查端口是否被占用
check_port() {
    local port=$1
    if netstat -tuln | grep -q ":$port "; then
        echo -e "${RED}错误: 端口 $port 已被占用${NC}"
        echo -e "${YELLOW}请检查是否有其他Gazebo实例正在运行${NC}"
        return 1
    fi
}

echo -e "${BLUE}检查端口可用性...${NC}"
check_port 8080 || exit 1
check_port 9002 || exit 1

# 创建日志目录
LOG_DIR="/tmp/gazebo_web_logs"
mkdir -p "$LOG_DIR"

echo -e "${GREEN}启动Gazebo Web服务器...${NC}"

# 启动参数
WORLD_FILE="/home/jetson/chess_robot_ws/src/chess_simulation/worlds/chess_world.sdf"
GAZEBO_ARGS="--verbose --gui-config $CONFIG_DIR/gazebo_web.yaml"

# 启动信息
echo -e "${BLUE}启动命令:${NC}"
echo -e "  gz sim $WORLD_FILE $GAZEBO_ARGS"
echo -e ""
echo -e "${GREEN}启动中...${NC}"
echo -e "${YELLOW}注意: 首次启动可能需要几分钟来下载和配置资源${NC}"
echo -e "${YELLOW}Web界面将在 http://$LOCAL_IP:8080 上可用${NC}"
echo -e ""
echo -e "${BLUE}按 Ctrl+C 退出${NC}"

# 启动Gazebo
gz sim "$WORLD_FILE" $GAZEBO_ARGS