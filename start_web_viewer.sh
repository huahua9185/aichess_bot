#!/bin/bash
# Gazebo Web查看器启动脚本

cd /home/jetson/aichess
source install/setup.bash 2>/dev/null || true

export ROS_DOMAIN_ID=0
export PYTHONPATH=/home/jetson/aichess/src:$PYTHONPATH

echo "🚀 启动Gazebo Web查看器..."
echo "请在浏览器中访问:"
for ip in $(ip addr show | grep "inet " | grep -v "127.0.0.1" | awk '{print $2}' | cut -d'/' -f1); do
    echo "  🌍 http://$ip:8080"
done
echo ""
echo "按 Ctrl+C 停止服务器"
echo ""

python3 web_viewer/gazebo_web_server_fixed.py
