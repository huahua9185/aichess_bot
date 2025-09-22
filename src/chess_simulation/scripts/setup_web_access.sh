#!/bin/bash
"""
配置Gazebo Web访问环境
设置网络和权限配置
"""

echo "🌐 配置象棋机器人Gazebo Web访问环境"
echo "======================================"

# 检查当前用户权限
if [ "$EUID" -eq 0 ]; then
    echo "⚠️  请不要使用root用户运行此脚本"
    exit 1
fi

# 获取网络信息
echo "📡 网络配置信息:"
echo "本机IP地址:"
ip addr show | grep "inet " | grep -v "127.0.0.1" | awk '{print "  - " $2}' | head -3

echo ""
echo "可用的访问地址:"
for ip in $(ip addr show | grep "inet " | grep -v "127.0.0.1" | awk '{print $2}' | cut -d'/' -f1); do
    echo "  🌍 http://$ip:8080"
done

echo ""
echo "局域网设备可通过以上任意地址访问Web界面"
echo ""

# 检查端口占用
echo "🔍 检查端口占用状态:"
if netstat -tuln | grep -q :8080; then
    echo "  ❌ 端口8080已被占用"
    echo "  占用进程:"
    sudo lsof -i :8080 2>/dev/null || echo "    无法查询占用进程"
    exit 1
else
    echo "  ✅ 端口8080可用"
fi

# 检查必要的Python库
echo ""
echo "🐍 检查Python依赖:"
python3 -c "
import sys
libraries = ['flask', 'flask_socketio', 'cv2', 'rclpy', 'PIL']
missing = []

for lib in libraries:
    try:
        __import__(lib)
        print(f'  ✅ {lib}')
    except ImportError:
        print(f'  ❌ {lib}')
        missing.append(lib)

if missing:
    print(f'\n❌ 缺少依赖: {missing}')
    print('请运行: pip3 install flask flask-socketio opencv-python pillow')
    sys.exit(1)
else:
    print('\n✅ 所有Python依赖已满足')
"

if [ $? -ne 0 ]; then
    exit 1
fi

# 检查ROS2环境
echo ""
echo "🤖 检查ROS2环境:"
if [ -z "$ROS_DOMAIN_ID" ]; then
    echo "  ⚠️  ROS_DOMAIN_ID未设置，使用默认值0"
    export ROS_DOMAIN_ID=0
else
    echo "  ✅ ROS_DOMAIN_ID=$ROS_DOMAIN_ID"
fi

# 检查Gazebo
echo ""
echo "🏗️  检查Gazebo环境:"
if which gz > /dev/null; then
    echo "  ✅ Gazebo已安装"
    gz --version | head -1 | sed 's/^/    /'
else
    echo "  ❌ Gazebo未找到"
    exit 1
fi

# 创建运行脚本
echo ""
echo "📝 创建启动脚本..."
cat > /home/jetson/aichess/start_web_viewer.sh << 'EOF'
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

python3 web_viewer/gazebo_web_server.py
EOF

chmod +x /home/jetson/aichess/start_web_viewer.sh

echo "  ✅ 启动脚本已创建: /home/jetson/aichess/start_web_viewer.sh"

# 创建服务文件(可选)
echo ""
echo "🔧 创建系统服务文件(可选)..."
cat > /tmp/gazebo-web-viewer.service << EOF
[Unit]
Description=Gazebo Chess Robot Web Viewer
After=network.target
Wants=network.target

[Service]
Type=simple
User=jetson
Group=jetson
WorkingDirectory=/home/jetson/aichess
Environment=ROS_DOMAIN_ID=0
Environment=PYTHONPATH=/home/jetson/aichess/src
ExecStart=/bin/bash /home/jetson/aichess/start_web_viewer.sh
Restart=on-failure
RestartSec=5

[Install]
WantedBy=multi-user.target
EOF

echo "  ✅ 系统服务文件已创建: /tmp/gazebo-web-viewer.service"
echo "  💡 如需开机自启动，请运行:"
echo "     sudo cp /tmp/gazebo-web-viewer.service /etc/systemd/system/"
echo "     sudo systemctl enable gazebo-web-viewer.service"

echo ""
echo "🎉 Web访问环境配置完成！"
echo ""
echo "使用方法:"
echo "  1. 直接运行: ./start_web_viewer.sh"
echo "  2. 或者运行: python3 web_viewer/gazebo_web_server.py"
echo ""
echo "📱 支持的客户端:"
echo "  - 桌面浏览器 (Chrome, Firefox, Safari)"
echo "  - 移动设备浏览器"
echo "  - 平板电脑"
echo ""
echo "🎮 功能特性:"
echo "  - ✅ 实时视频流 (30fps)"
echo "  - ✅ 仿真控制 (启动/停止/重置)"
echo "  - ✅ 系统状态监控"
echo "  - ✅ 性能指标显示"
echo "  - ✅ 响应式设计"
echo "  - ✅ 全屏显示"
echo "  - ✅ 截图功能"
echo ""