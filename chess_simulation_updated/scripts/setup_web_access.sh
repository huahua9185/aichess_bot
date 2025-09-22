#!/bin/bash

# Gazebo Web访问配置脚本
# 配置网络访问、端口和服务

set -e

# 颜色输出
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

echo -e "${BLUE}=== Gazebo Web访问配置脚本 ===${NC}"

# 获取主要IP地址
PRIMARY_IP=$(hostname -I | awk '{print $1}')
echo -e "${GREEN}检测到主要IP地址: ${YELLOW}$PRIMARY_IP${NC}"

# 检查必要的端口
PORTS=(8080 9002 3000)
echo -e "${BLUE}检查端口可用性...${NC}"

for port in "${PORTS[@]}"; do
    if netstat -tuln 2>/dev/null | grep -q ":$port "; then
        echo -e "${YELLOW}警告: 端口 $port 已被占用${NC}"
        echo -e "  使用以下命令查看占用进程:"
        echo -e "  ${BLUE}sudo lsof -i :$port${NC}"
    else
        echo -e "${GREEN}端口 $port 可用${NC}"
    fi
done

# 创建Web服务目录
WEB_DIR="/tmp/gazebo_web"
mkdir -p "$WEB_DIR"
echo -e "${GREEN}创建Web服务目录: $WEB_DIR${NC}"

# 创建简单的HTTP服务器脚本
cat > "$WEB_DIR/simple_server.py" << 'EOF'
#!/usr/bin/env python3
import http.server
import socketserver
import os
import sys

PORT = 3000
Handler = http.server.SimpleHTTPRequestHandler

# 设置CORS headers
class CORSHTTPRequestHandler(Handler):
    def end_headers(self):
        self.send_header('Access-Control-Allow-Origin', '*')
        self.send_header('Access-Control-Allow-Methods', 'GET, POST, OPTIONS')
        self.send_header('Access-Control-Allow-Headers', 'Content-Type')
        super().end_headers()

    def do_OPTIONS(self):
        self.send_response(200)
        self.end_headers()

if __name__ == "__main__":
    web_dir = sys.argv[1] if len(sys.argv) > 1 else "."
    os.chdir(web_dir)

    with socketserver.TCPServer(("", PORT), CORSHTTPRequestHandler) as httpd:
        print(f"Serving at http://0.0.0.0:{PORT}")
        print(f"Web directory: {os.getcwd()}")
        httpd.serve_forever()
EOF

chmod +x "$WEB_DIR/simple_server.py"

# 复制Web文件到服务目录
WEB_SOURCE="/home/jetson/chess_robot_ws/src/chess_simulation/web"
if [ -f "$WEB_SOURCE/index.html" ]; then
    cp -r "$WEB_SOURCE"/* "$WEB_DIR/"
    echo -e "${GREEN}Web文件已复制到服务目录${NC}"
else
    echo -e "${YELLOW}未找到Web源文件，将创建简单的测试页面${NC}"
    cat > "$WEB_DIR/index.html" << EOF
<!DOCTYPE html>
<html>
<head>
    <title>Gazebo Web Access Test</title>
    <style>
        body { font-family: Arial, sans-serif; margin: 50px; }
        .status { padding: 20px; border-radius: 5px; margin: 10px 0; }
        .success { background: #d4edda; color: #155724; }
        .info { background: #d1ecf1; color: #0c5460; }
    </style>
</head>
<body>
    <h1>🤖 Gazebo象棋机器人 Web访问测试</h1>
    <div class="status success">
        <h3>✅ Web服务器运行正常</h3>
        <p>当前时间: <span id="time"></span></p>
    </div>

    <div class="status info">
        <h3>📋 访问信息</h3>
        <ul>
            <li><strong>本机访问:</strong> <a href="http://localhost:3000">http://localhost:3000</a></li>
            <li><strong>局域网访问:</strong> <a href="http://$PRIMARY_IP:3000">http://$PRIMARY_IP:3000</a></li>
            <li><strong>Gazebo仿真:</strong> <a href="http://$PRIMARY_IP:8080">http://$PRIMARY_IP:8080</a></li>
            <li><strong>WebSocket:</strong> ws://$PRIMARY_IP:9002</li>
        </ul>
    </div>

    <div class="status info">
        <h3>🔧 测试步骤</h3>
        <ol>
            <li>启动Gazebo仿真: <code>./start_web_gazebo.sh</code></li>
            <li>启动WebSocket桥接: <code>python3 websocket_bridge.py</code></li>
            <li>访问完整界面: <a href="index.html">完整监控界面</a></li>
        </ol>
    </div>

    <script>
        function updateTime() {
            document.getElementById('time').textContent = new Date().toLocaleString();
        }
        updateTime();
        setInterval(updateTime, 1000);
    </script>
</body>
</html>
EOF
fi

# 生成启动脚本
cat > "$WEB_DIR/start_web_services.sh" << EOF
#!/bin/bash

# 启动所有Web服务

echo "🚀 启动Gazebo Web服务..."

# 获取脚本目录
SCRIPT_DIR="\$(cd "\$(dirname "\${BASH_SOURCE[0]}")" && pwd)"
SIMULATION_DIR="/home/jetson/chess_robot_ws/src/chess_simulation"

echo "📁 工作目录: \$SCRIPT_DIR"
echo "🌐 Web界面: http://$PRIMARY_IP:3000"
echo "🎮 Gazebo仿真: http://$PRIMARY_IP:8080"
echo "🔌 WebSocket: ws://$PRIMARY_IP:9002"

# 启动Web服务器
echo "启动Web服务器..."
cd "\$SCRIPT_DIR"
python3 simple_server.py &
WEB_PID=\$!

# 启动WebSocket桥接（如果存在）
if [ -f "\$SIMULATION_DIR/scripts/websocket_bridge.py" ]; then
    echo "启动WebSocket桥接..."
    cd "\$SIMULATION_DIR"
    source /opt/ros/humble/setup.bash
    if [ -f "/home/jetson/chess_robot_ws/install/setup.bash" ]; then
        source /home/jetson/chess_robot_ws/install/setup.bash
    fi
    python3 scripts/websocket_bridge.py &
    WS_PID=\$!
fi

# 清理函数
cleanup() {
    echo "🛑 停止服务..."
    kill \$WEB_PID 2>/dev/null || true
    [ ! -z "\$WS_PID" ] && kill \$WS_PID 2>/dev/null || true
    exit 0
}

trap cleanup SIGINT SIGTERM

echo "✅ 所有服务已启动"
echo "按 Ctrl+C 停止服务"

# 等待
wait
EOF

chmod +x "$WEB_DIR/start_web_services.sh"

# 输出访问信息
echo -e "\n${GREEN}=== Web访问配置完成 ===${NC}"
echo -e "${BLUE}Web服务目录:${NC} $WEB_DIR"
echo -e "${BLUE}主要IP地址:${NC} $PRIMARY_IP"
echo -e "\n${YELLOW}访问地址:${NC}"
echo -e "  🌐 Web界面: ${BLUE}http://$PRIMARY_IP:3000${NC}"
echo -e "  🎮 Gazebo仿真: ${BLUE}http://$PRIMARY_IP:8080${NC}"
echo -e "  🔌 WebSocket: ${BLUE}ws://$PRIMARY_IP:9002${NC}"

echo -e "\n${YELLOW}启动命令:${NC}"
echo -e "  📱 启动Web服务: ${BLUE}cd $WEB_DIR && ./start_web_services.sh${NC}"
echo -e "  🎮 启动Gazebo: ${BLUE}cd /home/jetson/chess_robot_ws/src/chess_simulation/scripts && ./start_web_gazebo.sh${NC}"

echo -e "\n${GREEN}配置完成！${NC}"