#!/usr/bin/env python3
"""
Gazebo Garden Web服务启动器
支持通过Web界面访问Gazebo仿真
"""

import subprocess
import time
import os
import signal
import sys
from threading import Thread
from http.server import HTTPServer, SimpleHTTPRequestHandler
import socket

class GazeboWebLauncher:
    def __init__(self):
        self.gazebo_process = None
        self.http_server = None

    def find_free_port(self, start=8080, end=8090):
        """查找可用端口"""
        for port in range(start, end):
            with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
                try:
                    s.bind(('', port))
                    return port
                except:
                    continue
        return None

    def start_gazebo_server(self):
        """启动Gazebo服务器，支持WebSocket连接"""
        print("=" * 60)
        print("启动Gazebo Garden服务器（支持Web访问）")
        print("=" * 60)

        # Gazebo Garden命令，启用WebSocket支持
        cmd = [
            "gz", "sim",
            "--headless-rendering",  # 无头模式
            "--gui-config", "/home/jetson/chess_robot_ws/src/chess_simulation/config/gui.config",  # GUI配置
            "-s",  # 服务器模式
            "-v", "4",  # 详细日志
            "/home/jetson/chess_robot_ws/src/chess_simulation/worlds/chess_world.sdf"
        ]

        print(f"执行命令: {' '.join(cmd)}")

        # 设置环境变量以启用WebSocket
        env = os.environ.copy()
        env['GZ_SIM_SERVER_WEBSOCKET_PORT'] = '9002'  # WebSocket端口
        env['GZ_SIM_RESOURCE_PATH'] = '/home/jetson/chess_robot_ws/src/chess_simulation/models'

        try:
            self.gazebo_process = subprocess.Popen(
                cmd,
                env=env,
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE
            )
            print(f"✓ Gazebo进程已启动 (PID: {self.gazebo_process.pid})")
            print(f"✓ WebSocket服务端口: 9002")
            return True
        except Exception as e:
            print(f"✗ 启动Gazebo失败: {e}")
            return False

    def start_web_server(self):
        """启动Web服务器"""
        port = self.find_free_port()
        if not port:
            print("✗ 找不到可用端口")
            return False

        class CustomHandler(SimpleHTTPRequestHandler):
            def __init__(self, *args, **kwargs):
                super().__init__(*args, directory='/home/jetson/chess_robot_ws/src/chess_simulation/web', **kwargs)

            def end_headers(self):
                self.send_header('Cross-Origin-Embedder-Policy', 'credentialless')
                self.send_header('Cross-Origin-Opener-Policy', 'same-origin')
                self.send_header('Access-Control-Allow-Origin', '*')
                super().end_headers()

        try:
            self.http_server = HTTPServer(('0.0.0.0', port), CustomHandler)
            print(f"✓ Web服务器已启动")
            print(f"✓ 访问地址: http://localhost:{port}")

            # 获取局域网IP
            import socket
            hostname = socket.gethostname()
            local_ip = socket.gethostbyname(hostname)
            print(f"✓ 局域网访问: http://{local_ip}:{port}")

            # 在新线程中运行服务器
            server_thread = Thread(target=self.http_server.serve_forever)
            server_thread.daemon = True
            server_thread.start()

            return True
        except Exception as e:
            print(f"✗ 启动Web服务器失败: {e}")
            return False

    def check_gazebo_status(self):
        """检查Gazebo服务状态"""
        try:
            result = subprocess.run(
                ["gz", "service", "-l"],
                capture_output=True,
                text=True,
                timeout=2
            )
            if result.returncode == 0:
                services = result.stdout.strip().split('\n')
                print(f"✓ 发现 {len(services)} 个Gazebo服务")
                return True
        except:
            pass
        return False

    def cleanup(self):
        """清理资源"""
        print("\n正在清理...")

        if self.gazebo_process:
            self.gazebo_process.terminate()
            try:
                self.gazebo_process.wait(timeout=5)
            except:
                self.gazebo_process.kill()
            print("✓ Gazebo进程已停止")

        if self.http_server:
            self.http_server.shutdown()
            print("✓ Web服务器已停止")

    def signal_handler(self, sig, frame):
        """处理退出信号"""
        print("\n收到退出信号...")
        self.cleanup()
        sys.exit(0)

    def run(self):
        """运行主程序"""
        # 设置信号处理
        signal.signal(signal.SIGINT, self.signal_handler)
        signal.signal(signal.SIGTERM, self.signal_handler)

        print("\n🚀 Gazebo Web服务启动器")
        print("=" * 60)

        # 启动Gazebo
        if not self.start_gazebo_server():
            return

        # 等待Gazebo初始化
        print("\n等待Gazebo初始化...")
        time.sleep(5)

        # 检查服务状态
        if self.check_gazebo_status():
            print("✓ Gazebo服务就绪")

        # 启动Web服务器
        if not self.start_web_server():
            self.cleanup()
            return

        print("\n" + "=" * 60)
        print("🎉 所有服务已启动成功!")
        print("📱 请在浏览器中访问Web界面")
        print("⌨️  按 Ctrl+C 停止服务")
        print("=" * 60 + "\n")

        # 保持运行
        try:
            while True:
                if self.gazebo_process and self.gazebo_process.poll() is not None:
                    print("✗ Gazebo进程意外退出")
                    break
                time.sleep(1)
        except KeyboardInterrupt:
            pass
        finally:
            self.cleanup()

if __name__ == "__main__":
    launcher = GazeboWebLauncher()
    launcher.run()