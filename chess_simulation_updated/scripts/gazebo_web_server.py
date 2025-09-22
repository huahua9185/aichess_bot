#!/usr/bin/env python3

import asyncio
import json
import websockets
import socket
from http.server import HTTPServer, SimpleHTTPRequestHandler
import threading
import subprocess
import os
import time

class GazeboWebBridge:
    def __init__(self, websocket_port=9002, http_port=8080):
        self.websocket_port = websocket_port
        self.http_port = http_port
        self.gz_process = None

    def start_gazebo(self, world_file=None):
        """启动Gazebo仿真器"""
        cmd = ["gz", "sim", "-v", "4", "-s", "--headless-rendering"]
        if world_file:
            cmd.append(world_file)

        print(f"启动Gazebo命令: {' '.join(cmd)}")
        self.gz_process = subprocess.Popen(cmd)
        time.sleep(5)  # 等待Gazebo启动

    async def handle_websocket(self, websocket, path):
        """处理WebSocket连接"""
        print(f"新的WebSocket连接: {websocket.remote_address}")
        try:
            async for message in websocket:
                data = json.loads(message)

                # 处理不同类型的命令
                if data.get('type') == 'gz_command':
                    result = self.execute_gz_command(data.get('command'))
                    await websocket.send(json.dumps({'type': 'command_result', 'result': result}))

                elif data.get('type') == 'scene_info':
                    scene_info = self.get_scene_info()
                    await websocket.send(json.dumps({'type': 'scene_data', 'data': scene_info}))

        except websockets.exceptions.ConnectionClosed:
            print(f"WebSocket连接关闭: {websocket.remote_address}")

    def execute_gz_command(self, command):
        """执行Gazebo命令"""
        try:
            result = subprocess.run(
                ["gz"] + command.split(),
                capture_output=True,
                text=True,
                timeout=5
            )
            return result.stdout
        except Exception as e:
            return f"错误: {str(e)}"

    def get_scene_info(self):
        """获取场景信息"""
        try:
            # 获取模型列表
            models = subprocess.run(
                ["gz", "model", "-l"],
                capture_output=True,
                text=True
            ).stdout

            # 获取主题列表
            topics = subprocess.run(
                ["gz", "topic", "-l"],
                capture_output=True,
                text=True
            ).stdout

            return {
                'models': models.strip().split('\n') if models else [],
                'topics': topics.strip().split('\n') if topics else []
            }
        except Exception as e:
            return {'error': str(e)}

    def serve_http(self):
        """启动HTTP服务器提供Web界面"""
        class CustomHTTPHandler(SimpleHTTPRequestHandler):
            def __init__(self, *args, **kwargs):
                super().__init__(*args, directory='/home/jetson/chess_robot_ws/src/chess_simulation/web', **kwargs)

            def end_headers(self):
                self.send_header('Access-Control-Allow-Origin', '*')
                self.send_header('Access-Control-Allow-Methods', 'GET, POST, OPTIONS')
                self.send_header('Access-Control-Allow-Headers', 'Content-Type')
                super().end_headers()

        httpd = HTTPServer(('0.0.0.0', self.http_port), CustomHTTPHandler)
        print(f"HTTP服务器启动在端口 {self.http_port}")
        httpd.serve_forever()

    async def start_websocket_server(self):
        """启动WebSocket服务器"""
        async with websockets.serve(self.handle_websocket, '0.0.0.0', self.websocket_port):
            print(f"WebSocket服务器启动在端口 {self.websocket_port}")
            await asyncio.Future()  # 永远运行

    def run(self):
        """运行所有服务"""
        # 启动HTTP服务器线程
        http_thread = threading.Thread(target=self.serve_http, daemon=True)
        http_thread.start()

        # 启动Gazebo
        self.start_gazebo()

        # 运行WebSocket服务器
        asyncio.run(self.start_websocket_server())

if __name__ == "__main__":
    print("===== Gazebo Web服务器 =====")
    print("访问地址: http://localhost:8080")
    print("WebSocket端口: 9002")
    print("=============================")

    bridge = GazeboWebBridge()
    try:
        bridge.run()
    except KeyboardInterrupt:
        print("\n服务器停止")
        if bridge.gz_process:
            bridge.gz_process.terminate()