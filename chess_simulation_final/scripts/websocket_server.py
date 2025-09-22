#!/usr/bin/env python3
"""
独立的WebSocket服务器
连接到已运行的Gazebo仿真环境
"""

import asyncio
import json
import websockets
import subprocess
import logging
import signal
import sys
from typing import Set, Dict, Any

# 配置日志
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

class GazeboWebSocketServer:
    def __init__(self, port: int = 9002):
        self.port = port
        self.clients: Set[websockets.WebSocketServerProtocol] = set()
        self.running = True

    async def register_client(self, websocket):
        """注册新客户端"""
        self.clients.add(websocket)
        logger.info(f"客户端连接: {websocket.remote_address}")

        # 发送欢迎消息
        welcome_msg = {
            "type": "welcome",
            "data": {
                "message": "已连接到Gazebo WebSocket服务器",
                "server_time": asyncio.get_event_loop().time()
            }
        }
        await websocket.send(json.dumps(welcome_msg))

    async def unregister_client(self, websocket):
        """注销客户端"""
        self.clients.discard(websocket)
        logger.info(f"客户端断开: {websocket.remote_address}")

    async def broadcast_message(self, message: Dict[str, Any]):
        """向所有客户端广播消息"""
        if self.clients:
            disconnected = set()
            for client in self.clients:
                try:
                    await client.send(json.dumps(message))
                except websockets.exceptions.ConnectionClosed:
                    disconnected.add(client)

            # 清理断开的连接
            for client in disconnected:
                self.clients.discard(client)

    def execute_gz_command(self, command: str) -> str:
        """执行Gazebo命令"""
        try:
            cmd_parts = command.split()
            if not cmd_parts:
                return "错误: 空命令"

            # 安全的命令白名单
            allowed_commands = ['gz', 'model', 'service', 'topic', 'sim']
            if cmd_parts[0] not in allowed_commands:
                return f"错误: 不允许的命令 {cmd_parts[0]}"

            result = subprocess.run(
                cmd_parts,
                capture_output=True,
                text=True,
                timeout=10
            )

            if result.returncode == 0:
                return result.stdout.strip()
            else:
                return f"错误: {result.stderr.strip()}"

        except subprocess.TimeoutExpired:
            return "错误: 命令超时"
        except Exception as e:
            return f"错误: {str(e)}"

    def get_gazebo_status(self) -> Dict[str, Any]:
        """获取Gazebo状态信息"""
        try:
            # 检查Gazebo服务
            services_result = subprocess.run(
                ["gz", "service", "-l"],
                capture_output=True,
                text=True,
                timeout=5
            )

            # 检查模型列表
            models_result = subprocess.run(
                ["gz", "model", "--list"],
                capture_output=True,
                text=True,
                timeout=5
            )

            # 检查话题列表
            topics_result = subprocess.run(
                ["gz", "topic", "-l"],
                capture_output=True,
                text=True,
                timeout=5
            )

            return {
                "gazebo_running": services_result.returncode == 0,
                "services": services_result.stdout.strip().split('\n') if services_result.returncode == 0 else [],
                "models": models_result.stdout.strip().split('\n') if models_result.returncode == 0 else [],
                "topics": topics_result.stdout.strip().split('\n') if topics_result.returncode == 0 else [],
                "timestamp": asyncio.get_event_loop().time()
            }

        except Exception as e:
            logger.error(f"获取Gazebo状态失败: {e}")
            return {
                "gazebo_running": False,
                "error": str(e),
                "timestamp": asyncio.get_event_loop().time()
            }

    async def handle_client_message(self, websocket, message: str):
        """处理客户端消息"""
        try:
            data = json.loads(message)
            msg_type = data.get("type")

            if msg_type == "gz_command":
                # 执行Gazebo命令
                command = data.get("command", "")
                result = self.execute_gz_command(command)

                response = {
                    "type": "command_result",
                    "command": command,
                    "result": result,
                    "timestamp": asyncio.get_event_loop().time()
                }
                await websocket.send(json.dumps(response))

            elif msg_type == "get_status":
                # 获取状态信息
                status = self.get_gazebo_status()
                response = {
                    "type": "status_update",
                    "data": status
                }
                await websocket.send(json.dumps(response))

            elif msg_type == "ping":
                # 心跳检测
                response = {
                    "type": "pong",
                    "timestamp": asyncio.get_event_loop().time()
                }
                await websocket.send(json.dumps(response))

            else:
                logger.warning(f"未知消息类型: {msg_type}")

        except json.JSONDecodeError:
            logger.error("无效的JSON消息")
        except Exception as e:
            logger.error(f"处理消息失败: {e}")

    async def client_handler(self, websocket, path):
        """处理客户端连接"""
        await self.register_client(websocket)

        try:
            async for message in websocket:
                await self.handle_client_message(websocket, message)

        except websockets.exceptions.ConnectionClosed:
            pass
        except Exception as e:
            logger.error(f"客户端处理错误: {e}")
        finally:
            await self.unregister_client(websocket)

    async def periodic_status_broadcast(self):
        """定期广播状态信息"""
        while self.running:
            try:
                status = self.get_gazebo_status()
                message = {
                    "type": "status_broadcast",
                    "data": status
                }
                await self.broadcast_message(message)

                # 每5秒广播一次
                await asyncio.sleep(5)

            except Exception as e:
                logger.error(f"状态广播失败: {e}")
                await asyncio.sleep(5)

    async def start_server(self):
        """启动WebSocket服务器"""
        logger.info(f"启动WebSocket服务器在端口 {self.port}")

        # 启动服务器和状态广播任务
        server = websockets.serve(self.client_handler, "0.0.0.0", self.port)
        status_task = asyncio.create_task(self.periodic_status_broadcast())

        await asyncio.gather(server, status_task)

    def stop_server(self):
        """停止服务器"""
        self.running = False
        logger.info("正在停止WebSocket服务器...")

def signal_handler(signum, frame):
    """信号处理器"""
    logger.info(f"收到信号 {signum}, 正在退出...")
    sys.exit(0)

async def main():
    # 设置信号处理
    signal.signal(signal.SIGINT, signal_handler)
    signal.signal(signal.SIGTERM, signal_handler)

    server = GazeboWebSocketServer(9002)

    try:
        await server.start_server()
    except KeyboardInterrupt:
        logger.info("收到中断信号")
    finally:
        server.stop_server()

if __name__ == "__main__":
    print("🚀 启动Gazebo WebSocket服务器")
    print("端口: 9002")
    print("按 Ctrl+C 停止")
    print("=" * 50)

    asyncio.run(main())