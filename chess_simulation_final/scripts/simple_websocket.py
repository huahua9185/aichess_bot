#!/usr/bin/env python3

import asyncio
import websockets
import json
import subprocess

async def handle_client(websocket, path=None):
    print(f"客户端连接: {websocket.remote_address}")

    # 发送欢迎消息
    welcome = {
        "type": "welcome",
        "data": {"message": "已连接到Gazebo WebSocket服务器"}
    }
    await websocket.send(json.dumps(welcome))

    try:
        async for message in websocket:
            try:
                data = json.loads(message)
                if data.get("type") == "get_status":
                    # 获取Gazebo状态
                    try:
                        models = subprocess.run(["gz", "model", "--list"],
                                              capture_output=True, text=True, timeout=5)
                        response = {
                            "type": "status_update",
                            "data": {
                                "gazebo_running": models.returncode == 0,
                                "models": models.stdout.strip().split('\n') if models.returncode == 0 else []
                            }
                        }
                    except:
                        response = {
                            "type": "status_update",
                            "data": {"gazebo_running": False, "models": []}
                        }
                    await websocket.send(json.dumps(response))

            except json.JSONDecodeError:
                pass

    except websockets.exceptions.ConnectionClosed:
        pass

    print(f"客户端断开: {websocket.remote_address}")

async def main():
    print("启动WebSocket服务器在端口 9004")
    print("等待客户端连接...")
    async with websockets.serve(handle_client, "0.0.0.0", 9004):
        await asyncio.Future()  # 永远运行

if __name__ == "__main__":
    asyncio.run(main())