#!/usr/bin/env python3

import asyncio
import websockets
import json

async def test_websocket():
    uri = "ws://localhost:9004"
    try:
        async with websockets.connect(uri) as websocket:
            print(f"已连接到 {uri}")

            # 监听欢迎消息
            welcome = await websocket.recv()
            print(f"收到消息: {welcome}")

            # 发送状态请求
            status_request = {"type": "get_status"}
            await websocket.send(json.dumps(status_request))
            print("已发送状态请求")

            # 接收状态响应
            response = await websocket.recv()
            print(f"收到状态响应: {response}")

    except Exception as e:
        print(f"连接失败: {e}")

if __name__ == "__main__":
    asyncio.run(test_websocket())