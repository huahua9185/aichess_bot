#!/usr/bin/env python3
"""
WebSocket桥接服务
将Gazebo的话题数据通过WebSocket发送到Web客户端
支持实时监控仿真状态和数据流
"""

import asyncio
import websockets
import json
import threading
import time
from typing import Dict, Set, Any
import logging

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from std_msgs.msg import String
from sensor_msgs.msg import Image, JointState
from geometry_msgs.msg import Pose, Twist
from chess_interfaces.msg import BoardState, ChessMove, SystemStatus

# 配置日志
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)


class WebSocketBridge(Node):
    """WebSocket桥接节点"""

    def __init__(self):
        super().__init__('websocket_bridge')

        # WebSocket连接管理
        self.clients: Set[websockets.WebSocketServerProtocol] = set()
        self.message_queue = asyncio.Queue()

        # 配置QoS
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            depth=10
        )

        # 订阅关键话题
        self.create_subscription(
            BoardState, '/chess/board_state',
            self._board_state_callback, qos_profile
        )

        self.create_subscription(
            ChessMove, '/chess/last_move',
            self._chess_move_callback, qos_profile
        )

        self.create_subscription(
            SystemStatus, '/chess/system_status',
            self._system_status_callback, qos_profile
        )

        self.create_subscription(
            JointState, '/dofbot_pro/joint_states',
            self._joint_state_callback, qos_profile
        )

        self.create_subscription(
            Image, '/camera/rgb/image_raw',
            self._image_callback, qos_profile
        )

        # 发布者
        self.command_pub = self.create_publisher(
            String, '/chess/web_commands', 10
        )

        self.get_logger().info('WebSocket桥接服务已初始化')

    def _board_state_callback(self, msg: BoardState):
        """棋盘状态回调"""
        data = {
            'type': 'board_state',
            'timestamp': time.time(),
            'data': {
                'pieces': msg.pieces,
                'current_player': msg.current_player,
                'game_phase': msg.game_phase,
                'move_count': msg.move_count
            }
        }
        asyncio.create_task(self._broadcast_message(data))

    def _chess_move_callback(self, msg: ChessMove):
        """象棋移动回调"""
        data = {
            'type': 'chess_move',
            'timestamp': time.time(),
            'data': {
                'from_square': msg.from_square,
                'to_square': msg.to_square,
                'piece_type': msg.piece_type,
                'is_capture': msg.is_capture,
                'move_time': msg.move_time
            }
        }
        asyncio.create_task(self._broadcast_message(data))

    def _system_status_callback(self, msg: SystemStatus):
        """系统状态回调"""
        data = {
            'type': 'system_status',
            'timestamp': time.time(),
            'data': {
                'current_state': msg.current_state,
                'error_message': msg.error_message,
                'progress': msg.progress,
                'services_ready': msg.services_ready,
                'hardware_connected': msg.hardware_connected
            }
        }
        asyncio.create_task(self._broadcast_message(data))

    def _joint_state_callback(self, msg: JointState):
        """关节状态回调"""
        data = {
            'type': 'joint_state',
            'timestamp': time.time(),
            'data': {
                'joint_names': msg.name,
                'positions': list(msg.position),
                'velocities': list(msg.velocity) if msg.velocity else [],
                'efforts': list(msg.effort) if msg.effort else []
            }
        }
        asyncio.create_task(self._broadcast_message(data))

    def _image_callback(self, msg: Image):
        """图像回调（降频处理）"""
        # 只处理部分图像以减少带宽
        if hasattr(self, '_last_image_time'):
            if time.time() - self._last_image_time < 0.2:  # 5Hz
                return
        self._last_image_time = time.time()

        data = {
            'type': 'camera_image',
            'timestamp': time.time(),
            'data': {
                'width': msg.width,
                'height': msg.height,
                'encoding': msg.encoding,
                'is_bigendian': msg.is_bigendian,
                'step': msg.step,
                # 注意：实际应用中需要将图像数据转换为base64
                'data_size': len(msg.data)
            }
        }
        asyncio.create_task(self._broadcast_message(data))

    async def _broadcast_message(self, data: Dict[str, Any]):
        """广播消息到所有WebSocket客户端"""
        if not self.clients:
            return

        message = json.dumps(data)
        disconnected_clients = set()

        for client in self.clients:
            try:
                await client.send(message)
            except websockets.exceptions.ConnectionClosed:
                disconnected_clients.add(client)
            except Exception as e:
                logger.warning(f"发送消息失败: {e}")
                disconnected_clients.add(client)

        # 移除断开的连接
        self.clients -= disconnected_clients

    async def _handle_client_message(self, websocket, message: str):
        """处理客户端消息"""
        try:
            data = json.loads(message)
            command_type = data.get('type')

            if command_type == 'start_game':
                self._publish_command('start_game')
            elif command_type == 'pause_game':
                self._publish_command('pause_game')
            elif command_type == 'reset_game':
                self._publish_command('reset_game')
            elif command_type == 'emergency_stop':
                self._publish_command('emergency_stop')
            else:
                logger.warning(f"未知命令类型: {command_type}")

        except json.JSONDecodeError:
            logger.error(f"无效的JSON消息: {message}")
        except Exception as e:
            logger.error(f"处理客户端消息失败: {e}")

    def _publish_command(self, command: str):
        """发布命令到ROS话题"""
        msg = String()
        msg.data = command
        self.command_pub.publish(msg)
        self.get_logger().info(f"发布命令: {command}")

    async def websocket_handler(self, websocket, path):
        """WebSocket连接处理器"""
        self.clients.add(websocket)
        client_addr = websocket.remote_address
        logger.info(f"新的WebSocket连接: {client_addr}")

        try:
            # 发送欢迎消息
            welcome_msg = {
                'type': 'welcome',
                'timestamp': time.time(),
                'data': {
                    'message': '欢迎连接到Gazebo象棋机器人系统',
                    'client_count': len(self.clients)
                }
            }
            await websocket.send(json.dumps(welcome_msg))

            # 监听客户端消息
            async for message in websocket:
                await self._handle_client_message(websocket, message)

        except websockets.exceptions.ConnectionClosed:
            logger.info(f"WebSocket连接已关闭: {client_addr}")
        except Exception as e:
            logger.error(f"WebSocket处理错误: {e}")
        finally:
            self.clients.discard(websocket)


def main():
    """主函数"""
    # 初始化ROS2
    rclpy.init()

    # 创建桥接节点
    bridge = WebSocketBridge()

    # 在单独线程中运行ROS2
    def spin_ros():
        try:
            rclpy.spin(bridge)
        except Exception as e:
            logger.error(f"ROS2运行错误: {e}")
        finally:
            bridge.destroy_node()
            rclpy.shutdown()

    ros_thread = threading.Thread(target=spin_ros)
    ros_thread.daemon = True
    ros_thread.start()

    # 启动WebSocket服务器
    async def start_server():
        server = await websockets.serve(
            bridge.websocket_handler,
            "0.0.0.0",  # 监听所有接口
            9002,       # WebSocket端口
            ping_interval=20,
            ping_timeout=10
        )

        logger.info("WebSocket服务器已启动在端口 9002")
        logger.info("等待客户端连接...")

        await server.wait_closed()

    try:
        asyncio.run(start_server())
    except KeyboardInterrupt:
        logger.info("WebSocket桥接服务已停止")
    except Exception as e:
        logger.error(f"服务器错误: {e}")


if __name__ == '__main__':
    main()