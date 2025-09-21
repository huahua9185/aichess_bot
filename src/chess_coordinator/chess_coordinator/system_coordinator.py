#!/usr/bin/env python3
"""
象棋机器人系统协调器
统一管理整个象棋机器人系统的各个子系统
协调相机、视觉、游戏、机械臂等模块的工作流程
"""

import rclpy
from rclpy.node import Node
from chess_interfaces.msg import BoardState, ChessMove
from chess_interfaces.srv import DetectBoard, PlanMove, ExecuteMove
from std_msgs.msg import String
from geometry_msgs.msg import Point
import json
import time
import threading
from enum import Enum
from typing import Dict, List, Optional, Any
from dataclasses import dataclass


class SystemState(Enum):
    """系统状态枚举"""
    INITIALIZING = "initializing"
    IDLE = "idle"
    STARTING_GAME = "starting_game"
    WAITING_PLAYER_MOVE = "waiting_player_move"
    PROCESSING_PLAYER_MOVE = "processing_player_move"
    AI_THINKING = "ai_thinking"
    EXECUTING_AI_MOVE = "executing_ai_move"
    GAME_OVER = "game_over"
    ERROR = "error"
    MAINTENANCE = "maintenance"


@dataclass
class SubsystemStatus:
    """子系统状态数据类"""
    name: str
    status: str
    last_update: float
    error_count: int
    is_active: bool


class SystemCoordinator(Node):
    """象棋机器人系统协调器"""

    def __init__(self):
        super().__init__('system_coordinator')

        # 参数声明
        self.declare_parameter('auto_start', False)
        self.declare_parameter('simulation_mode', True)
        self.declare_parameter('health_check_interval', 5.0)
        self.declare_parameter('startup_timeout', 30.0)
        self.declare_parameter('move_timeout', 60.0)
        self.declare_parameter('max_errors_per_subsystem', 3)

        # 获取参数
        self.auto_start = self.get_parameter('auto_start').value
        self.simulation_mode = self.get_parameter('simulation_mode').value
        self.health_check_interval = self.get_parameter('health_check_interval').value
        self.startup_timeout = self.get_parameter('startup_timeout').value
        self.move_timeout = self.get_parameter('move_timeout').value
        self.max_errors_per_subsystem = self.get_parameter('max_errors_per_subsystem').value

        # 系统状态
        self.current_state = SystemState.INITIALIZING
        self.game_active = False
        self.system_start_time = time.time()
        self.current_operation_start_time = None

        # 子系统状态管理
        self.subsystems = {
            'camera': SubsystemStatus('camera', 'unknown', 0, 0, False),
            'vision': SubsystemStatus('vision', 'unknown', 0, 0, False),
            'game_engine': SubsystemStatus('game_engine', 'unknown', 0, 0, False),
            'chess_ai': SubsystemStatus('chess_ai', 'unknown', 0, 0, False),
            'arm_controller': SubsystemStatus('arm_controller', 'unknown', 0, 0, False)
        }

        # 游戏状态
        self.current_board_state = None
        self.last_move = None
        self.game_statistics = {
            'games_played': 0,
            'moves_made': 0,
            'errors_encountered': 0,
            'uptime_seconds': 0
        }

        # 线程锁
        self.state_lock = threading.Lock()

        # 订阅者 - 监听各子系统状态
        self.camera_status_sub = self.create_subscription(
            String, '/chess/camera_status', self.camera_status_callback, 10)
        self.vision_status_sub = self.create_subscription(
            String, '/chess/vision_status', self.vision_status_callback, 10)
        self.game_status_sub = self.create_subscription(
            String, '/chess/game_status', self.game_status_callback, 10)
        self.ai_status_sub = self.create_subscription(
            String, '/chess/ai_status', self.ai_status_callback, 10)
        self.arm_status_sub = self.create_subscription(
            String, '/chess/arm_status', self.arm_status_callback, 10)

        # 订阅游戏相关消息
        self.board_state_sub = self.create_subscription(
            BoardState, '/chess/board_state', self.board_state_callback, 10)
        self.move_status_sub = self.create_subscription(
            String, '/chess/move_status', self.move_status_callback, 10)

        # 发布者
        self.system_status_pub = self.create_publisher(
            String, '/chess/system_status', 10)
        self.game_command_pub = self.create_publisher(
            String, '/chess/game_command', 10)
        self.system_command_pub = self.create_publisher(
            String, '/chess/system_command', 10)

        # 服务客户端
        self.detect_board_client = self.create_client(
            DetectBoard, '/chess/detect_board')

        # 定时器
        self.main_loop_timer = self.create_timer(1.0, self.main_loop)
        self.health_check_timer = self.create_timer(
            self.health_check_interval, self.health_check)
        self.status_pub_timer = self.create_timer(2.0, self.publish_system_status)

        self.get_logger().info(f"系统协调器已启动 - 仿真模式: {self.simulation_mode}")

        # 开始初始化流程
        if self.auto_start:
            self.start_initialization()
        else:
            # 设置初始操作时间
            self.current_operation_start_time = time.time()

    def start_initialization(self):
        """开始系统初始化"""
        with self.state_lock:
            self.current_state = SystemState.INITIALIZING
            self.current_operation_start_time = time.time()

        self.get_logger().info("开始系统初始化...")

    def main_loop(self):
        """主循环 - 系统状态机"""
        with self.state_lock:
            current_time = time.time()

            # 更新系统运行时间
            self.game_statistics['uptime_seconds'] = current_time - self.system_start_time

            # 检查超时
            self.check_timeouts(current_time)

            # 状态机逻辑
            if self.current_state == SystemState.INITIALIZING:
                self.handle_initializing_state()
            elif self.current_state == SystemState.IDLE:
                self.handle_idle_state()
            elif self.current_state == SystemState.STARTING_GAME:
                self.handle_starting_game_state()
            elif self.current_state == SystemState.WAITING_PLAYER_MOVE:
                self.handle_waiting_player_move_state()
            elif self.current_state == SystemState.PROCESSING_PLAYER_MOVE:
                self.handle_processing_player_move_state()
            elif self.current_state == SystemState.AI_THINKING:
                self.handle_ai_thinking_state()
            elif self.current_state == SystemState.EXECUTING_AI_MOVE:
                self.handle_executing_ai_move_state()
            elif self.current_state == SystemState.GAME_OVER:
                self.handle_game_over_state()
            elif self.current_state == SystemState.ERROR:
                self.handle_error_state()

    def handle_initializing_state(self):
        """处理初始化状态"""
        # 检查所有子系统是否就绪
        all_ready = all(
            subsystem.is_active and subsystem.status == 'running'
            for subsystem in self.subsystems.values()
        )

        if all_ready:
            self.transition_to_state(SystemState.IDLE)
            self.get_logger().info("系统初始化完成，进入空闲状态")
        elif time.time() - self.current_operation_start_time > self.startup_timeout:
            self.transition_to_state(SystemState.ERROR)
            self.get_logger().error("系统初始化超时")

    def handle_idle_state(self):
        """处理空闲状态"""
        # 在空闲状态下可以接受新游戏命令
        pass

    def handle_starting_game_state(self):
        """处理开始游戏状态"""
        # 初始化游戏状态
        self.send_game_command({'command': 'start_game'})
        self.transition_to_state(SystemState.WAITING_PLAYER_MOVE)

    def handle_waiting_player_move_state(self):
        """处理等待玩家移动状态"""
        # 持续检测棋盘状态变化
        if self.current_board_state:
            # 检测是否有新的移动
            if self.detect_board_change():
                self.transition_to_state(SystemState.PROCESSING_PLAYER_MOVE)

    def handle_processing_player_move_state(self):
        """处理玩家移动状态"""
        # 验证和处理玩家移动
        if self.validate_player_move():
            self.transition_to_state(SystemState.AI_THINKING)
        else:
            self.get_logger().warning("玩家移动无效，返回等待状态")
            self.transition_to_state(SystemState.WAITING_PLAYER_MOVE)

    def handle_ai_thinking_state(self):
        """处理AI思考状态"""
        # 请求AI移动
        self.request_ai_move()

    def handle_executing_ai_move_state(self):
        """处理执行AI移动状态"""
        # 等待机械臂执行完成
        pass

    def handle_game_over_state(self):
        """处理游戏结束状态"""
        self.game_active = False
        self.get_logger().info("游戏结束")
        # 可以自动重新开始或等待命令
        time.sleep(5)
        self.transition_to_state(SystemState.IDLE)

    def handle_error_state(self):
        """处理错误状态"""
        self.get_logger().error("系统处于错误状态，尝试恢复...")
        # 实施错误恢复策略
        time.sleep(3)
        self.attempt_error_recovery()

    def transition_to_state(self, new_state: SystemState):
        """状态转换"""
        old_state = self.current_state
        self.current_state = new_state
        self.current_operation_start_time = time.time()

        self.get_logger().info(f"状态转换: {old_state.value} -> {new_state.value}")

        # 状态转换时的特殊操作
        if new_state == SystemState.STARTING_GAME:
            self.game_active = True
            self.game_statistics['games_played'] += 1

    def camera_status_callback(self, msg):
        """相机状态回调"""
        self.update_subsystem_status('camera', msg)

    def vision_status_callback(self, msg):
        """视觉状态回调"""
        self.update_subsystem_status('vision', msg)

    def game_status_callback(self, msg):
        """游戏引擎状态回调"""
        self.update_subsystem_status('game_engine', msg)

    def ai_status_callback(self, msg):
        """AI状态回调"""
        self.update_subsystem_status('chess_ai', msg)

    def arm_status_callback(self, msg):
        """机械臂状态回调"""
        self.update_subsystem_status('arm_controller', msg)

    def update_subsystem_status(self, subsystem_name: str, msg: String):
        """更新子系统状态"""
        try:
            status_data = json.loads(msg.data)
            subsystem = self.subsystems[subsystem_name]

            subsystem.status = status_data.get('node_status', 'unknown')
            subsystem.last_update = time.time()
            subsystem.is_active = True

            # 检查错误状态
            if subsystem.status == 'error':
                subsystem.error_count += 1
                if subsystem.error_count >= self.max_errors_per_subsystem:
                    self.handle_subsystem_failure(subsystem_name)

        except (json.JSONDecodeError, KeyError) as e:
            self.get_logger().error(f"解析{subsystem_name}状态失败: {e}")

    def board_state_callback(self, msg):
        """棋盘状态回调"""
        self.current_board_state = msg

    def move_status_callback(self, msg):
        """移动状态回调"""
        try:
            status_data = json.loads(msg.data)

            if status_data.get('status') == 'completed':
                if self.current_state == SystemState.EXECUTING_AI_MOVE:
                    self.game_statistics['moves_made'] += 1
                    self.transition_to_state(SystemState.WAITING_PLAYER_MOVE)
            elif status_data.get('status') == 'failed':
                self.get_logger().error(f"移动执行失败: {status_data.get('message', '')}")
                self.handle_move_error()

        except json.JSONDecodeError as e:
            self.get_logger().error(f"移动状态解析失败: {e}")

    def detect_board_change(self) -> bool:
        """检测棋盘变化"""
        # 实现棋盘变化检测逻辑
        # 这里简化为总是返回False，实际应该比较棋盘状态
        return False

    def validate_player_move(self) -> bool:
        """验证玩家移动"""
        # 实现移动验证逻辑
        return True

    def request_ai_move(self):
        """请求AI移动"""
        try:
            game_state_data = {
                'action': 'get_move',
                'fen': 'rnbqkbnr/pppppppp/8/8/8/8/PPPPPPPP/RNBQKBNR w KQkq - 0 1',  # 起始FEN
                'time_left': 60.0,
                'skill_level': 10
            }

            self.send_game_command(game_state_data)

        except Exception as e:
            self.get_logger().error(f"请求AI移动失败: {e}")

    def send_game_command(self, command_data: Dict):
        """发送游戏命令"""
        msg = String()
        msg.data = json.dumps(command_data)
        self.game_command_pub.publish(msg)

    def check_timeouts(self, current_time: float):
        """检查各种超时"""
        if not self.current_operation_start_time:
            return

        elapsed = current_time - self.current_operation_start_time

        # 移动超时检查
        if self.current_state in [
            SystemState.PROCESSING_PLAYER_MOVE,
            SystemState.AI_THINKING,
            SystemState.EXECUTING_AI_MOVE
        ]:
            if elapsed > self.move_timeout:
                self.get_logger().warning(f"操作超时: {self.current_state.value}")
                self.handle_timeout()

    def handle_timeout(self):
        """处理超时"""
        self.game_statistics['errors_encountered'] += 1
        self.transition_to_state(SystemState.ERROR)

    def handle_move_error(self):
        """处理移动错误"""
        self.game_statistics['errors_encountered'] += 1
        self.transition_to_state(SystemState.ERROR)

    def handle_subsystem_failure(self, subsystem_name: str):
        """处理子系统故障"""
        self.get_logger().error(f"子系统故障: {subsystem_name}")
        self.transition_to_state(SystemState.ERROR)

    def attempt_error_recovery(self):
        """尝试错误恢复"""
        # 实现错误恢复逻辑
        self.get_logger().info("尝试从错误状态恢复...")

        # 检查子系统状态
        failed_subsystems = [
            name for name, subsystem in self.subsystems.items()
            if subsystem.error_count >= self.max_errors_per_subsystem
        ]

        if not failed_subsystems:
            self.transition_to_state(SystemState.IDLE)
            self.get_logger().info("错误恢复成功")
        else:
            self.get_logger().error(f"无法恢复，失败的子系统: {failed_subsystems}")

    def health_check(self):
        """健康检查"""
        current_time = time.time()
        unhealthy_subsystems = []

        for name, subsystem in self.subsystems.items():
            # 检查子系统是否长时间未更新
            if subsystem.is_active and (current_time - subsystem.last_update) > 10.0:
                unhealthy_subsystems.append(name)
                subsystem.is_active = False

        if unhealthy_subsystems:
            self.get_logger().warning(f"检测到不健康的子系统: {unhealthy_subsystems}")

    def publish_system_status(self):
        """发布系统状态"""
        status = {
            'system_state': self.current_state.value,
            'game_active': self.game_active,
            'simulation_mode': self.simulation_mode,
            'subsystems': {
                name: {
                    'status': subsystem.status,
                    'is_active': subsystem.is_active,
                    'error_count': subsystem.error_count,
                    'last_update': subsystem.last_update
                }
                for name, subsystem in self.subsystems.items()
            },
            'statistics': self.game_statistics,
            'timestamp': time.time()
        }

        msg = String()
        msg.data = json.dumps(status)
        self.system_status_pub.publish(msg)

    def start_new_game(self):
        """开始新游戏"""
        with self.state_lock:
            if self.current_state == SystemState.IDLE:
                self.transition_to_state(SystemState.STARTING_GAME)
                self.get_logger().info("开始新游戏")
            else:
                self.get_logger().warning(f"无法开始游戏，当前状态: {self.current_state.value}")

    def stop_game(self):
        """停止游戏"""
        with self.state_lock:
            self.game_active = False
            self.transition_to_state(SystemState.IDLE)
            self.get_logger().info("游戏已停止")

    def emergency_stop(self):
        """紧急停止"""
        with self.state_lock:
            self.game_active = False
            self.transition_to_state(SystemState.ERROR)
            self.get_logger().warning("执行紧急停止")

            # 向所有子系统发送停止命令
            stop_command = {'command': 'emergency_stop'}
            self.send_system_command(stop_command)

    def send_system_command(self, command_data: Dict):
        """发送系统命令"""
        msg = String()
        msg.data = json.dumps(command_data)
        self.system_command_pub.publish(msg)

    def get_system_info(self) -> Dict:
        """获取系统信息"""
        return {
            'current_state': self.current_state.value,
            'uptime': time.time() - self.system_start_time,
            'subsystems_status': {
                name: subsystem.status for name, subsystem in self.subsystems.items()
            },
            'statistics': self.game_statistics
        }


def main(args=None):
    rclpy.init(args=args)
    node = SystemCoordinator()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()