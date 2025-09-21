#!/usr/bin/env python3
"""
象棋游戏协调器节点
统一管理游戏流程、状态机和各模块协调
负责整个象棋对弈的流程控制
"""

import rclpy
from rclpy.node import Node
from chess_interfaces.msg import BoardState, ChessMove
from chess_interfaces.srv import DetectBoard, PlanMove, ExecuteMove
from std_msgs.msg import String
import json
import time
import threading
from enum import Enum
from typing import Optional, Dict, List
import chess


class GameState(Enum):
    """游戏状态枚举"""
    IDLE = "idle"
    WAITING_FOR_BOARD = "waiting_for_board"
    DETECTING_BOARD = "detecting_board"
    PLAYER_TURN = "player_turn"
    AI_THINKING = "ai_thinking"
    ROBOT_MOVING = "robot_moving"
    GAME_OVER = "game_over"
    ERROR = "error"


class GameCoordinator(Node):
    """象棋游戏协调器"""

    def __init__(self):
        super().__init__('game_coordinator')

        # 参数声明
        self.declare_parameter('auto_start', False)
        self.declare_parameter('player_color', 'white')  # white/black
        self.declare_parameter('ai_skill_level', 10)
        self.declare_parameter('time_limit_per_move', 300.0)  # 5分钟
        self.declare_parameter('detection_timeout', 10.0)
        self.declare_parameter('move_timeout', 30.0)

        # 获取参数
        self.auto_start = self.get_parameter('auto_start').value
        self.player_color = self.get_parameter('player_color').value
        self.ai_skill_level = self.get_parameter('ai_skill_level').value
        self.time_limit_per_move = self.get_parameter('time_limit_per_move').value
        self.detection_timeout = self.get_parameter('detection_timeout').value
        self.move_timeout = self.get_parameter('move_timeout').value

        # 游戏状态
        self.current_state = GameState.IDLE
        self.game_active = False
        self.current_board_state = None
        self.last_known_fen = chess.STARTING_FEN
        self.move_history = []
        self.game_start_time = None
        self.current_turn_start_time = None

        # 时间控制
        self.player_time_left = self.time_limit_per_move
        self.ai_time_left = self.time_limit_per_move

        # 错误状态
        self.last_error = None
        self.error_count = 0
        self.max_errors = 3

        # 订阅者
        self.board_state_sub = self.create_subscription(
            BoardState, '/chess/board_state', self.board_state_callback, 10)
        self.ai_move_sub = self.create_subscription(
            ChessMove, '/chess/ai_move', self.ai_move_callback, 10)
        self.move_status_sub = self.create_subscription(
            String, '/chess/move_status', self.move_status_callback, 10)
        self.game_command_sub = self.create_subscription(
            String, '/chess/game_command', self.game_command_callback, 10)

        # 发布者
        self.game_state_pub = self.create_publisher(
            String, '/chess/game_state', 10)
        self.move_request_pub = self.create_publisher(
            ChessMove, '/chess/move_request', 10)
        self.status_pub = self.create_publisher(
            String, '/chess/coordinator_status', 10)

        # 服务客户端
        self.detect_board_client = self.create_client(
            DetectBoard, '/chess/detect_board')
        self.plan_move_client = self.create_client(
            PlanMove, '/chess/plan_move')
        self.execute_move_client = self.create_client(
            ExecuteMove, '/chess/execute_move')

        # 定时器
        self.state_timer = self.create_timer(1.0, self.state_machine_tick)
        self.status_timer = self.create_timer(2.0, self.publish_status)

        # 线程锁
        self.state_lock = threading.Lock()

        self.get_logger().info(f"象棋游戏协调器已启动 - 玩家颜色: {self.player_color}")

        # 自动启动
        if self.auto_start:
            self.start_new_game()

    def board_state_callback(self, msg):
        """棋盘状态回调"""
        with self.state_lock:
            self.current_board_state = msg

            # 在等待棋盘状态时更新状态机
            if self.current_state == GameState.WAITING_FOR_BOARD:
                self.transition_to_state(GameState.DETECTING_BOARD)

    def ai_move_callback(self, msg):
        """AI移动回调"""
        with self.state_lock:
            if self.current_state == GameState.AI_THINKING:
                self.get_logger().info(f"收到AI移动: {msg.move_notation}")
                self.execute_robot_move(msg)

    def move_status_callback(self, msg):
        """移动状态回调"""
        try:
            status_data = json.loads(msg.data)

            with self.state_lock:
                if status_data.get('status') == 'completed':
                    if self.current_state == GameState.ROBOT_MOVING:
                        self.on_move_completed()
                elif status_data.get('status') == 'failed':
                    self.handle_move_error(status_data.get('error', '未知错误'))

        except json.JSONDecodeError as e:
            self.get_logger().error(f"移动状态JSON解析失败: {e}")

    def game_command_callback(self, msg):
        """游戏命令回调"""
        try:
            command_data = json.loads(msg.data)
            command = command_data.get('command')

            if command == 'start_game':
                self.start_new_game()
            elif command == 'pause_game':
                self.pause_game()
            elif command == 'resume_game':
                self.resume_game()
            elif command == 'reset_game':
                self.reset_game()
            elif command == 'set_difficulty':
                level = command_data.get('level', 10)
                self.set_ai_difficulty(level)
            elif command == 'force_move':
                self.force_player_move()
            else:
                self.get_logger().warning(f"未知命令: {command}")

        except json.JSONDecodeError as e:
            self.get_logger().error(f"游戏命令JSON解析失败: {e}")

    def state_machine_tick(self):
        """状态机主循环"""
        with self.state_lock:
            if not self.game_active:
                return

            current_time = time.time()

            # 检查超时
            self.check_timeouts(current_time)

            # 状态机逻辑
            if self.current_state == GameState.IDLE:
                self.handle_idle_state()
            elif self.current_state == GameState.WAITING_FOR_BOARD:
                self.handle_waiting_for_board_state()
            elif self.current_state == GameState.DETECTING_BOARD:
                self.handle_detecting_board_state()
            elif self.current_state == GameState.PLAYER_TURN:
                self.handle_player_turn_state()
            elif self.current_state == GameState.AI_THINKING:
                self.handle_ai_thinking_state()
            elif self.current_state == GameState.ROBOT_MOVING:
                self.handle_robot_moving_state()
            elif self.current_state == GameState.GAME_OVER:
                self.handle_game_over_state()
            elif self.current_state == GameState.ERROR:
                self.handle_error_state()

    def handle_idle_state(self):
        """处理空闲状态"""
        if self.game_active:
            self.transition_to_state(GameState.WAITING_FOR_BOARD)

    def handle_waiting_for_board_state(self):
        """处理等待棋盘状态"""
        # 由board_state_callback触发转换
        pass

    def handle_detecting_board_state(self):
        """处理检测棋盘状态"""
        if self.current_board_state:
            # 分析棋盘状态变化
            board_changed = self.analyze_board_change()

            if board_changed:
                self.update_game_state_from_board()

            # 确定下一步状态
            if self.is_player_turn():
                self.transition_to_state(GameState.PLAYER_TURN)
            else:
                self.transition_to_state(GameState.AI_THINKING)

    def handle_player_turn_state(self):
        """处理玩家回合状态"""
        # 等待玩家移动
        # 通过棋盘状态变化检测玩家移动
        pass

    def handle_ai_thinking_state(self):
        """处理AI思考状态"""
        # 发送AI移动请求
        self.request_ai_move()

    def handle_robot_moving_state(self):
        """处理机器人移动状态"""
        # 等待移动完成
        pass

    def handle_game_over_state(self):
        """处理游戏结束状态"""
        self.game_active = False

    def handle_error_state(self):
        """处理错误状态"""
        self.error_count += 1

        if self.error_count >= self.max_errors:
            self.get_logger().error("错误次数过多，停止游戏")
            self.transition_to_state(GameState.GAME_OVER)
        else:
            # 尝试恢复
            self.transition_to_state(GameState.WAITING_FOR_BOARD)

    def transition_to_state(self, new_state: GameState):
        """状态转换"""
        old_state = self.current_state
        self.current_state = new_state
        self.get_logger().info(f"状态转换: {old_state.value} -> {new_state.value}")

        # 状态转换时的操作
        if new_state == GameState.PLAYER_TURN:
            self.current_turn_start_time = time.time()
        elif new_state == GameState.AI_THINKING:
            self.current_turn_start_time = time.time()

    def start_new_game(self):
        """开始新游戏"""
        with self.state_lock:
            self.game_active = True
            self.current_state = GameState.IDLE
            self.last_known_fen = chess.STARTING_FEN
            self.move_history.clear()
            self.game_start_time = time.time()
            self.player_time_left = self.time_limit_per_move
            self.ai_time_left = self.time_limit_per_move
            self.error_count = 0

        self.get_logger().info("开始新游戏")
        self.publish_game_state_update()

    def pause_game(self):
        """暂停游戏"""
        with self.state_lock:
            self.game_active = False
        self.get_logger().info("游戏已暂停")

    def resume_game(self):
        """恢复游戏"""
        with self.state_lock:
            self.game_active = True
        self.get_logger().info("游戏已恢复")

    def reset_game(self):
        """重置游戏"""
        self.start_new_game()

    def set_ai_difficulty(self, level: int):
        """设置AI难度"""
        if 1 <= level <= 20:
            self.ai_skill_level = level
            self.get_logger().info(f"AI难度设置为: {level}")
        else:
            self.get_logger().warning("AI难度必须在1-20之间")

    def force_player_move(self):
        """强制玩家移动（超时处理）"""
        with self.state_lock:
            if self.current_state == GameState.PLAYER_TURN:
                # 转为AI回合
                self.transition_to_state(GameState.AI_THINKING)

    def analyze_board_change(self) -> bool:
        """分析棋盘变化"""
        if not self.current_board_state:
            return False

        # 这里应该实现棋盘状态分析逻辑
        # 比较当前状态与已知状态，检测移动
        return True

    def update_game_state_from_board(self):
        """根据棋盘状态更新游戏状态"""
        # 实现FEN解析和游戏状态更新
        pass

    def is_player_turn(self) -> bool:
        """判断是否是玩家回合"""
        board = chess.Board(self.last_known_fen)
        player_is_white = (self.player_color == 'white')
        return board.turn == player_is_white

    def request_ai_move(self):
        """请求AI移动"""
        try:
            game_state_data = {
                'action': 'get_move',
                'fen': self.last_known_fen,
                'time_left': self.ai_time_left,
                'skill_level': self.ai_skill_level
            }

            msg = String()
            msg.data = json.dumps(game_state_data)
            self.game_state_pub.publish(msg)

        except Exception as e:
            self.get_logger().error(f"请求AI移动失败: {e}")

    def execute_robot_move(self, move_msg: ChessMove):
        """执行机器人移动"""
        try:
            self.transition_to_state(GameState.ROBOT_MOVING)

            # 发布移动请求
            self.move_request_pub.publish(move_msg)

        except Exception as e:
            self.get_logger().error(f"执行机器人移动失败: {e}")
            self.handle_move_error(str(e))

    def on_move_completed(self):
        """移动完成处理"""
        self.get_logger().info("机器人移动完成")

        # 更新游戏状态
        self.transition_to_state(GameState.WAITING_FOR_BOARD)

    def handle_move_error(self, error_msg: str):
        """处理移动错误"""
        self.last_error = error_msg
        self.get_logger().error(f"移动错误: {error_msg}")
        self.transition_to_state(GameState.ERROR)

    def check_timeouts(self, current_time: float):
        """检查超时"""
        if not self.current_turn_start_time:
            return

        elapsed = current_time - self.current_turn_start_time

        if self.current_state == GameState.PLAYER_TURN:
            if elapsed > self.time_limit_per_move:
                self.get_logger().warning("玩家移动超时")
                self.force_player_move()
        elif self.current_state == GameState.AI_THINKING:
            if elapsed > self.time_limit_per_move:
                self.get_logger().warning("AI思考超时")
                self.transition_to_state(GameState.ERROR)
        elif self.current_state == GameState.ROBOT_MOVING:
            if elapsed > self.move_timeout:
                self.get_logger().warning("机器人移动超时")
                self.transition_to_state(GameState.ERROR)

    def publish_game_state_update(self):
        """发布游戏状态更新"""
        game_state = {
            'action': 'state_update',
            'state': self.current_state.value,
            'fen': self.last_known_fen,
            'move_history': [str(move) for move in self.move_history],
            'player_color': self.player_color,
            'player_time_left': self.player_time_left,
            'ai_time_left': self.ai_time_left,
            'game_active': self.game_active
        }

        msg = String()
        msg.data = json.dumps(game_state)
        self.game_state_pub.publish(msg)

    def publish_status(self):
        """发布协调器状态"""
        status = {
            'node_status': 'running',
            'current_state': self.current_state.value,
            'game_active': self.game_active,
            'error_count': self.error_count,
            'last_error': self.last_error,
            'uptime': time.time() - self.game_start_time if self.game_start_time else 0,
            'timestamp': time.time()
        }

        msg = String()
        msg.data = json.dumps(status)
        self.status_pub.publish(msg)

    def get_game_statistics(self) -> Dict:
        """获取游戏统计信息"""
        return {
            'total_moves': len(self.move_history),
            'game_duration': time.time() - self.game_start_time if self.game_start_time else 0,
            'player_time_used': self.time_limit_per_move - self.player_time_left,
            'ai_time_used': self.time_limit_per_move - self.ai_time_left,
            'error_count': self.error_count
        }


def main(args=None):
    rclpy.init(args=args)
    node = GameCoordinator()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()