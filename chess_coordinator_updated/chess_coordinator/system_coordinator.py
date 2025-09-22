#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from chess_interfaces.msg import BoardState, ChessMove, GameStatus
from chess_interfaces.srv import DetectBoard, PlanMove, ExecuteMove
from std_msgs.msg import String
import time
import threading
from enum import Enum


class SystemState(Enum):
    """系统状态枚举"""
    IDLE = 0
    INITIALIZING = 1
    DETECTING_BOARD = 2
    WAITING_HUMAN = 3
    PROCESSING_HUMAN_MOVE = 4
    AI_THINKING = 5
    PLANNING_MOVE = 6
    EXECUTING_MOVE = 7
    GAME_OVER = 8
    ERROR = 9


class SystemCoordinator(Node):
    """系统状态机和协调控制器"""

    def __init__(self):
        super().__init__('system_coordinator')

        # 声明参数
        self.declare_parameter('auto_start', True)
        self.declare_parameter('detection_interval', 2.0)
        self.declare_parameter('move_timeout', 30.0)
        self.declare_parameter('safety_check_interval', 1.0)

        # 获取参数
        self.auto_start = self.get_parameter('auto_start').value
        self.detection_interval = self.get_parameter('detection_interval').value
        self.move_timeout = self.get_parameter('move_timeout').value
        self.safety_check_interval = self.get_parameter('safety_check_interval').value

        # 系统状态
        self.system_state = SystemState.IDLE
        self.last_state_change = time.time()
        self.error_message = ""

        # 数据存储
        self.current_board = None
        self.current_game_status = None
        self.last_ai_move = None
        self.pending_move = None

        # 订阅者
        self.board_sub = self.create_subscription(
            BoardState, 'chess/board_state', self._board_callback, 10)
        self.game_status_sub = self.create_subscription(
            GameStatus, 'chess/game_status', self._game_status_callback, 10)
        self.ai_move_sub = self.create_subscription(
            ChessMove, 'chess/ai_move', self._ai_move_callback, 10)

        # 发布者
        self.system_status_pub = self.create_publisher(
            String, 'chess/system_status', 10)
        self.command_pub = self.create_publisher(
            String, 'chess/system_command', 10)

        # 服务客户端
        self.detect_client = self.create_client(DetectBoard, 'chess/detect_board')
        self.plan_client = self.create_client(PlanMove, 'chess/plan_move')
        self.execute_client = self.create_client(ExecuteMove, 'chess/execute_move')

        # 定时器
        self.main_timer = self.create_timer(0.1, self._main_loop)
        self.status_timer = self.create_timer(1.0, self._publish_status)
        self.detection_timer = self.create_timer(
            self.detection_interval, self._trigger_detection)

        # 线程锁
        self.state_lock = threading.Lock()

        self.get_logger().info('System coordinator initialized')

        if self.auto_start:
            self._change_state(SystemState.INITIALIZING)

    def _board_callback(self, msg):
        """棋盘状态回调"""
        self.current_board = msg

    def _game_status_callback(self, msg):
        """游戏状态回调"""
        self.current_game_status = msg

    def _ai_move_callback(self, msg):
        """AI移动回调"""
        self.last_ai_move = msg
        if self.system_state == SystemState.AI_THINKING:
            self.pending_move = msg
            self._change_state(SystemState.PLANNING_MOVE)

    def _main_loop(self):
        """主控制循环"""
        with self.state_lock:
            try:
                if self.system_state == SystemState.IDLE:
                    self._handle_idle_state()
                elif self.system_state == SystemState.INITIALIZING:
                    self._handle_initializing_state()
                elif self.system_state == SystemState.DETECTING_BOARD:
                    self._handle_detecting_state()
                elif self.system_state == SystemState.WAITING_HUMAN:
                    self._handle_waiting_human_state()
                elif self.system_state == SystemState.PROCESSING_HUMAN_MOVE:
                    self._handle_processing_human_state()
                elif self.system_state == SystemState.AI_THINKING:
                    self._handle_ai_thinking_state()
                elif self.system_state == SystemState.PLANNING_MOVE:
                    self._handle_planning_state()
                elif self.system_state == SystemState.EXECUTING_MOVE:
                    self._handle_executing_state()
                elif self.system_state == SystemState.GAME_OVER:
                    self._handle_game_over_state()
                elif self.system_state == SystemState.ERROR:
                    self._handle_error_state()

            except Exception as e:
                self.get_logger().error(f'Error in main loop: {str(e)}')
                self._change_state(SystemState.ERROR, str(e))

    def _handle_idle_state(self):
        """处理空闲状态"""
        # 等待启动命令或自动启动
        pass

    def _handle_initializing_state(self):
        """处理初始化状态"""
        self.get_logger().info('Initializing system...')

        # 检查所有服务是否可用
        if self._check_services_available():
            self._change_state(SystemState.DETECTING_BOARD)
        else:
            # 等待服务启动
            if time.time() - self.last_state_change > 10.0:
                self._change_state(SystemState.ERROR, "Services not available after 10 seconds")

    def _handle_detecting_state(self):
        """处理检测状态"""
        if self.current_board is not None and self.current_board.board_detected:
            self.get_logger().info('Board detected, waiting for human move')
            self._change_state(SystemState.WAITING_HUMAN)

    def _handle_waiting_human_state(self):
        """处理等待人类移动状态"""
        if self.current_game_status is None:
            return

        if self.current_game_status.game_state == GameStatus.GAME_AI_THINKING:
            self._change_state(SystemState.AI_THINKING)
        elif self.current_game_status.game_state == GameStatus.GAME_FINISHED:
            self._change_state(SystemState.GAME_OVER)

    def _handle_processing_human_state(self):
        """处理人类移动状态"""
        # 验证人类移动的合法性
        # 简化版本：直接转到AI思考
        self._change_state(SystemState.AI_THINKING)

    def _handle_ai_thinking_state(self):
        """处理AI思考状态"""
        if self.pending_move is not None:
            self._change_state(SystemState.PLANNING_MOVE)
        elif self.current_game_status and self.current_game_status.game_state != GameStatus.GAME_AI_THINKING:
            # AI思考完成
            if time.time() - self.last_state_change > self.move_timeout:
                self._change_state(SystemState.ERROR, "AI thinking timeout")

    def _handle_planning_state(self):
        """处理移动规划状态"""
        if self.pending_move is None:
            self._change_state(SystemState.ERROR, "No pending move to plan")
            return

        # 异步调用规划服务
        self._plan_move_async(self.pending_move)
        self._change_state(SystemState.EXECUTING_MOVE)

    def _handle_executing_state(self):
        """处理执行移动状态"""
        # 等待移动执行完成
        if time.time() - self.last_state_change > self.move_timeout:
            self._change_state(SystemState.ERROR, "Move execution timeout")

    def _handle_game_over_state(self):
        """处理游戏结束状态"""
        self.get_logger().info('Game over')
        # 可以在这里添加游戏结束后的处理逻辑

    def _handle_error_state(self):
        """处理错误状态"""
        self.get_logger().error(f'System in error state: {self.error_message}')
        # 错误恢复逻辑
        if time.time() - self.last_state_change > 10.0:
            self.get_logger().info('Attempting to recover from error...')
            self.error_message = ""
            self._change_state(SystemState.IDLE)

    def _change_state(self, new_state, error_msg=""):
        """改变系统状态"""
        old_state = self.system_state
        self.system_state = new_state
        self.last_state_change = time.time()

        if new_state == SystemState.ERROR:
            self.error_message = error_msg

        self.get_logger().info(f'State changed: {old_state.name} -> {new_state.name}')

    def _check_services_available(self):
        """检查所有必需的服务是否可用"""
        services = [
            (self.detect_client, 'detect_board'),
            (self.plan_client, 'plan_move'),
            (self.execute_client, 'execute_move')
        ]

        for client, name in services:
            if not client.wait_for_service(timeout_sec=1.0):
                self.get_logger().warn(f'Service {name} not available')
                return False

        return True

    def _trigger_detection(self):
        """触发棋盘检测"""
        if self.system_state in [SystemState.WAITING_HUMAN, SystemState.DETECTING_BOARD]:
            self._detect_board_async()

    def _detect_board_async(self):
        """异步检测棋盘"""
        if not self.detect_client.service_is_ready():
            return

        request = DetectBoard.Request()
        request.force_detection = False
        request.timeout = 5.0
        request.use_depth = True
        request.detect_aruco = True
        request.detection_mode = DetectBoard.Request.ACCURATE_MODE

        future = self.detect_client.call_async(request)
        future.add_done_callback(self._detection_callback)

    def _detection_callback(self, future):
        """检测结果回调"""
        try:
            response = future.result()
            if response.success:
                self.get_logger().debug(f'Board detection successful, confidence: {response.confidence}')
            else:
                self.get_logger().warn(f'Board detection failed: {response.error_message}')
        except Exception as e:
            self.get_logger().error(f'Detection service call failed: {str(e)}')

    def _plan_move_async(self, move):
        """异步规划移动"""
        if not self.plan_client.service_is_ready():
            return

        request = PlanMove.Request()
        request.target_move = move
        request.planning_mode = PlanMove.Request.SAFE_PLANNING
        request.velocity_scaling = 0.3
        request.acceleration_scaling = 0.3
        request.avoid_pieces = True
        request.pick_height = 0.05
        request.place_height = 0.02
        request.safe_height = 0.1
        request.planning_timeout = 10.0

        future = self.plan_client.call_async(request)
        future.add_done_callback(self._planning_callback)

    def _planning_callback(self, future):
        """规划结果回调"""
        try:
            response = future.result()
            if response.success:
                self.get_logger().info(f'Move planning successful, execution time: {response.execution_time}s')
                self._execute_move_async(self.pending_move, response.joint_trajectory)
            else:
                self.get_logger().error(f'Move planning failed: {response.error_message}')
                self._change_state(SystemState.ERROR, "Move planning failed")
        except Exception as e:
            self.get_logger().error(f'Planning service call failed: {str(e)}')
            self._change_state(SystemState.ERROR, "Planning service failed")

    def _execute_move_async(self, move, trajectory):
        """异步执行移动"""
        if not self.execute_client.service_is_ready():
            return

        request = ExecuteMove.Request()
        request.chess_move = move
        request.trajectory = trajectory
        request.execution_mode = ExecuteMove.Request.SIMULATION_MODE
        request.velocity_scaling = 0.3
        request.enable_collision_detection = True
        request.enable_force_control = False
        request.execution_timeout = 30.0
        request.wait_for_completion = True

        future = self.execute_client.call_async(request)
        future.add_done_callback(self._execution_callback)

    def _execution_callback(self, future):
        """执行结果回调"""
        try:
            response = future.result()
            if response.success:
                self.get_logger().info(f'Move execution successful, duration: {response.actual_duration}s')
                self.pending_move = None
                self._change_state(SystemState.WAITING_HUMAN)
            else:
                self.get_logger().error(f'Move execution failed: {response.error_message}')
                self._change_state(SystemState.ERROR, "Move execution failed")
        except Exception as e:
            self.get_logger().error(f'Execution service call failed: {str(e)}')
            self._change_state(SystemState.ERROR, "Execution service failed")

    def _publish_status(self):
        """发布系统状态"""
        status_msg = String()
        status_data = {
            'system_state': self.system_state.name,
            'uptime': time.time() - self.last_state_change,
            'error_message': self.error_message,
            'services_ready': self._check_services_available()
        }

        status_msg.data = str(status_data)
        self.system_status_pub.publish(status_msg)


def main(args=None):
    rclpy.init(args=args)

    try:
        coordinator = SystemCoordinator()
        rclpy.spin(coordinator)
    except KeyboardInterrupt:
        pass
    finally:
        if 'coordinator' in locals():
            coordinator.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()