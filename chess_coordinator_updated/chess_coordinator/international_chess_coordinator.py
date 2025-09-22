#!/usr/bin/env python3

"""
国际象棋游戏协调器
负责协调引擎、视觉、机械臂等各个节点，实现完整的人机对弈流程
"""

import rclpy
from rclpy.node import Node
import chess
import time
import threading
from enum import Enum
from typing import Optional, Dict, List

from chess_interfaces.msg import BoardState, ChessMove, GameStatus
from chess_interfaces.srv import GetEngineMove, ValidateMove, DetectBoard
from std_msgs.msg import String


class ChessGameState(Enum):
    """国际象棋游戏状态枚举"""
    IDLE = 0
    INITIALIZING = 1
    WAITING_FOR_PLAYER = 2
    VALIDATING_HUMAN_MOVE = 3
    ENGINE_THINKING = 4
    PLANNING_ENGINE_MOVE = 5
    EXECUTING_ENGINE_MOVE = 6
    GAME_OVER = 7
    ERROR = 8


class InternationalChessCoordinator(Node):
    """国际象棋游戏协调器节点"""

    def __init__(self):
        super().__init__('国际象棋协调器')

        # 声明参数
        self.declare_parameter('auto_start', False)
        self.declare_parameter('human_plays_white', True)
        self.declare_parameter('simulation_mode', True)
        self.declare_parameter('move_timeout', 300.0)  # 5分钟思考时间
        self.declare_parameter('board_detection_interval', 2.0)
        self.declare_parameter('engine_depth', 15)
        self.declare_parameter('engine_skill_level', 10)
        self.declare_parameter('debug_mode', False)

        # 获取参数
        self.auto_start = self.get_parameter('auto_start').value
        self.human_plays_white = self.get_parameter('human_plays_white').value
        self.simulation_mode = self.get_parameter('simulation_mode').value
        self.move_timeout = self.get_parameter('move_timeout').value
        self.detection_interval = self.get_parameter('board_detection_interval').value
        self.engine_depth = self.get_parameter('engine_depth').value
        self.engine_skill_level = self.get_parameter('engine_skill_level').value
        self.debug_mode = self.get_parameter('debug_mode').value

        # 游戏状态
        self.game_state = ChessGameState.IDLE
        self.chess_board = chess.Board()
        self.game_start_time = None
        self.move_count = 0
        self.last_state_change = time.time()
        self.error_message = ""

        # 移动历史
        self.move_history = []
        self.captured_pieces = {'white': [], 'black': []}

        # 时间统计
        self.thinking_time = {'human': 0.0, 'engine': 0.0}
        self.move_start_time = None
        self.engine_thinking_start_time = None

        # 当前状态追踪
        self.current_board_state = None
        self.last_human_move = None
        self.current_engine_move = None
        self.pending_validation = False

        # 订阅棋盘状态
        self.board_subscription = self.create_subscription(
            BoardState, '棋盘状态', self._board_state_callback, 10
        )

        # 发布游戏状态
        self.game_status_publisher = self.create_publisher(
            GameStatus, '游戏状态', 10
        )

        # 发布引擎移动
        self.engine_move_publisher = self.create_publisher(
            ChessMove, '引擎移动', 10
        )

        # 服务客户端
        self.engine_client = self.create_client(GetEngineMove, '获取引擎移动')
        self.validate_client = self.create_client(ValidateMove, '验证移动')
        self.detect_client = self.create_client(DetectBoard, '检测棋盘')

        # 主控制循环定时器
        self.control_timer = self.create_timer(0.1, self._main_control_loop)

        # 状态发布定时器
        self.status_timer = self.create_timer(1.0, self._publish_game_status)

        # 棋盘检测定时器
        self.detection_timer = self.create_timer(
            self.detection_interval, self._request_board_detection
        )

        # 线程锁
        self.state_lock = threading.Lock()

        # 异步调用追踪
        self.pending_futures = {}

        self.get_logger().info(
            f'国际象棋协调器已启动 - '
            f'人类执{"白" if self.human_plays_white else "黑"}棋, '
            f'{"仿真" if self.simulation_mode else "真实"}模式'
        )

        if self.auto_start:
            self._initialize_game()

    def _board_state_callback(self, msg: BoardState):
        """棋盘状态回调"""
        self.current_board_state = msg

        if (msg.board_detected and
            msg.fen_notation and
            self.game_state == ChessGameState.WAITING_FOR_PLAYER):

            # 检查是否有新的人类移动
            self._check_for_human_move(msg)

    def _main_control_loop(self):
        """主控制循环"""
        with self.state_lock:
            try:
                current_time = time.time()

                if self.game_state == ChessGameState.IDLE:
                    pass  # 等待游戏开始

                elif self.game_state == ChessGameState.INITIALIZING:
                    self._handle_initializing()

                elif self.game_state == ChessGameState.WAITING_FOR_PLAYER:
                    self._handle_waiting_for_player()

                elif self.game_state == ChessGameState.VALIDATING_HUMAN_MOVE:
                    self._handle_validating_human_move()

                elif self.game_state == ChessGameState.ENGINE_THINKING:
                    self._handle_engine_thinking()

                elif self.game_state == ChessGameState.PLANNING_ENGINE_MOVE:
                    self._handle_planning_engine_move()

                elif self.game_state == ChessGameState.EXECUTING_ENGINE_MOVE:
                    self._handle_executing_engine_move()

                elif self.game_state == ChessGameState.GAME_OVER:
                    self._handle_game_over()

                elif self.game_state == ChessGameState.ERROR:
                    self._handle_error_state()

                # 检查超时
                self._check_timeouts(current_time)

            except Exception as e:
                self.get_logger().error(f'主控制循环错误: {e}')
                self._change_state(ChessGameState.ERROR, f'控制循环异常: {e}')

    def _handle_initializing(self):
        """处理初始化状态"""
        # 检查所有服务是否可用
        services_ready = (
            self.engine_client.wait_for_service(timeout_sec=1.0) and
            self.validate_client.wait_for_service(timeout_sec=1.0) and
            self.detect_client.wait_for_service(timeout_sec=1.0)
        )

        if services_ready:
            self.get_logger().info('所有服务已就绪，开始游戏')
            self._start_new_game()
        else:
            # 等待服务，如果超时则报错
            if time.time() - self.last_state_change > 30.0:
                self._change_state(ChessGameState.ERROR, '等待服务超时')

    def _handle_waiting_for_player(self):
        """处理等待玩家移动状态"""
        if self.chess_board.is_game_over():
            self._change_state(ChessGameState.GAME_OVER)
            return

        # 检查是否轮到引擎
        is_engine_turn = (
            (self.chess_board.turn == chess.WHITE and not self.human_plays_white) or
            (self.chess_board.turn == chess.BLACK and self.human_plays_white)
        )

        if is_engine_turn:
            self._change_state(ChessGameState.ENGINE_THINKING)
            self.engine_thinking_start_time = time.time()

    def _handle_validating_human_move(self):
        """处理验证人类移动状态"""
        if not self.pending_validation:
            self._change_state(ChessGameState.WAITING_FOR_PLAYER)

    def _handle_engine_thinking(self):
        """处理引擎思考状态"""
        if not hasattr(self, 'engine_future'):
            # 发起引擎计算请求
            self._request_engine_move()

        elif self.engine_future.done():
            # 引擎计算完成
            self._process_engine_response()

    def _handle_planning_engine_move(self):
        """处理规划引擎移动状态"""
        # 在仿真模式下，直接执行移动
        if self.simulation_mode and self.current_engine_move:
            self._execute_engine_move_sim()
        # 真实模式下需要路径规划
        else:
            # TODO: 实现真实机械臂路径规划
            pass

    def _handle_executing_engine_move(self):
        """处理执行引擎移动状态"""
        if not hasattr(self, 'execution_start_time'):
            self.execution_start_time = time.time()

        # 在仿真模式下，模拟执行时间
        if self.simulation_mode:
            if time.time() - self.execution_start_time > 2.0:  # 模拟2秒执行时间
                self._complete_engine_move()

    def _handle_game_over(self):
        """处理游戏结束状态"""
        if not hasattr(self, 'game_over_logged'):
            self._log_game_result()
            self.game_over_logged = True

    def _handle_error_state(self):
        """处理错误状态"""
        # 错误恢复逻辑
        if time.time() - self.last_state_change > 10.0:
            self.get_logger().info('尝试从错误状态恢复...')
            self.error_message = ""
            self._change_state(ChessGameState.IDLE)

    def _check_for_human_move(self, board_state: BoardState):
        """检查是否有新的人类移动"""
        if not board_state.fen_notation:
            return

        try:
            new_board = chess.Board(board_state.fen_notation)

            # 比较当前棋盘和新棋盘状态
            if new_board.fen() != self.chess_board.fen():
                # 检测到棋盘变化，验证是否是合法移动
                self._validate_human_move(board_state.fen_notation)

        except ValueError as e:
            self.get_logger().warn(f'无效的FEN格式: {board_state.fen_notation} - {e}')

    def _validate_human_move(self, new_fen: str):
        """验证人类移动"""
        if self.pending_validation:
            return

        # 找出移动
        try:
            new_board = chess.Board(new_fen)
            # 简化：假设只有一步移动差异
            # 实际应用中需要更复杂的差异检测算法

            # 暂时直接接受新位置
            self.chess_board = new_board
            self.move_count += 1

            # 记录人类思考时间
            if self.move_start_time:
                thinking_time = time.time() - self.move_start_time
                self.thinking_time['human'] += thinking_time

            self.get_logger().info(f'人类移动已接受，轮到{"白方" if self.chess_board.turn == chess.WHITE else "黑方"}')

            self._change_state(ChessGameState.WAITING_FOR_PLAYER)

        except Exception as e:
            self.get_logger().error(f'验证人类移动失败: {e}')

    def _request_engine_move(self):
        """请求引擎移动"""
        if not self.engine_client.service_is_ready():
            return

        request = GetEngineMove.Request()
        request.fen_position = self.chess_board.fen()
        request.depth = self.engine_depth
        request.time_limit = 5.0
        request.skill_level = self.engine_skill_level
        request.multipv = 1

        self.engine_future = self.engine_client.call_async(request)
        self.get_logger().info('引擎正在思考...')

    def _process_engine_response(self):
        """处理引擎响应"""
        try:
            result = self.engine_future.result()

            if result.success:
                self.current_engine_move = result.best_move

                # 记录引擎思考时间
                if self.engine_thinking_start_time:
                    thinking_time = time.time() - self.engine_thinking_start_time
                    self.thinking_time['engine'] += thinking_time

                self.get_logger().info(
                    f'引擎推荐移动: {result.best_move_san} '
                    f'(评估: {result.evaluation:.2f})'
                )

                # 发布引擎移动
                self.engine_move_publisher.publish(result.best_move)

                self._change_state(ChessGameState.PLANNING_ENGINE_MOVE)

            else:
                self.get_logger().error(f'引擎计算失败: {result.error_message}')
                self._change_state(ChessGameState.ERROR, '引擎计算失败')

        except Exception as e:
            self.get_logger().error(f'处理引擎响应失败: {e}')
            self._change_state(ChessGameState.ERROR, f'引擎响应处理异常: {e}')

        finally:
            if hasattr(self, 'engine_future'):
                delattr(self, 'engine_future')

    def _execute_engine_move_sim(self):
        """在仿真模式下执行引擎移动"""
        if not self.current_engine_move:
            return

        try:
            # 解析UCI移动
            move = chess.Move.from_uci(self.current_engine_move.uci_move)

            # 验证移动合法性
            if move in self.chess_board.legal_moves:
                self.chess_board.push(move)
                self.move_count += 1

                # 添加到移动历史
                self.move_history.append({
                    'move': self.current_engine_move.algebraic_notation,
                    'fen': self.chess_board.fen(),
                    'timestamp': time.time(),
                    'player': 'engine'
                })

                self.get_logger().info(f'引擎移动已执行: {self.current_engine_move.algebraic_notation}')
                self._change_state(ChessGameState.EXECUTING_ENGINE_MOVE)

            else:
                self.get_logger().error(f'引擎返回非法移动: {self.current_engine_move.uci_move}')
                self._change_state(ChessGameState.ERROR, '引擎非法移动')

        except Exception as e:
            self.get_logger().error(f'执行引擎移动失败: {e}')
            self._change_state(ChessGameState.ERROR, f'移动执行异常: {e}')

    def _complete_engine_move(self):
        """完成引擎移动"""
        self.current_engine_move = None

        if hasattr(self, 'execution_start_time'):
            delattr(self, 'execution_start_time')

        # 检查游戏是否结束
        if self.chess_board.is_game_over():
            self._change_state(ChessGameState.GAME_OVER)
        else:
            # 轮到人类
            self.move_start_time = time.time()
            self._change_state(ChessGameState.WAITING_FOR_PLAYER)

    def _request_board_detection(self):
        """请求棋盘检测"""
        if (self.game_state in [ChessGameState.WAITING_FOR_PLAYER] and
            self.detect_client.service_is_ready()):

            request = DetectBoard.Request()
            request.force_detection = False
            request.timeout = 3.0
            request.use_depth = True
            request.detection_mode = DetectBoard.Request.FAST_MODE

            future = self.detect_client.call_async(request)
            # 不等待结果，让棋盘状态回调处理

    def _publish_game_status(self):
        """发布游戏状态"""
        status = GameStatus()
        status.timestamp = self.get_clock().now().to_msg()

        # 状态映射
        status_mapping = {
            ChessGameState.IDLE: GameStatus.GAME_IDLE,
            ChessGameState.INITIALIZING: GameStatus.GAME_INITIALIZING,
            ChessGameState.WAITING_FOR_PLAYER: GameStatus.GAME_WAITING_HUMAN,
            ChessGameState.VALIDATING_HUMAN_MOVE: GameStatus.GAME_HUMAN_THINKING,
            ChessGameState.ENGINE_THINKING: GameStatus.GAME_AI_THINKING,
            ChessGameState.PLANNING_ENGINE_MOVE: GameStatus.GAME_EXECUTING_MOVE,
            ChessGameState.EXECUTING_ENGINE_MOVE: GameStatus.GAME_EXECUTING_MOVE,
            ChessGameState.GAME_OVER: GameStatus.GAME_FINISHED,
            ChessGameState.ERROR: GameStatus.GAME_ERROR
        }

        status.game_state = status_mapping.get(self.game_state, GameStatus.GAME_ERROR)
        status.human_turn = (
            (self.chess_board.turn == chess.WHITE and self.human_plays_white) or
            (self.chess_board.turn == chess.BLACK and not self.human_plays_white)
        )

        status.turn_number = self.chess_board.fullmove_number
        status.moves_count = self.move_count

        if self.current_engine_move:
            status.last_move = self.current_engine_move

        status.ai_thinking_time = self.thinking_time['engine']
        status.human_thinking_time = self.thinking_time['human']

        if self.game_start_time:
            status.total_game_time = time.time() - self.game_start_time

        status.error_message = self.error_message
        status.simulation_mode = self.simulation_mode

        # 发布状态
        self.game_status_publisher.publish(status)

    def _check_timeouts(self, current_time: float):
        """检查各种超时情况"""
        state_duration = current_time - self.last_state_change

        # 移动超时检查
        if (self.game_state in [
            ChessGameState.WAITING_FOR_PLAYER,
            ChessGameState.ENGINE_THINKING,
            ChessGameState.VALIDATING_HUMAN_MOVE
        ] and state_duration > self.move_timeout):

            self.get_logger().warn(f'状态 {self.game_state.name} 超时')
            self._change_state(ChessGameState.ERROR, '移动超时')

    def _change_state(self, new_state: ChessGameState, error_msg: str = ""):
        """改变游戏状态"""
        if new_state == self.game_state:
            return

        old_state = self.game_state
        self.game_state = new_state
        self.last_state_change = time.time()

        if new_state == ChessGameState.ERROR:
            self.error_message = error_msg

        self.get_logger().info(f'状态变更: {old_state.name} -> {new_state.name}')

        if error_msg:
            self.get_logger().error(f'错误信息: {error_msg}')

    def _initialize_game(self):
        """初始化游戏"""
        self._change_state(ChessGameState.INITIALIZING)

    def _start_new_game(self):
        """开始新游戏"""
        self.chess_board.reset()
        self.move_count = 0
        self.move_history.clear()
        self.captured_pieces = {'white': [], 'black': []}
        self.thinking_time = {'human': 0.0, 'engine': 0.0}
        self.game_start_time = time.time()

        self.get_logger().info(
            f'新游戏开始！人类执{"白" if self.human_plays_white else "黑"}棋'
        )

        if self.human_plays_white:
            self.move_start_time = time.time()
            self._change_state(ChessGameState.WAITING_FOR_PLAYER)
        else:
            self._change_state(ChessGameState.ENGINE_THINKING)

    def _log_game_result(self):
        """记录游戏结果"""
        result_msg = "游戏结束！"

        if self.chess_board.is_checkmate():
            winner = "白方" if self.chess_board.turn == chess.BLACK else "黑方"
            result_msg += f" {winner}获胜（将死）"
        elif self.chess_board.is_stalemate():
            result_msg += " 和棋（僵局）"
        elif self.chess_board.is_insufficient_material():
            result_msg += " 和棋（子力不足）"
        elif self.chess_board.is_seventyfive_moves():
            result_msg += " 和棋（75回合规则）"
        elif self.chess_board.is_fivefold_repetition():
            result_msg += " 和棋（五次重复）"

        total_time = time.time() - self.game_start_time if self.game_start_time else 0
        result_msg += f"\n游戏时长: {total_time/60:.1f}分钟"
        result_msg += f"\n移动总数: {self.move_count}"
        result_msg += f"\n人类思考时间: {self.thinking_time['human']:.1f}秒"
        result_msg += f"\n引擎思考时间: {self.thinking_time['engine']:.1f}秒"

        self.get_logger().info(result_msg)

    # 公共接口方法
    def start_new_game(self):
        """公共接口：开始新游戏"""
        with self.state_lock:
            self._start_new_game()

    def reset_game(self):
        """公共接口：重置游戏"""
        with self.state_lock:
            self._change_state(ChessGameState.IDLE)
            self.error_message = ""

    def get_current_fen(self) -> str:
        """获取当前FEN位置"""
        return self.chess_board.fen()

    def get_move_history(self) -> List[Dict]:
        """获取移动历史"""
        return self.move_history.copy()


def main(args=None):
    rclpy.init(args=args)

    try:
        coordinator = InternationalChessCoordinator()
        rclpy.spin(coordinator)
    except KeyboardInterrupt:
        pass
    finally:
        if 'coordinator' in locals():
            coordinator.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()