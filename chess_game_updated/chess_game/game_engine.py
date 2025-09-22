#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from chess_interfaces.msg import BoardState, ChessMove, GameStatus
from chess_interfaces.srv import DetectBoard
import numpy as np
import time
import subprocess
import threading
import queue


class GameEngine(Node):
    """象棋游戏引擎节点"""

    def __init__(self):
        super().__init__('game_engine')

        # 声明参数
        self.declare_parameter('difficulty_level', 5)
        self.declare_parameter('max_thinking_time', 10.0)
        self.declare_parameter('auto_start', False)
        self.declare_parameter('human_plays_red', True)

        # 获取参数
        self.difficulty_level = self.get_parameter('difficulty_level').value
        self.max_thinking_time = self.get_parameter('max_thinking_time').value
        self.auto_start = self.get_parameter('auto_start').value
        self.human_plays_red = self.get_parameter('human_plays_red').value

        # 游戏状态
        self.game_state = GameStatus.GAME_IDLE
        self.game_result = GameStatus.RESULT_ONGOING
        self.current_board = None
        self.move_history = []
        self.turn_number = 1
        self.human_turn = self.human_plays_red  # 红方先走
        self.game_start_time = None
        self.ai_thinking_start = None

        # 订阅者
        self.board_sub = self.create_subscription(
            BoardState, 'chess/board_state', self._board_callback, 10)

        # 发布者
        self.status_pub = self.create_publisher(
            GameStatus, 'chess/game_status', 10)
        self.move_pub = self.create_publisher(
            ChessMove, 'chess/ai_move', 10)

        # 服务客户端
        self.detect_client = self.create_client(DetectBoard, 'chess/detect_board')

        # 定时器
        self.status_timer = self.create_timer(1.0, self._publish_status)

        # 象棋引擎相关
        self.engine_queue = queue.Queue()
        self.engine_thread = None
        self.engine_thinking = False

        # 象棋规则和逻辑
        self.chess_rules = ChessRules()

        self.get_logger().info(f'Game engine initialized, difficulty: {self.difficulty_level}')

        if self.auto_start:
            self._start_new_game()

    def _board_callback(self, msg):
        """棋盘状态回调"""
        if not msg.board_detected:
            return

        old_board = self.current_board
        self.current_board = msg

        # 检查是否有新的移动
        if old_board is not None and self._detect_move(old_board, msg):
            if self.human_turn and self.game_state == GameStatus.GAME_WAITING_HUMAN:
                self._process_human_move(old_board, msg)

    def _detect_move(self, old_board, new_board):
        """检测是否有移动发生"""
        if old_board is None or new_board is None:
            return False

        old_state = np.array(old_board.board_state)
        new_state = np.array(new_board.board_state)

        return not np.array_equal(old_state, new_state)

    def _process_human_move(self, old_board, new_board):
        """处理人类玩家移动"""
        try:
            move = self._extract_move(old_board, new_board)
            if move and self.chess_rules.is_valid_move(old_board.board_state, move):
                self.get_logger().info(f'Human move detected: {move.move_description}')
                self.move_history.append(move)
                self.turn_number += 1
                self.human_turn = False

                # 检查游戏状态
                if self._check_game_over():
                    return

                # 轮到AI思考
                self.game_state = GameStatus.GAME_AI_THINKING
                self.ai_thinking_start = time.time()
                self._start_ai_thinking()

            else:
                self.get_logger().warn('Invalid human move detected')

        except Exception as e:
            self.get_logger().error(f'Error processing human move: {str(e)}')

    def _extract_move(self, old_board, new_board):
        """从棋盘变化中提取移动"""
        old_state = np.array(old_board.board_state)
        new_state = np.array(new_board.board_state)

        # 找到差异位置
        diff_positions = np.where(old_state != new_state)[0]

        if len(diff_positions) != 2:
            return None

        from_pos = diff_positions[0]
        to_pos = diff_positions[1]

        # 确定哪个是起始位置，哪个是目标位置
        if old_state[from_pos] != 0 and new_state[from_pos] == 0:
            # from_pos是起始位置
            pass
        else:
            # 交换位置
            from_pos, to_pos = to_pos, from_pos

        # 创建移动消息
        move = ChessMove()
        move.timestamp = self.get_clock().now().to_msg()
        move.from_index = int(from_pos)
        move.to_index = int(to_pos)
        move.piece_type = int(old_state[from_pos])
        move.captured_piece = int(old_state[to_pos])
        move.move_type = ChessMove.CAPTURE_MOVE if move.captured_piece > 0 else ChessMove.NORMAL_MOVE

        # 转换为象棋坐标
        move.from_position = self._index_to_position(from_pos)
        move.to_position = self._index_to_position(to_pos)
        move.move_description = f'{move.from_position}-{move.to_position}'

        return move

    def _index_to_position(self, index):
        """将数组索引转换为象棋坐标"""
        row = index // 9
        col = index % 9
        return f'{chr(ord("a") + col)}{row}'

    def _start_ai_thinking(self):
        """启动AI思考"""
        if self.engine_thread and self.engine_thread.is_alive():
            return

        self.engine_thinking = True
        self.engine_thread = threading.Thread(target=self._ai_thinking_worker)
        self.engine_thread.daemon = True
        self.engine_thread.start()

    def _ai_thinking_worker(self):
        """AI思考工作线程"""
        try:
            # 模拟AI思考过程
            thinking_time = min(self.max_thinking_time,
                               2.0 + self.difficulty_level * 0.5)

            self.get_logger().info(f'AI thinking for {thinking_time:.1f} seconds...')
            time.sleep(thinking_time)

            # 生成AI移动
            ai_move = self._generate_ai_move()
            if ai_move:
                self.move_history.append(ai_move)
                self.move_pub.publish(ai_move)

                self.get_logger().info(f'AI move: {ai_move.move_description}')

                self.turn_number += 1
                self.human_turn = True
                self.game_state = GameStatus.GAME_WAITING_HUMAN

        except Exception as e:
            self.get_logger().error(f'Error in AI thinking: {str(e)}')
            self.game_state = GameStatus.GAME_ERROR

        finally:
            self.engine_thinking = False

    def _generate_ai_move(self):
        """生成AI移动（简化版本）"""
        if self.current_board is None:
            return None

        # 简化的AI逻辑：随机选择一个有效移动
        valid_moves = self.chess_rules.get_valid_moves(
            self.current_board.board_state, not self.human_plays_red)

        if not valid_moves:
            return None

        # 选择第一个有效移动（实际应该使用更智能的算法）
        selected_move = valid_moves[0]

        move = ChessMove()
        move.timestamp = self.get_clock().now().to_msg()
        move.from_index = selected_move['from']
        move.to_index = selected_move['to']
        move.piece_type = selected_move['piece']
        move.move_type = ChessMove.NORMAL_MOVE
        move.from_position = self._index_to_position(selected_move['from'])
        move.to_position = self._index_to_position(selected_move['to'])
        move.move_description = f'AI: {move.from_position}-{move.to_position}'
        move.move_score = selected_move.get('score', 0.0)

        return move

    def _check_game_over(self):
        """检查游戏是否结束"""
        # 简化的游戏结束检查
        if len(self.move_history) > 100:  # 超过100步
            self.game_result = GameStatus.RESULT_DRAW
            self.game_state = GameStatus.GAME_FINISHED
            return True

        return False

    def _start_new_game(self):
        """开始新游戏"""
        self.game_state = GameStatus.GAME_INITIALIZING
        self.game_result = GameStatus.RESULT_ONGOING
        self.move_history = []
        self.turn_number = 1
        self.human_turn = self.human_plays_red
        self.game_start_time = time.time()

        self.get_logger().info('Starting new chess game')

        # 等待棋盘检测
        self.game_state = GameStatus.GAME_WAITING_HUMAN

    def _publish_status(self):
        """发布游戏状态"""
        status = GameStatus()
        status.timestamp = self.get_clock().now().to_msg()
        status.game_state = self.game_state
        status.game_result = self.game_result
        status.human_turn = self.human_turn
        status.turn_number = self.turn_number
        status.moves_count = len(self.move_history)

        if self.move_history:
            status.last_move = self.move_history[-1]

        # 计算思考时间
        current_time = time.time()
        if self.ai_thinking_start and self.engine_thinking:
            status.ai_thinking_time = current_time - self.ai_thinking_start

        if self.game_start_time:
            status.total_game_time = current_time - self.game_start_time

        status.difficulty_level = self.difficulty_level
        status.simulation_mode = True  # 当前在仿真模式
        status.safety_ok = True
        status.engine_status = 'Ready'
        status.arm_status = 'Ready'
        status.camera_status = 'Ready'

        self.status_pub.publish(status)


class ChessRules:
    """象棋规则类"""

    def is_valid_move(self, board_state, move):
        """检查移动是否有效"""
        # 简化的规则检查
        if move.from_index < 0 or move.from_index >= 90:
            return False
        if move.to_index < 0 or move.to_index >= 90:
            return False
        if move.from_index == move.to_index:
            return False

        piece = board_state[move.from_index]
        if piece == 0:
            return False

        return True

    def get_valid_moves(self, board_state, is_black_turn):
        """获取所有有效移动"""
        valid_moves = []

        # 简化版本：只返回几个示例移动
        for i in range(90):
            if board_state[i] > 0:
                # 检查棋子颜色
                piece_is_black = board_state[i] > 7
                if piece_is_black == is_black_turn:
                    # 添加一些可能的移动
                    for j in range(max(0, i-9), min(90, i+10)):
                        if j != i and board_state[j] == 0:
                            valid_moves.append({
                                'from': i,
                                'to': j,
                                'piece': board_state[i],
                                'score': 0.0
                            })
                            if len(valid_moves) >= 10:  # 限制返回数量
                                return valid_moves

        return valid_moves


def main(args=None):
    rclpy.init(args=args)

    try:
        engine = GameEngine()
        rclpy.spin(engine)
    except KeyboardInterrupt:
        pass
    finally:
        if 'engine' in locals():
            engine.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()