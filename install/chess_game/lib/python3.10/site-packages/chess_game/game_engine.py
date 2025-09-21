#!/usr/bin/env python3
"""
象棋游戏引擎节点
管理游戏状态、规则验证和移动处理
使用python-chess库处理国际象棋逻辑
"""

import rclpy
from rclpy.node import Node
from chess_interfaces.msg import BoardState, ChessMove
from chess_interfaces.srv import DetectBoard
from std_msgs.msg import Header, String
import chess
import chess.pgn
import numpy as np
from typing import Optional, List, Dict, Tuple
import json
import time


class GameEngine(Node):
    """象棋游戏引擎节点"""

    def __init__(self):
        super().__init__('game_engine')

        # 参数声明
        self.declare_parameter('auto_start', False)
        self.declare_parameter('game_mode', 'human_vs_ai')  # human_vs_ai, ai_vs_ai, human_vs_human
        self.declare_parameter('ai_color', 'black')  # white, black
        self.declare_parameter('time_control', 600.0)  # 10分钟
        self.declare_parameter('skill_level', 15)  # 1-20
        self.declare_parameter('game_log_enabled', True)

        # 获取参数
        self.auto_start = self.get_parameter('auto_start').value
        self.game_mode = self.get_parameter('game_mode').value
        self.ai_color = self.get_parameter('ai_color').value
        self.time_control = self.get_parameter('time_control').value
        self.skill_level = self.get_parameter('skill_level').value
        self.game_log_enabled = self.get_parameter('game_log_enabled').value

        # 游戏状态
        self.board = chess.Board()  # python-chess棋盘
        self.game_started = False
        self.game_over = False
        self.current_player = 'white'
        self.move_history = []
        self.start_time = None
        self.last_move_time = None

        # 时间控制
        self.white_time_left = self.time_control
        self.black_time_left = self.time_control

        # 位置映射（ROS索引到python-chess方格）
        self.ros_to_chess_mapping = self.create_position_mapping()
        self.chess_to_ros_mapping = {v: k for k, v in self.ros_to_chess_mapping.items()}

        # 棋子映射
        self.piece_mapping = {
            chess.PAWN: 1, chess.ROOK: 2, chess.KNIGHT: 3,
            chess.BISHOP: 4, chess.QUEEN: 5, chess.KING: 6
        }

        # 订阅者
        self.board_state_sub = self.create_subscription(
            BoardState, '/chess/final_board_state', self.board_state_callback, 10)

        # 发布者
        self.game_state_pub = self.create_publisher(
            String, '/chess/game_state', 10)
        self.move_request_pub = self.create_publisher(
            ChessMove, '/chess/move_request', 10)
        self.game_status_pub = self.create_publisher(
            String, '/chess/game_status', 10)

        # 定时器
        self.status_timer = self.create_timer(1.0, self.publish_status)

        # 游戏日志
        self.game_log = []

        self.get_logger().info(f"游戏引擎已启动 - 模式: {self.game_mode}, AI颜色: {self.ai_color}")

        # 自动开始游戏
        if self.auto_start:
            self.start_new_game()

    def create_position_mapping(self) -> Dict[int, int]:
        """创建ROS棋盘索引到python-chess方格的映射"""
        mapping = {}
        for row in range(8):
            for col in range(8):
                ros_index = row * 8 + col  # ROS: 0-63 (从左上角开始)
                chess_square = chess.square(col, 7 - row)  # python-chess: a1=0, h8=63
                mapping[ros_index] = chess_square
        return mapping

    def board_state_callback(self, msg):
        """棋盘状态回调"""
        if not self.game_started:
            # 检查是否为标准开局位置
            if self.is_starting_position(msg.board_squares):
                self.start_new_game()
                return

        # 处理棋盘状态更新
        self.process_board_update(msg)

    def is_starting_position(self, board_squares) -> bool:
        """检查是否为标准开局位置"""
        # 标准开局位置
        standard_position = [
            -2, -3, -4, -5, -6, -4, -3, -2,  # 第8行（黑方后排）
            -1, -1, -1, -1, -1, -1, -1, -1,  # 第7行（黑方兵）
             0,  0,  0,  0,  0,  0,  0,  0,  # 第6行
             0,  0,  0,  0,  0,  0,  0,  0,  # 第5行
             0,  0,  0,  0,  0,  0,  0,  0,  # 第4行
             0,  0,  0,  0,  0,  0,  0,  0,  # 第3行
             1,  1,  1,  1,  1,  1,  1,  1,  # 第2行（白方兵）
             2,  3,  4,  5,  6,  4,  3,  2,  # 第1行（白方后排）
        ]

        # 计算匹配程度
        matches = sum(1 for i, j in zip(board_squares, standard_position) if i == j)
        return matches >= 60  # 至少94%匹配

    def process_board_update(self, board_msg):
        """处理棋盘更新"""
        if not self.game_started or self.game_over:
            return

        # 转换ROS棋盘状态到python-chess
        new_board = self.ros_board_to_chess_board(board_msg.board_squares)

        # 检测移动
        if new_board != self.board:
            move = self.detect_move(self.board, new_board)
            if move:
                self.process_move(move)
            else:
                self.get_logger().warning("检测到棋盘变化但无法识别有效移动")

    def ros_board_to_chess_board(self, ros_board) -> chess.Board:
        """将ROS棋盘状态转换为python-chess棋盘"""
        board = chess.Board(fen=None)  # 空棋盘
        board.clear()

        for ros_idx, piece_code in enumerate(ros_board):
            if piece_code != 0:
                chess_square = self.ros_to_chess_mapping[ros_idx]

                # 确定棋子颜色和类型
                is_white = piece_code > 0
                piece_type = abs(piece_code)

                # 映射棋子类型
                if piece_type == 1:
                    chess_piece = chess.PAWN
                elif piece_type == 2:
                    chess_piece = chess.ROOK
                elif piece_type == 3:
                    chess_piece = chess.KNIGHT
                elif piece_type == 4:
                    chess_piece = chess.BISHOP
                elif piece_type == 5:
                    chess_piece = chess.QUEEN
                elif piece_type == 6:
                    chess_piece = chess.KING
                else:
                    continue

                # 放置棋子
                piece = chess.Piece(chess_piece, is_white)
                board.set_piece_at(chess_square, piece)

        return board

    def detect_move(self, old_board: chess.Board, new_board: chess.Board) -> Optional[chess.Move]:
        """检测从旧棋盘到新棋盘的移动"""
        # 比较两个棋盘状态，找出差异
        differences = []
        for square in chess.SQUARES:
            old_piece = old_board.piece_at(square)
            new_piece = new_board.piece_at(square)

            if old_piece != new_piece:
                differences.append({
                    'square': square,
                    'old_piece': old_piece,
                    'new_piece': new_piece
                })

        # 分析差异找出移动
        if len(differences) == 2:
            # 正常移动：一个位置变空，另一个位置出现棋子
            from_square = None
            to_square = None

            for diff in differences:
                if diff['old_piece'] is not None and diff['new_piece'] is None:
                    from_square = diff['square']
                elif diff['old_piece'] is None and diff['new_piece'] is not None:
                    to_square = diff['square']
                elif (diff['old_piece'] is not None and diff['new_piece'] is not None and
                      diff['old_piece'] != diff['new_piece']):
                    # 吃子移动
                    to_square = diff['square']

            if from_square is not None and to_square is not None:
                move = chess.Move(from_square, to_square)

                # 检查升变
                piece = old_board.piece_at(from_square)
                if (piece and piece.piece_type == chess.PAWN and
                    ((piece.color == chess.WHITE and chess.square_rank(to_square) == 7) or
                     (piece.color == chess.BLACK and chess.square_rank(to_square) == 0))):
                    # 默认升变为后
                    move = chess.Move(from_square, to_square, promotion=chess.QUEEN)

                # 验证移动是否合法
                if move in old_board.legal_moves:
                    return move

        # 处理王车易位等特殊移动
        elif len(differences) == 4:
            # 可能是王车易位
            for move in old_board.legal_moves:
                if old_board.is_castling(move):
                    # 临时执行移动看是否匹配
                    temp_board = old_board.copy()
                    temp_board.push(move)
                    if self.boards_equal(temp_board, new_board):
                        return move

        return None

    def boards_equal(self, board1: chess.Board, board2: chess.Board) -> bool:
        """比较两个棋盘是否相等（仅比较棋子位置）"""
        for square in chess.SQUARES:
            if board1.piece_at(square) != board2.piece_at(square):
                return False
        return True

    def process_move(self, move: chess.Move):
        """处理移动"""
        if move not in self.board.legal_moves:
            self.get_logger().error(f"非法移动: {move}")
            return

        # 记录移动时间
        current_time = time.time()
        if self.last_move_time:
            move_time = current_time - self.last_move_time
            if self.board.turn == chess.WHITE:
                self.white_time_left -= move_time
            else:
                self.black_time_left -= move_time

        # 执行移动
        self.board.push(move)
        self.move_history.append(move)
        self.last_move_time = current_time

        # 记录日志
        move_info = {
            'move': move.uci(),
            'san': self.board.san(move),
            'timestamp': current_time,
            'fen': self.board.fen(),
            'player': 'white' if self.board.turn == chess.BLACK else 'black'
        }
        self.game_log.append(move_info)

        self.get_logger().info(f"移动执行: {self.board.san(move)} ({move.uci()})")

        # 切换当前玩家
        self.current_player = 'black' if self.current_player == 'white' else 'white'

        # 检查游戏结束条件
        self.check_game_end()

        # 如果轮到AI，请求AI移动
        if not self.game_over and self.is_ai_turn():
            self.request_ai_move()

        # 发布游戏状态
        self.publish_game_state()

    def is_ai_turn(self) -> bool:
        """检查是否轮到AI走棋"""
        if self.game_mode == 'human_vs_human':
            return False
        elif self.game_mode == 'ai_vs_ai':
            return True
        else:  # human_vs_ai
            return self.current_player == self.ai_color

    def request_ai_move(self):
        """请求AI移动"""
        # 发布AI移动请求
        msg = String()
        msg.data = json.dumps({
            'action': 'get_move',
            'fen': self.board.fen(),
            'time_left': self.black_time_left if self.ai_color == 'black' else self.white_time_left,
            'skill_level': self.skill_level
        })
        self.game_state_pub.publish(msg)
        self.get_logger().info("已请求AI移动")

    def check_game_end(self):
        """检查游戏结束条件"""
        if self.board.is_checkmate():
            winner = 'white' if self.board.turn == chess.BLACK else 'black'
            self.end_game(f"将死 - {winner}获胜")
        elif self.board.is_stalemate():
            self.end_game("和棋 - 逼和")
        elif self.board.is_insufficient_material():
            self.end_game("和棋 - 子力不足")
        elif self.board.is_seventyfive_moves():
            self.end_game("和棋 - 75步规则")
        elif self.board.is_fivefold_repetition():
            self.end_game("和棋 - 五次重复")
        elif self.white_time_left <= 0:
            self.end_game("超时 - 黑方获胜")
        elif self.black_time_left <= 0:
            self.end_game("超时 - 白方获胜")

    def end_game(self, result: str):
        """结束游戏"""
        self.game_over = True
        self.get_logger().info(f"游戏结束: {result}")

        # 记录游戏结果
        game_result = {
            'result': result,
            'moves': len(self.move_history),
            'duration': time.time() - self.start_time if self.start_time else 0,
            'final_fen': self.board.fen()
        }
        self.game_log.append(game_result)

        # 保存游戏日志
        if self.game_log_enabled:
            self.save_game_log()

        # 发布游戏结束状态
        self.publish_game_state()

    def start_new_game(self):
        """开始新游戏"""
        self.board = chess.Board()
        self.game_started = True
        self.game_over = False
        self.current_player = 'white'
        self.move_history = []
        self.game_log = []
        self.start_time = time.time()
        self.last_move_time = self.start_time
        self.white_time_left = self.time_control
        self.black_time_left = self.time_control

        self.get_logger().info("新游戏开始")
        self.publish_game_state()

        # 如果AI执白先走
        if self.ai_color == 'white' and self.game_mode in ['human_vs_ai', 'ai_vs_ai']:
            self.request_ai_move()

    def publish_game_state(self):
        """发布游戏状态"""
        state = {
            'game_started': self.game_started,
            'game_over': self.game_over,
            'current_player': self.current_player,
            'fen': self.board.fen(),
            'moves': len(self.move_history),
            'white_time': self.white_time_left,
            'black_time': self.black_time_left,
            'last_move': self.move_history[-1].uci() if self.move_history else None,
            'check': self.board.is_check(),
            'legal_moves': [move.uci() for move in self.board.legal_moves]
        }

        msg = String()
        msg.data = json.dumps(state)
        self.game_state_pub.publish(msg)

    def publish_status(self):
        """定时发布状态"""
        if self.game_started and not self.game_over:
            # 更新时间
            current_time = time.time()
            if self.last_move_time:
                elapsed = current_time - self.last_move_time
                if self.board.turn == chess.WHITE:
                    self.white_time_left = max(0, self.white_time_left - elapsed)
                else:
                    self.black_time_left = max(0, self.black_time_left - elapsed)
                self.last_move_time = current_time

            # 检查超时
            if self.white_time_left <= 0 or self.black_time_left <= 0:
                self.check_game_end()

        # 发布状态
        status = {
            'engine_status': 'running',
            'game_mode': self.game_mode,
            'game_started': self.game_started,
            'game_over': self.game_over,
            'current_player': self.current_player,
            'move_count': len(self.move_history),
            'white_time': round(self.white_time_left, 1),
            'black_time': round(self.black_time_left, 1)
        }

        msg = String()
        msg.data = json.dumps(status)
        self.game_status_pub.publish(msg)

    def save_game_log(self):
        """保存游戏日志"""
        try:
            filename = f"chess_game_{int(time.time())}.json"
            with open(filename, 'w') as f:
                json.dump(self.game_log, f, indent=2)
            self.get_logger().info(f"游戏日志已保存: {filename}")
        except Exception as e:
            self.get_logger().error(f"保存游戏日志失败: {e}")

    def reset_game(self):
        """重置游戏"""
        self.game_started = False
        self.game_over = False
        self.board = chess.Board()
        self.current_player = 'white'
        self.move_history = []
        self.game_log = []
        self.white_time_left = self.time_control
        self.black_time_left = self.time_control
        self.get_logger().info("游戏已重置")


def main(args=None):
    rclpy.init(args=args)
    node = GameEngine()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()