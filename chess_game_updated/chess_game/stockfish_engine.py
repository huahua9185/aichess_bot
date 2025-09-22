#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from chess_interfaces.msg import ChessMove, BoardState
from stockfish import Stockfish
import threading
import time


class StockfishEngine(Node):
    """Stockfish象棋引擎集成节点"""

    def __init__(self):
        super().__init__('stockfish_engine')

        # 声明参数
        self.declare_parameter('stockfish_path', '/usr/games/stockfish')
        self.declare_parameter('depth', 15)
        self.declare_parameter('time_limit', 5.0)
        self.declare_parameter('threads', 2)

        # 获取参数
        self.stockfish_path = self.get_parameter('stockfish_path').value
        self.depth = self.get_parameter('depth').value
        self.time_limit = self.get_parameter('time_limit').value
        self.threads = self.get_parameter('threads').value

        # 初始化Stockfish引擎
        self.engine = None
        self._initialize_engine()

        # 订阅棋盘状态
        self.board_sub = self.create_subscription(
            BoardState, 'chess/board_state', self._board_callback, 10)

        # 发布AI移动
        self.move_pub = self.create_publisher(
            ChessMove, 'chess/stockfish_move', 10)

        # 当前状态
        self.current_board = None
        self.is_thinking = False

        self.get_logger().info('Stockfish engine initialized')

    def _initialize_engine(self):
        """初始化Stockfish引擎"""
        try:
            # 检查不同可能的Stockfish路径
            possible_paths = [
                '/usr/games/stockfish',
                '/usr/bin/stockfish',
                '/usr/local/bin/stockfish',
                'stockfish'
            ]

            for path in possible_paths:
                try:
                    self.engine = Stockfish(path=path)
                    if self.engine.is_fen_valid("rnbqkbnr/pppppppp/8/8/8/8/PPPPPPPP/RNBQKBNR w KQkq - 0 1"):
                        self.get_logger().info(f'Stockfish found at: {path}')
                        break
                except:
                    continue

            if self.engine is None:
                raise Exception("Stockfish not found")

            # 配置引擎参数
            self.engine.set_depth(self.depth)
            self.engine.set_fen_position("rnbqkbnr/pppppppp/8/8/8/8/PPPPPPPP/RNBQKBNR w KQkq - 0 1")

            self.get_logger().info(f'Stockfish configured: depth={self.depth}')

        except Exception as e:
            self.get_logger().error(f'Failed to initialize Stockfish: {str(e)}')
            self.get_logger().info('Installing Stockfish...')
            self._install_stockfish()

    def _install_stockfish(self):
        """安装Stockfish引擎"""
        import subprocess
        try:
            # 尝试安装Stockfish
            subprocess.run(['sudo', 'apt', 'update'], check=True)
            subprocess.run(['sudo', 'apt', 'install', '-y', 'stockfish'], check=True)

            # 重新初始化
            self.engine = Stockfish(path='/usr/games/stockfish')
            if self.engine.is_fen_valid("rnbqkbnr/pppppppp/8/8/8/8/PPPPPPPP/RNBQKBNR w KQkq - 0 1"):
                self.engine.set_depth(self.depth)
                self.get_logger().info('Stockfish installed and initialized successfully')
            else:
                raise Exception("Stockfish installation failed")

        except Exception as e:
            self.get_logger().error(f'Failed to install Stockfish: {str(e)}')
            # 使用备用的简单引擎
            self.engine = SimpleChessEngine()

    def _board_callback(self, msg):
        """棋盘状态回调"""
        if not msg.board_detected or self.is_thinking:
            return

        self.current_board = msg
        # 在这里可以触发AI分析
        # self._analyze_position()

    def get_best_move(self, board_state, is_white_turn=True):
        """获取最佳移动"""
        if self.engine is None:
            return None

        try:
            # 将棋盘状态转换为FEN格式
            fen = self._board_to_fen(board_state, is_white_turn)
            self.engine.set_fen_position(fen)

            # 获取最佳移动
            best_move = self.engine.get_best_move()
            if best_move:
                return self._convert_stockfish_move(best_move)

        except Exception as e:
            self.get_logger().error(f'Error getting best move: {str(e)}')

        return None

    def _board_to_fen(self, board_state, is_white_turn=True):
        """将棋盘状态转换为FEN格式"""
        # 简化版本：返回标准开局FEN
        # 实际应该根据board_state数组转换
        if is_white_turn:
            return "rnbqkbnr/pppppppp/8/8/8/8/PPPPPPPP/RNBQKBNR w KQkq - 0 1"
        else:
            return "rnbqkbnr/pppppppp/8/8/8/8/PPPPPPPP/RNBQKBNR b KQkq - 0 1"

    def _convert_stockfish_move(self, stockfish_move):
        """转换Stockfish移动格式"""
        move = ChessMove()
        move.timestamp = self.get_clock().now().to_msg()

        # 解析移动字符串（如"e2e4"）
        if len(stockfish_move) >= 4:
            from_pos = stockfish_move[:2]
            to_pos = stockfish_move[2:4]

            move.from_position = from_pos
            move.to_position = to_pos
            move.move_description = f'Stockfish: {from_pos}-{to_pos}'
            move.move_type = ChessMove.NORMAL_MOVE

            # 转换为数组索引（简化）
            move.from_index = self._position_to_index(from_pos)
            move.to_index = self._position_to_index(to_pos)

        return move

    def _position_to_index(self, position):
        """将象棋位置转换为数组索引"""
        if len(position) < 2:
            return 0

        col = ord(position[0]) - ord('a')
        row = int(position[1]) - 1
        return row * 9 + col

    def analyze_position(self, board_state, time_limit=None):
        """分析当前局面"""
        if self.engine is None:
            return {}

        try:
            if time_limit is None:
                time_limit = self.time_limit

            fen = self._board_to_fen(board_state)
            self.engine.set_fen_position(fen)

            # 获取评估信息
            evaluation = self.engine.get_evaluation()
            best_move = self.engine.get_best_move()

            return {
                'evaluation': evaluation,
                'best_move': best_move,
                'fen': fen
            }

        except Exception as e:
            self.get_logger().error(f'Error analyzing position: {str(e)}')
            return {}


class SimpleChessEngine:
    """简单的备用象棋引擎"""

    def __init__(self):
        self.position = "rnbqkbnr/pppppppp/8/8/8/8/PPPPPPPP/RNBQKBNR w KQkq - 0 1"

    def is_fen_valid(self, fen):
        return True

    def set_depth(self, depth):
        pass

    def set_fen_position(self, fen):
        self.position = fen

    def get_best_move(self):
        # 返回一个简单的移动
        return "e2e4"

    def get_evaluation(self):
        return {'type': 'cp', 'value': 0}


def main(args=None):
    rclpy.init(args=args)

    try:
        engine = StockfishEngine()
        rclpy.spin(engine)
    except KeyboardInterrupt:
        pass
    finally:
        if 'engine' in locals():
            engine.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()