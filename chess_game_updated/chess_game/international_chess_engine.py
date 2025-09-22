#!/usr/bin/env python3

"""
国际象棋Stockfish引擎节点
集成Stockfish引擎，提供AI对弈能力和局面分析
"""

import rclpy
from rclpy.node import Node
from chess_interfaces.msg import ChessMove, BoardState, GameStatus
from chess_interfaces.srv import GetEngineMove, ValidateMove
import chess
import chess.engine
import chess.pgn
import threading
import time
from typing import Optional, Dict, List


class InternationalChessEngine(Node):
    """国际象棋Stockfish引擎节点"""

    def __init__(self):
        super().__init__('国际象棋引擎')

        # 声明参数
        self.declare_parameter('engine_path', '/usr/games/stockfish')
        self.declare_parameter('default_depth', 15)
        self.declare_parameter('default_time', 5.0)
        self.declare_parameter('skill_level', 20)
        self.declare_parameter('threads', 2)
        self.declare_parameter('memory_mb', 256)
        self.declare_parameter('simulation_mode', True)

        # 获取参数
        self.engine_path = self.get_parameter('engine_path').value
        self.default_depth = self.get_parameter('default_depth').value
        self.default_time = self.get_parameter('default_time').value
        self.skill_level = self.get_parameter('skill_level').value
        self.threads = self.get_parameter('threads').value
        self.memory_mb = self.get_parameter('memory_mb').value
        self.simulation_mode = self.get_parameter('simulation_mode').value

        # 初始化Stockfish引擎
        self.engine = None
        self.chess_board = chess.Board()
        self.is_analyzing = False
        self.analysis_lock = threading.Lock()

        self._initialize_engine()

        # 创建服务
        self.get_move_service = self.create_service(
            GetEngineMove, '获取引擎移动', self._get_engine_move_callback
        )

        self.validate_move_service = self.create_service(
            ValidateMove, '验证移动', self._validate_move_callback
        )

        # 订阅棋盘状态
        self.board_subscription = self.create_subscription(
            BoardState, '棋盘状态', self._board_state_callback, 10
        )

        # 发布引擎分析
        self.analysis_publisher = self.create_publisher(
            ChessMove, '引擎分析', 10
        )

        # 当前棋局状态
        self.current_fen = chess.STARTING_FEN
        self.last_board_update = None

        self.get_logger().info(f'国际象棋Stockfish引擎已初始化 - 技能等级: {self.skill_level}')

    def _initialize_engine(self):
        """初始化Stockfish引擎"""
        try:
            # 尝试不同的Stockfish路径
            possible_paths = [
                '/usr/games/stockfish',
                '/usr/bin/stockfish',
                '/usr/local/bin/stockfish',
                'stockfish',
                '/opt/homebrew/bin/stockfish'  # macOS Homebrew
            ]

            for path in possible_paths:
                try:
                    self.engine = chess.engine.SimpleEngine.popen_uci(path)
                    self.get_logger().info(f'Stockfish找到于: {path}')
                    break
                except (chess.engine.EngineTerminatedError, FileNotFoundError):
                    continue

            if self.engine is None:
                raise Exception("未找到Stockfish引擎")

            # 配置引擎参数
            self.engine.configure({
                "Skill Level": min(20, max(0, self.skill_level)),
                "Threads": self.threads,
                "Hash": self.memory_mb,
                "UCI_LimitStrength": self.skill_level < 20,
                "UCI_Elo": 800 + (self.skill_level * 100) if self.skill_level < 20 else 3200
            })

            self.get_logger().info(
                f'Stockfish配置完成: 技能={self.skill_level}, 线程={self.threads}, 内存={self.memory_mb}MB'
            )

        except Exception as e:
            self.get_logger().error(f'Stockfish初始化失败: {str(e)}')
            if not self.simulation_mode:
                self._attempt_install_stockfish()

    def _attempt_install_stockfish(self):
        """尝试安装Stockfish引擎"""
        import subprocess
        try:
            self.get_logger().info('正在安装Stockfish...')
            subprocess.run(['sudo', 'apt', 'update'], check=True)
            subprocess.run(['sudo', 'apt', 'install', '-y', 'stockfish'], check=True)

            # 重新初始化
            self.engine = chess.engine.SimpleEngine.popen_uci('/usr/games/stockfish')
            self.get_logger().info('Stockfish安装并初始化成功')

        except Exception as e:
            self.get_logger().error(f'Stockfish安装失败: {str(e)}')

    def _board_state_callback(self, msg: BoardState):
        """棋盘状态回调"""
        if msg.board_detected and msg.fen_notation:
            try:
                # 验证FEN格式
                test_board = chess.Board(msg.fen_notation)
                self.current_fen = msg.fen_notation
                self.chess_board = test_board
                self.last_board_update = self.get_clock().now()

            except ValueError as e:
                self.get_logger().warn(f'无效的FEN格式: {msg.fen_notation} - {e}')

    def _get_engine_move_callback(self, request: GetEngineMove.Request,
                                response: GetEngineMove.Response) -> GetEngineMove.Response:
        """获取引擎移动服务回调"""

        with self.analysis_lock:
            self.is_analyzing = True

        try:
            if self.engine is None:
                response.success = False
                response.error_message = "Stockfish引擎未初始化"
                return response

            # 设置棋盘位置
            try:
                board = chess.Board(request.fen_position)
            except ValueError as e:
                response.success = False
                response.error_message = f"无效的FEN位置: {e}"
                return response

            # 检查游戏是否结束
            if board.is_game_over():
                response.success = False
                response.error_message = "游戏已结束"
                response.is_forced_move = True
                return response

            # 配置引擎参数
            if request.skill_level > 0:
                skill = min(20, max(1, request.skill_level))
                self.engine.configure({"Skill Level": skill})

            # 设置分析限制
            limit = chess.engine.Limit()
            if request.time_limit > 0:
                limit.time = request.time_limit
            if request.depth > 0:
                limit.depth = min(30, max(1, request.depth))

            # 如果没有指定限制，使用默认值
            if limit.time is None and limit.depth is None:
                limit.time = self.default_time

            # 分析位置
            start_time = time.time()

            # 多重PV分析
            multipv = max(1, min(5, request.multipv)) if request.multipv > 0 else 1
            if multipv > 1:
                self.engine.configure({"MultiPV": multipv})

            result = self.engine.play(board, limit)
            thinking_time = time.time() - start_time

            if result.move is None:
                response.success = False
                response.error_message = "引擎无法找到合法移动"
                return response

            # 构建响应
            response.success = True
            response.best_move_uci = result.move.uci()
            response.best_move_san = board.san(result.move)
            response.thinking_time = thinking_time

            # 获取评估信息
            if hasattr(result, 'info') and result.info:
                info = result.info
                if 'score' in info:
                    score = info['score'].relative
                    if score.is_mate():
                        response.evaluation = 9999 if score.mate() > 0 else -9999
                    else:
                        response.evaluation = score.score() / 100.0  # 转换为pawns

                if 'pv' in info and info['pv']:
                    response.principal_variation = [move.uci() for move in info['pv']]

                if 'depth' in info:
                    response.search_depth = info['depth']
                if 'nodes' in info:
                    response.nodes_searched = info['nodes']

            # 创建ChessMove消息
            chess_move = ChessMove()
            chess_move.timestamp = self.get_clock().now().to_msg()
            chess_move.uci_move = response.best_move_uci
            chess_move.algebraic_notation = response.best_move_san
            chess_move.from_square = result.move.uci()[:2]
            chess_move.to_square = result.move.uci()[2:4]
            chess_move.from_square_index = result.move.from_square
            chess_move.to_square_index = result.move.to_square

            # 分析移动类型
            if board.is_capture(result.move):
                chess_move.move_type = ChessMove.CAPTURE_MOVE
            elif board.is_castling(result.move):
                chess_move.move_type = ChessMove.CASTLE_MOVE
            elif board.is_en_passant(result.move):
                chess_move.move_type = ChessMove.EN_PASSANT_MOVE
            elif result.move.promotion:
                chess_move.move_type = ChessMove.PROMOTION_MOVE
                chess_move.promotion_piece_type = self._piece_type_to_int(result.move.promotion)
            else:
                chess_move.move_type = ChessMove.NORMAL_MOVE

            # 检查是否将军/将死
            temp_board = board.copy()
            temp_board.push(result.move)
            chess_move.gives_check = temp_board.is_check()
            chess_move.gives_checkmate = temp_board.is_checkmate()

            response.best_move = chess_move
            response.confidence = min(1.0, max(0.0, 1.0 - thinking_time / 10.0))

            # 检查是否只有一个合法移动
            legal_moves = list(board.legal_moves)
            response.is_forced_move = len(legal_moves) == 1

            self.get_logger().info(
                f'引擎推荐移动: {response.best_move_san} '
                f'(评估: {response.evaluation:.2f}, 用时: {thinking_time:.2f}s)'
            )

        except Exception as e:
            response.success = False
            response.error_message = f"引擎分析错误: {str(e)}"
            self.get_logger().error(f"引擎分析失败: {e}")

        finally:
            with self.analysis_lock:
                self.is_analyzing = False

        return response

    def _validate_move_callback(self, request: ValidateMove.Request,
                              response: ValidateMove.Response) -> ValidateMove.Response:
        """验证移动服务回调"""

        try:
            # 设置棋盘位置
            board = chess.Board(request.fen_position)

            # 解析移动
            move = None
            if request.move_uci:
                try:
                    move = chess.Move.from_uci(request.move_uci)
                except ValueError:
                    response.is_legal = False
                    response.validation_status = ValidateMove.Response.INVALID_FORMAT
                    response.error_message = f"无效的UCI格式: {request.move_uci}"
                    return response

            elif request.move_san:
                try:
                    move = board.parse_san(request.move_san)
                except ValueError:
                    response.is_legal = False
                    response.validation_status = ValidateMove.Response.INVALID_FORMAT
                    response.error_message = f"无效的代数记号: {request.move_san}"
                    return response

            if move is None:
                response.is_legal = False
                response.validation_status = ValidateMove.Response.INVALID_FORMAT
                response.error_message = "未提供移动信息"
                return response

            # 验证移动合法性
            if move not in board.legal_moves:
                response.is_legal = False

                # 详细错误分析
                if not (0 <= move.from_square <= 63):
                    response.validation_status = ValidateMove.Response.INVALID_FORMAT
                elif board.piece_at(move.from_square) is None:
                    response.validation_status = ValidateMove.Response.NO_PIECE_AT_SOURCE
                elif board.turn != (board.piece_at(move.from_square).color == chess.WHITE):
                    response.validation_status = ValidateMove.Response.WRONG_TURN
                else:
                    response.validation_status = ValidateMove.Response.INVALID_PATH

                response.error_message = "移动不符合国际象棋规则"
            else:
                response.is_legal = True
                response.validation_status = ValidateMove.Response.VALID

                # 构建验证后的移动信息
                chess_move = ChessMove()
                chess_move.timestamp = self.get_clock().now().to_msg()
                chess_move.uci_move = move.uci()
                chess_move.algebraic_notation = board.san(move)
                chess_move.from_square = chess.square_name(move.from_square)
                chess_move.to_square = chess.square_name(move.to_square)
                chess_move.from_square_index = move.from_square
                chess_move.to_square_index = move.to_square

                response.validated_move = chess_move

                # 预测结果
                temp_board = board.copy()
                temp_board.push(move)
                response.resulting_fen = temp_board.fen()
                response.gives_check = temp_board.is_check()
                response.gives_checkmate = temp_board.is_checkmate()
                response.gives_stalemate = temp_board.is_stalemate()
                response.gives_draw = temp_board.is_insufficient_material() or \
                                    temp_board.is_seventyfive_moves() or \
                                    temp_board.is_fivefold_repetition()

                if response.gives_draw:
                    if temp_board.is_insufficient_material():
                        response.draw_reason = "子力不足"
                    elif temp_board.is_seventyfive_moves():
                        response.draw_reason = "75回合规则"
                    elif temp_board.is_fivefold_repetition():
                        response.draw_reason = "五次重复局面"

            # 提供所有合法移动
            response.legal_moves = [move.uci() for move in board.legal_moves]
            response.move_description = f"{move.uci()} - {board.san(move) if move in board.legal_moves else '无效移动'}"

        except ValueError as e:
            response.is_legal = False
            response.validation_status = ValidateMove.Response.INVALID_FORMAT
            response.error_message = f"FEN格式错误: {e}"
        except Exception as e:
            response.is_legal = False
            response.validation_status = ValidateMove.Response.INVALID_FORMAT
            response.error_message = f"验证错误: {e}"

        return response

    def _piece_type_to_int(self, piece_type: int) -> int:
        """将python-chess棋子类型转换为消息整数"""
        # chess.QUEEN=5, chess.ROOK=4, chess.BISHOP=3, chess.KNIGHT=2
        # 我们的消息格式: 2=后, 3=车, 4=象, 5=马
        mapping = {
            chess.QUEEN: 2,
            chess.ROOK: 3,
            chess.BISHOP: 4,
            chess.KNIGHT: 5
        }
        return mapping.get(piece_type, 2)  # 默认升变为后

    def destroy_node(self):
        """节点销毁时清理资源"""
        if self.engine:
            try:
                self.engine.quit()
            except:
                pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)

    try:
        engine = InternationalChessEngine()
        rclpy.spin(engine)
    except KeyboardInterrupt:
        pass
    finally:
        if 'engine' in locals():
            engine.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()