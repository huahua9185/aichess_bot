#!/usr/bin/env python3
"""
象棋AI节点
集成Stockfish引擎，提供AI对弈功能
支持多种难度级别和时间控制
"""

import rclpy
from rclpy.node import Node
from chess_interfaces.msg import ChessMove
from std_msgs.msg import String
import chess
import chess.engine
from stockfish import Stockfish
import json
import time
import threading
from typing import Optional, Dict, List


class ChessAI(Node):
    """象棋AI节点"""

    def __init__(self):
        super().__init__('chess_ai')

        # 参数声明
        self.declare_parameter('engine_path', '/usr/bin/stockfish')
        self.declare_parameter('default_skill_level', 15)  # 1-20
        self.declare_parameter('default_depth', 15)
        self.declare_parameter('thinking_time', 1.0)  # 秒
        self.declare_parameter('hash_size', 128)  # MB
        self.declare_parameter('threads', 1)

        # 获取参数
        self.engine_path = self.get_parameter('engine_path').value
        self.default_skill_level = self.get_parameter('default_skill_level').value
        self.default_depth = self.get_parameter('default_depth').value
        self.thinking_time = self.get_parameter('thinking_time').value
        self.hash_size = self.get_parameter('hash_size').value
        self.threads = self.get_parameter('threads').value

        # Stockfish引擎
        self.stockfish = None
        self.engine_ready = False
        self.current_position = None

        # AI状态
        self.is_thinking = False
        self.thinking_start_time = None

        # 初始化Stockfish
        self.initialize_stockfish()

        # 订阅者
        self.game_state_sub = self.create_subscription(
            String, '/chess/game_state', self.game_state_callback, 10)

        # 发布者
        self.move_pub = self.create_publisher(
            ChessMove, '/chess/ai_move', 10)
        self.ai_status_pub = self.create_publisher(
            String, '/chess/ai_status', 10)

        # 定时器
        self.status_timer = self.create_timer(1.0, self.publish_status)

        self.get_logger().info(f"象棋AI节点已启动 - 技能等级: {self.default_skill_level}")

    def initialize_stockfish(self):
        """初始化Stockfish引擎"""
        try:
            # Stockfish配置
            stockfish_config = {
                "Debug Log File": "",
                "Contempt": 0,
                "Min Split Depth": 0,
                "Threads": self.threads,
                "Ponder": "false",
                "Hash": self.hash_size,
                "MultiPV": 1,
                "Skill Level": self.default_skill_level,
                "Move Overhead": 30,
                "Minimum Thinking Time": 20,
                "Slow Mover": 100,
                "UCI_Chess960": "false",
            }

            self.stockfish = Stockfish(
                path=self.engine_path,
                parameters=stockfish_config
            )

            if self.stockfish.is_development_build_of_engine():
                self.get_logger().info("使用Stockfish开发版")
            else:
                self.get_logger().info("使用Stockfish正式版")

            self.engine_ready = True
            self.get_logger().info("Stockfish引擎初始化成功")

        except Exception as e:
            self.get_logger().error(f"Stockfish引擎初始化失败: {e}")
            self.engine_ready = False

    def game_state_callback(self, msg):
        """游戏状态回调"""
        try:
            game_data = json.loads(msg.data)

            # 检查是否是AI移动请求
            if 'action' in game_data and game_data['action'] == 'get_move':
                self.handle_move_request(game_data)

        except json.JSONDecodeError as e:
            self.get_logger().error(f"JSON解析失败: {e}")
        except Exception as e:
            self.get_logger().error(f"处理游戏状态失败: {e}")

    def handle_move_request(self, request_data):
        """处理移动请求"""
        if not self.engine_ready:
            self.get_logger().error("Stockfish引擎未就绪")
            return

        if self.is_thinking:
            self.get_logger().warning("AI正在思考中，忽略新请求")
            return

        # 启动思考线程
        thinking_thread = threading.Thread(
            target=self.think_and_move,
            args=(request_data,)
        )
        thinking_thread.daemon = True
        thinking_thread.start()

    def think_and_move(self, request_data):
        """思考并生成移动"""
        self.is_thinking = True
        self.thinking_start_time = time.time()

        try:
            fen = request_data.get('fen', chess.STARTING_FEN)
            time_left = request_data.get('time_left', 60.0)
            skill_level = request_data.get('skill_level', self.default_skill_level)

            self.get_logger().info(f"AI开始思考 - 技能等级: {skill_level}, 剩余时间: {time_left:.1f}s")

            # 设置位置
            if not self.stockfish.set_fen_position(fen):
                self.get_logger().error("无效的FEN位置")
                return

            # 调整引擎参数
            self.adjust_engine_settings(skill_level, time_left)

            # 计算移动时间
            move_time = min(self.thinking_time, time_left * 0.1)  # 最多使用10%的剩余时间

            # 获取最佳移动
            best_move = self.get_best_move(move_time)

            if best_move:
                self.publish_move(best_move, fen)
            else:
                self.get_logger().error("AI无法找到合法移动")

        except Exception as e:
            self.get_logger().error(f"AI思考过程出错: {e}")
        finally:
            self.is_thinking = False
            thinking_time = time.time() - self.thinking_start_time
            self.get_logger().info(f"AI思考完成，耗时: {thinking_time:.2f}s")

    def adjust_engine_settings(self, skill_level, time_left):
        """调整引擎设置"""
        try:
            # 设置技能等级
            self.stockfish.set_skill_level(skill_level)

            # 根据剩余时间调整深度
            if time_left > 300:  # 5分钟以上，使用较深搜索
                depth = self.default_depth
            elif time_left > 60:  # 1-5分钟，中等深度
                depth = max(10, self.default_depth - 3)
            else:  # 少于1分钟，快速搜索
                depth = max(6, self.default_depth - 6)

            self.stockfish.set_depth(depth)

        except Exception as e:
            self.get_logger().warning(f"调整引擎设置失败: {e}")

    def get_best_move(self, time_limit: float) -> Optional[str]:
        """获取最佳移动"""
        try:
            # 使用时间限制
            start_time = time.time()

            # 获取最佳移动
            best_move = self.stockfish.get_best_move_time(int(time_limit * 1000))

            if best_move:
                # 获取评估信息
                evaluation = self.stockfish.get_evaluation()
                self.get_logger().info(f"最佳移动: {best_move}, 评估: {evaluation}")

                return best_move
            else:
                # 如果Stockfish无法返回移动，尝试获取任何合法移动
                legal_moves = self.get_legal_moves()
                if legal_moves:
                    return legal_moves[0]  # 返回第一个合法移动

        except Exception as e:
            self.get_logger().error(f"获取最佳移动失败: {e}")

        return None

    def get_legal_moves(self) -> List[str]:
        """获取当前位置的所有合法移动"""
        try:
            board = chess.Board(self.stockfish.get_fen_position())
            return [move.uci() for move in board.legal_moves]
        except Exception as e:
            self.get_logger().error(f"获取合法移动失败: {e}")
            return []

    def publish_move(self, move_uci: str, fen: str):
        """发布AI移动"""
        try:
            # 解析移动
            move = chess.Move.from_uci(move_uci)

            # 转换为ROS消息
            chess_move = ChessMove()
            chess_move.header.stamp = self.get_clock().now().to_msg()
            chess_move.header.frame_id = 'chess_board'

            # 转换坐标
            from_square = self.chess_square_to_ros_index(move.from_square)
            to_square = self.chess_square_to_ros_index(move.to_square)

            chess_move.from_square = from_square
            chess_move.to_square = to_square
            chess_move.promotion_piece = 0

            # 处理升变
            if move.promotion:
                if move.promotion == chess.QUEEN:
                    chess_move.promotion_piece = 5
                elif move.promotion == chess.ROOK:
                    chess_move.promotion_piece = 2
                elif move.promotion == chess.BISHOP:
                    chess_move.promotion_piece = 4
                elif move.promotion == chess.KNIGHT:
                    chess_move.promotion_piece = 3

            # 设置移动类型
            board = chess.Board(fen)
            if board.is_castling(move):
                chess_move.is_castling = True
            elif board.is_en_passant(move):
                chess_move.is_en_passant = True
            elif board.is_capture(move):
                chess_move.is_capture = True

            chess_move.move_notation = move_uci

            # 发布移动
            self.move_pub.publish(chess_move)
            self.get_logger().info(f"AI移动已发布: {move_uci}")

        except Exception as e:
            self.get_logger().error(f"发布AI移动失败: {e}")

    def chess_square_to_ros_index(self, chess_square: int) -> int:
        """将python-chess方格转换为ROS索引"""
        file = chess.square_file(chess_square)  # 0-7 (a-h)
        rank = chess.square_rank(chess_square)  # 0-7 (1-8)
        ros_index = (7 - rank) * 8 + file  # ROS从左上角开始
        return ros_index

    def get_engine_info(self) -> Dict:
        """获取引擎信息"""
        if not self.engine_ready:
            return {'status': 'not_ready'}

        try:
            info = {
                'status': 'ready',
                'skill_level': self.stockfish.get_parameters()['Skill Level'],
                'depth': self.stockfish.get_parameters().get('Depth', 'auto'),
                'hash_size': self.stockfish.get_parameters()['Hash'],
                'threads': self.stockfish.get_parameters()['Threads'],
                'is_thinking': self.is_thinking
            }

            if self.is_thinking and self.thinking_start_time:
                info['thinking_time'] = time.time() - self.thinking_start_time

            return info

        except Exception as e:
            self.get_logger().error(f"获取引擎信息失败: {e}")
            return {'status': 'error', 'error': str(e)}

    def publish_status(self):
        """定时发布AI状态"""
        status = {
            'node_status': 'running',
            'engine_info': self.get_engine_info(),
            'timestamp': time.time()
        }

        msg = String()
        msg.data = json.dumps(status)
        self.ai_status_pub.publish(msg)

    def analyze_position(self, fen: str, depth: int = 15) -> Dict:
        """分析位置"""
        if not self.engine_ready:
            return {'error': 'Engine not ready'}

        try:
            self.stockfish.set_fen_position(fen)
            self.stockfish.set_depth(depth)

            analysis = {
                'best_move': self.stockfish.get_best_move(),
                'evaluation': self.stockfish.get_evaluation(),
                'top_moves': self.stockfish.get_top_moves(5),
                'fen': fen
            }

            return analysis

        except Exception as e:
            self.get_logger().error(f"位置分析失败: {e}")
            return {'error': str(e)}

    def set_difficulty(self, skill_level: int):
        """设置AI难度"""
        if not self.engine_ready:
            return False

        try:
            if 1 <= skill_level <= 20:
                self.stockfish.set_skill_level(skill_level)
                self.default_skill_level = skill_level
                self.get_logger().info(f"AI难度已设置为: {skill_level}")
                return True
            else:
                self.get_logger().warning("技能等级必须在1-20之间")
                return False

        except Exception as e:
            self.get_logger().error(f"设置AI难度失败: {e}")
            return False


def main(args=None):
    rclpy.init(args=args)
    node = ChessAI()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # 清理资源
        if node.stockfish:
            try:
                del node.stockfish
            except:
                pass
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()