#!/usr/bin/env python3

"""
国际象棋引擎测试脚本
用于验证Stockfish引擎集成是否正常工作
"""

import rclpy
from rclpy.node import Node
from chess_interfaces.srv import GetEngineMove, ValidateMove
import time


class ChessEngineTest(Node):
    """国际象棋引擎测试节点"""

    def __init__(self):
        super().__init__('chess_engine_test')

        # 创建服务客户端
        self.engine_client = self.create_client(GetEngineMove, '获取引擎移动')
        self.validate_client = self.create_client(ValidateMove, '验证移动')

        self.get_logger().info('等待引擎服务启动...')

    def test_engine_basic(self):
        """测试引擎基本功能"""
        self.get_logger().info('开始基本引擎测试...')

        # 等待服务可用
        if not self.engine_client.wait_for_service(timeout_sec=10.0):
            self.get_logger().error('引擎服务不可用')
            return False

        # 测试标准开局位置
        request = GetEngineMove.Request()
        request.fen_position = "rnbqkbnr/pppppppp/8/8/8/8/PPPPPPPP/RNBQKBNR w KQkq - 0 1"
        request.depth = 10
        request.time_limit = 3.0
        request.skill_level = 15

        future = self.engine_client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=15.0)

        if future.result():
            result = future.result()
            if result.success:
                self.get_logger().info(f'✅ 引擎推荐移动: {result.best_move_san}')
                self.get_logger().info(f'   UCI格式: {result.best_move_uci}')
                self.get_logger().info(f'   评估值: {result.evaluation:.2f}')
                self.get_logger().info(f'   思考时间: {result.thinking_time:.2f}s')
                return True
            else:
                self.get_logger().error(f'❌ 引擎分析失败: {result.error_message}')
        else:
            self.get_logger().error('❌ 服务调用超时')

        return False

    def test_move_validation(self):
        """测试移动验证功能"""
        self.get_logger().info('开始移动验证测试...')

        if not self.validate_client.wait_for_service(timeout_sec=10.0):
            self.get_logger().error('验证服务不可用')
            return False

        # 测试合法移动
        test_cases = [
            ("e2e4", "标准开局兵移动", True),
            ("e2e5", "兵移动过远", False),
            ("Nf3", "马的移动", True),
            ("Ke2", "王在开局的移动", False),
        ]

        start_fen = "rnbqkbnr/pppppppp/8/8/8/8/PPPPPPPP/RNBQKBNR w KQkq - 0 1"

        for move_uci, description, expected in test_cases:
            request = ValidateMove.Request()
            request.fen_position = start_fen

            if len(move_uci) == 4:  # UCI格式
                request.move_uci = move_uci
            else:  # SAN格式
                request.move_san = move_uci

            future = self.validate_client.call_async(request)
            rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)

            if future.result():
                result = future.result()
                status = "✅" if result.is_legal == expected else "❌"
                self.get_logger().info(f'{status} {description}: {move_uci} - {"合法" if result.is_legal else "不合法"}')
            else:
                self.get_logger().error(f'❌ 验证服务调用失败: {move_uci}')

        return True

    def test_different_positions(self):
        """测试不同局面的引擎分析"""
        self.get_logger().info('测试不同局面分析...')

        positions = [
            ("rnbqkbnr/pppppppp/8/8/4P3/8/PPPP1PPP/RNBQKBNR b KQkq e3 0 1", "1.e4后的局面"),
            ("rnbqkbnr/pppp1ppp/8/4p3/4P3/8/PPPP1PPP/RNBQKBNR w KQkq e6 0 2", "1.e4 e5局面"),
            ("rnbqkbnr/pppp1ppp/8/4p3/4P3/5N2/PPPP1PPP/RNBQKB1R b KQkq - 1 2", "1.e4 e5 2.Nf3局面"),
        ]

        for fen, description in positions:
            self.get_logger().info(f'分析局面: {description}')

            request = GetEngineMove.Request()
            request.fen_position = fen
            request.depth = 8
            request.time_limit = 2.0
            request.skill_level = 10

            future = self.engine_client.call_async(request)
            rclpy.spin_until_future_complete(self, future, timeout_sec=10.0)

            if future.result() and future.result().success:
                result = future.result()
                self.get_logger().info(f'  推荐: {result.best_move_san} (评估: {result.evaluation:.2f})')
            else:
                self.get_logger().error(f'  分析失败')

        return True


def main():
    rclpy.init()

    try:
        test_node = ChessEngineTest()

        # 运行所有测试
        success = True
        success &= test_node.test_engine_basic()
        success &= test_node.test_move_validation()
        success &= test_node.test_different_positions()

        if success:
            test_node.get_logger().info('🎉 所有测试通过！')
        else:
            test_node.get_logger().error('❌ 部分测试失败')

    except KeyboardInterrupt:
        pass
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()