#!/usr/bin/env python3

"""
国际象棋机器人系统集成测试
验证所有仿真节点的功能和协同工作
"""

import rclpy
from rclpy.node import Node
import time
import threading
from typing import Dict, List, Optional

from chess_interfaces.srv import GetEngineMove, ValidateMove, DetectBoard
from chess_interfaces.msg import BoardState, GameStatus, ChessMove
import chess


class ChessSystemIntegrationTest(Node):
    """国际象棋系统集成测试节点"""

    def __init__(self):
        super().__init__('国际象棋系统集成测试')

        self.test_results = {}
        self.services_ready = {}

        # 创建服务客户端
        self.engine_client = self.create_client(GetEngineMove, '获取引擎移动')
        self.validate_client = self.create_client(ValidateMove, '验证移动')
        self.detect_client = self.create_client(DetectBoard, '检测棋盘')

        # 订阅消息
        self.board_state = None
        self.game_status = None
        self.engine_moves = []

        self.board_subscription = self.create_subscription(
            BoardState, '棋盘状态', self._board_callback, 10
        )

        self.status_subscription = self.create_subscription(
            GameStatus, '游戏状态', self._status_callback, 10
        )

        self.move_subscription = self.create_subscription(
            ChessMove, '引擎移动', self._move_callback, 10
        )

        self.get_logger().info('🧪 集成测试节点已启动')

    def _board_callback(self, msg: BoardState):
        """棋盘状态回调"""
        self.board_state = msg

    def _status_callback(self, msg: GameStatus):
        """游戏状态回调"""
        self.game_status = msg

    def _move_callback(self, msg: ChessMove):
        """引擎移动回调"""
        self.engine_moves.append(msg)

    def run_all_tests(self) -> bool:
        """运行所有集成测试"""
        self.get_logger().info('🚀 开始运行集成测试...')

        tests = [
            ('服务可用性测试', self.test_services_availability),
            ('引擎功能测试', self.test_engine_functionality),
            ('移动验证测试', self.test_move_validation),
            ('棋盘检测测试', self.test_board_detection),
            ('系统消息测试', self.test_message_flow),
            ('游戏流程测试', self.test_game_flow)
        ]

        all_passed = True
        for test_name, test_func in tests:
            self.get_logger().info(f'📋 运行测试: {test_name}')
            try:
                result = test_func()
                self.test_results[test_name] = result
                status = '✅ 通过' if result else '❌ 失败'
                self.get_logger().info(f'   结果: {status}')
                if not result:
                    all_passed = False
            except Exception as e:
                self.get_logger().error(f'   ❌ 测试异常: {e}')
                self.test_results[test_name] = False
                all_passed = False

        self._print_test_summary()
        return all_passed

    def test_services_availability(self) -> bool:
        """测试所有服务可用性"""
        services = [
            (self.engine_client, '获取引擎移动'),
            (self.validate_client, '验证移动'),
            (self.detect_client, '检测棋盘')
        ]

        for client, name in services:
            if not client.wait_for_service(timeout_sec=10.0):
                self.get_logger().error(f'   服务不可用: {name}')
                return False
            else:
                self.get_logger().info(f'   ✓ 服务就绪: {name}')

        return True

    def test_engine_functionality(self) -> bool:
        """测试引擎功能"""
        # 测试标准开局位置
        request = GetEngineMove.Request()
        request.fen_position = chess.STARTING_FEN
        request.depth = 10
        request.time_limit = 5.0
        request.skill_level = 10

        future = self.engine_client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=15.0)

        if not future.result():
            self.get_logger().error('   引擎服务调用超时')
            return False

        result = future.result()
        if not result.success:
            self.get_logger().error(f'   引擎分析失败: {result.error_message}')
            return False

        self.get_logger().info(f'   ✓ 引擎推荐: {result.best_move_san}')
        self.get_logger().info(f'   ✓ 评估值: {result.evaluation:.2f}')

        # 验证返回的移动格式
        if not result.best_move_uci or not result.best_move_san:
            self.get_logger().error('   引擎返回格式无效')
            return False

        return True

    def test_move_validation(self) -> bool:
        """测试移动验证功能"""
        test_cases = [
            ('e2e4', True, '标准开局兵移动'),
            ('e2e5', False, '兵移动过远'),
            ('Nf3', True, '马的移动'),
            ('Ka1', False, '王的非法移动')
        ]

        for move, expected, description in test_cases:
            request = ValidateMove.Request()
            request.fen_position = chess.STARTING_FEN

            if len(move) == 4:  # UCI格式
                request.move_uci = move
            else:  # SAN格式
                request.move_san = move

            future = self.validate_client.call_async(request)
            rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)

            if not future.result():
                self.get_logger().error(f'   验证服务调用失败: {move}')
                return False

            result = future.result()
            if result.is_legal != expected:
                self.get_logger().error(f'   验证错误: {description}')
                return False

            self.get_logger().info(f'   ✓ {description}: {"合法" if result.is_legal else "不合法"}')

        return True

    def test_board_detection(self) -> bool:
        """测试棋盘检测功能"""
        request = DetectBoard.Request()
        request.force_detection = True
        request.timeout = 5.0
        request.use_depth = True
        request.detection_mode = DetectBoard.Request.FAST_MODE

        future = self.detect_client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=10.0)

        if not future.result():
            self.get_logger().error('   棋盘检测服务调用超时')
            return False

        result = future.result()
        if not result.success:
            self.get_logger().error(f'   棋盘检测失败: {result.error_message}')
            return False

        self.get_logger().info(f'   ✓ 棋盘检测成功，置信度: {result.confidence:.2f}')
        return True

    def test_message_flow(self) -> bool:
        """测试系统消息流"""
        # 等待消息
        timeout = 10.0
        start_time = time.time()

        while (time.time() - start_time < timeout and
               (self.board_state is None or self.game_status is None)):
            rclpy.spin_once(self, timeout_sec=0.1)

        if self.board_state is None:
            self.get_logger().error('   未收到棋盘状态消息')
            return False

        if self.game_status is None:
            self.get_logger().error('   未收到游戏状态消息')
            return False

        self.get_logger().info('   ✓ 棋盘状态消息正常')
        self.get_logger().info('   ✓ 游戏状态消息正常')

        return True

    def test_game_flow(self) -> bool:
        """测试游戏流程"""
        # 这是一个简化的游戏流程测试
        # 实际测试需要更复杂的状态验证

        if self.game_status is None:
            self.get_logger().error('   没有游戏状态信息')
            return False

        # 检查游戏状态是否合理
        valid_states = [
            GameStatus.GAME_IDLE,
            GameStatus.GAME_INITIALIZING,
            GameStatus.GAME_WAITING_HUMAN,
            GameStatus.GAME_AI_THINKING
        ]

        if self.game_status.game_state not in valid_states:
            self.get_logger().error(f'   游戏状态异常: {self.game_status.game_state}')
            return False

        self.get_logger().info(f'   ✓ 游戏状态正常: {self.game_status.game_state}')
        return True

    def _print_test_summary(self):
        """打印测试总结"""
        self.get_logger().info('=' * 60)
        self.get_logger().info('📊 集成测试结果总结')
        self.get_logger().info('=' * 60)

        passed = sum(1 for result in self.test_results.values() if result)
        total = len(self.test_results)

        for test_name, result in self.test_results.items():
            status = '✅ 通过' if result else '❌ 失败'
            self.get_logger().info(f'  {status} {test_name}')

        self.get_logger().info('-' * 60)
        self.get_logger().info(f'📈 总体结果: {passed}/{total} 个测试通过')

        if passed == total:
            self.get_logger().info('🎉 所有集成测试通过！系统就绪。')
        else:
            self.get_logger().error(f'⚠️  有 {total - passed} 个测试失败，请检查系统配置')

        self.get_logger().info('=' * 60)


def main():
    rclpy.init()

    try:
        # 等待系统启动
        print('⏳ 等待系统启动...')
        time.sleep(5.0)

        test_node = ChessSystemIntegrationTest()

        # 运行测试
        success = test_node.run_all_tests()

        if success:
            print('\n🎊 集成测试完成 - 系统正常运行！')
            return 0
        else:
            print('\n❌ 集成测试失败 - 请检查系统状态')
            return 1

    except KeyboardInterrupt:
        print('\n⛔ 测试被用户中断')
        return 1
    finally:
        if 'test_node' in locals():
            test_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    exit(main())