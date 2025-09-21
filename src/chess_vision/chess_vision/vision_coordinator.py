#!/usr/bin/env python3
"""
视觉协调器节点
协调棋盘检测和棋子检测，输出完整的棋盘状态
处理检测结果的融合和验证
"""

import rclpy
from rclpy.node import Node
from chess_interfaces.msg import BoardState, PieceDetection
from chess_interfaces.srv import DetectBoard
from std_msgs.msg import Header
import numpy as np
from typing import Dict, List


class VisionCoordinator(Node):
    """视觉协调器节点"""

    def __init__(self):
        super().__init__('vision_coordinator')

        # 参数声明
        self.declare_parameter('update_rate', 5.0)  # Hz
        self.declare_parameter('confidence_threshold', 0.5)
        self.declare_parameter('board_validation_timeout', 10.0)  # 秒
        self.declare_parameter('piece_detection_timeout', 5.0)  # 秒

        # 获取参数
        self.update_rate = self.get_parameter('update_rate').value
        self.confidence_threshold = self.get_parameter('confidence_threshold').value
        self.board_validation_timeout = self.get_parameter('board_validation_timeout').value
        self.piece_detection_timeout = self.get_parameter('piece_detection_timeout').value

        # 状态变量
        self.board_detected = False
        self.last_board_state = None
        self.last_piece_detection = None
        self.last_board_update_time = None
        self.last_piece_update_time = None

        # 游戏状态跟踪
        self.current_board_position = [0] * 64  # 当前棋盘位置
        self.move_history = []  # 移动历史
        self.game_started = False

        # 标准开局位置
        self.standard_starting_position = [
            -2, -3, -4, -5, -6, -4, -3, -2,  # 黑方后排
            -1, -1, -1, -1, -1, -1, -1, -1,  # 黑方兵
             0,  0,  0,  0,  0,  0,  0,  0,  # 空格
             0,  0,  0,  0,  0,  0,  0,  0,  # 空格
             0,  0,  0,  0,  0,  0,  0,  0,  # 空格
             0,  0,  0,  0,  0,  0,  0,  0,  # 空格
             1,  1,  1,  1,  1,  1,  1,  1,  # 白方兵
             2,  3,  4,  5,  6,  4,  3,  2,  # 白方后排
        ]

        # 订阅者
        self.board_state_sub = self.create_subscription(
            BoardState, '/chess/board_state', self.board_state_callback, 10)
        self.piece_detection_sub = self.create_subscription(
            PieceDetection, '/chess/piece_detection', self.piece_detection_callback, 10)

        # 发布者
        self.final_board_state_pub = self.create_publisher(
            BoardState, '/chess/final_board_state', 10)

        # 服务客户端
        self.detect_board_client = self.create_client(DetectBoard, '/chess/detect_board')

        # 定时器
        self.update_timer = self.create_timer(1.0 / self.update_rate, self.update_callback)

        self.get_logger().info("视觉协调器节点已启动")

    def board_state_callback(self, msg):
        """棋盘状态回调"""
        self.last_board_state = msg
        self.last_board_update_time = self.get_clock().now()

        # 检查棋盘是否被正确检测
        if len(msg.square_positions_3d) == 64:
            self.board_detected = True
            self.get_logger().info("棋盘检测成功")
        else:
            self.board_detected = False

    def piece_detection_callback(self, msg):
        """棋子检测回调"""
        self.last_piece_detection = msg
        self.last_piece_update_time = self.get_clock().now()

        # 处理检测到的棋子
        self.process_piece_detection(msg)

    def process_piece_detection(self, detection_msg):
        """处理棋子检测结果"""
        if not self.board_detected:
            return

        # 过滤低置信度的检测
        filtered_pieces = [0] * 64
        for i in range(64):
            if (detection_msg.confidence_scores[i] >= self.confidence_threshold and
                detection_msg.detected_pieces[i] != 0):
                filtered_pieces[i] = detection_msg.detected_pieces[i]

        # 更新当前棋盘位置
        self.update_board_position(filtered_pieces)

    def update_board_position(self, new_position):
        """更新棋盘位置"""
        # 检测移动
        if self.game_started:
            move = self.detect_move(self.current_board_position, new_position)
            if move:
                self.move_history.append(move)
                self.get_logger().info(f"检测到移动: {move}")

        # 更新位置
        self.current_board_position = new_position.copy()

        # 如果还没开始游戏，检查是否匹配标准开局
        if not self.game_started:
            if self.is_standard_starting_position(new_position):
                self.game_started = True
                self.get_logger().info("检测到标准开局位置，游戏开始")

    def detect_move(self, old_position, new_position):
        """检测移动"""
        differences = []
        for i in range(64):
            if old_position[i] != new_position[i]:
                differences.append({
                    'square': i,
                    'old_piece': old_position[i],
                    'new_piece': new_position[i]
                })

        # 简单的移动检测：一个位置变空，另一个位置出现棋子
        if len(differences) == 2:
            from_square = None
            to_square = None

            for diff in differences:
                if diff['old_piece'] != 0 and diff['new_piece'] == 0:
                    from_square = diff['square']
                elif diff['old_piece'] == 0 and diff['new_piece'] != 0:
                    to_square = diff['square']

            if from_square is not None and to_square is not None:
                return {
                    'from': from_square,
                    'to': to_square,
                    'piece': new_position[to_square],
                    'captured': old_position[to_square] if old_position[to_square] != 0 else None
                }

        # 处理吃子移动
        elif len(differences) == 1:
            diff = differences[0]
            if diff['old_piece'] != 0 and diff['new_piece'] != 0:
                # 可能是吃子移动的目标格
                # 需要更复杂的逻辑来确定起始格
                pass

        return None

    def is_standard_starting_position(self, position):
        """检查是否为标准开局位置"""
        # 允许一定的误差
        matches = 0
        for i in range(64):
            if position[i] == self.standard_starting_position[i]:
                matches += 1

        # 如果80%以上匹配，认为是开局位置
        return matches >= 51  # 64 * 0.8

    def update_callback(self):
        """定时更新回调"""
        current_time = self.get_clock().now()

        # 检查数据是否超时
        board_timeout = (self.last_board_update_time is None or
                        (current_time - self.last_board_update_time).nanoseconds / 1e9 >
                        self.board_validation_timeout)

        piece_timeout = (self.last_piece_update_time is None or
                        (current_time - self.last_piece_update_time).nanoseconds / 1e9 >
                        self.piece_detection_timeout)

        # 如果棋盘检测超时，尝试重新检测
        if board_timeout and self.detect_board_client.service_is_ready():
            self.request_board_detection()

        # 发布最终的棋盘状态
        if self.board_detected and self.last_board_state:
            self.publish_final_board_state()

    def request_board_detection(self):
        """请求棋盘检测"""
        request = DetectBoard.Request()
        future = self.detect_board_client.call_async(request)
        future.add_done_callback(self.board_detection_response_callback)

    def board_detection_response_callback(self, future):
        """棋盘检测响应回调"""
        try:
            response = future.result()
            if response.success:
                self.get_logger().info("棋盘重新检测成功")
            else:
                self.get_logger().warning(f"棋盘重新检测失败: {response.message}")
        except Exception as e:
            self.get_logger().error(f"棋盘检测服务调用失败: {e}")

    def publish_final_board_state(self):
        """发布最终的棋盘状态"""
        if not self.last_board_state:
            return

        msg = BoardState()
        msg.header = Header()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.last_board_state.header.frame_id

        # 使用协调后的棋盘位置
        msg.board_squares = self.current_board_position.copy()

        # 复制3D位置信息
        msg.square_positions_3d = self.last_board_state.square_positions_3d

        # 更新游戏状态
        msg.white_to_move = self.calculate_turn()
        msg.castling_rights = self.calculate_castling_rights()
        msg.en_passant_square = self.calculate_en_passant()
        msg.halfmove_clock = self.calculate_halfmove_clock()
        msg.fullmove_number = self.calculate_fullmove_number()

        self.final_board_state_pub.publish(msg)

    def calculate_turn(self) -> bool:
        """计算轮到谁走"""
        # 基于移动历史计算
        return len(self.move_history) % 2 == 0  # 偶数轮白方，奇数轮黑方

    def calculate_castling_rights(self) -> List[bool]:
        """计算王车易位权利"""
        # 简化实现，实际需要跟踪王和车是否移动过
        return [True, True, True, True]  # [白王侧, 白后侧, 黑王侧, 黑后侧]

    def calculate_en_passant(self) -> int:
        """计算吃过路兵目标格"""
        # 检查最后一步是否是兵的两格移动
        if self.move_history:
            last_move = self.move_history[-1]
            # 简化实现
            return -1
        return -1

    def calculate_halfmove_clock(self) -> int:
        """计算半回合时钟"""
        # 从最后一次吃子或兵移动开始计数
        # 简化实现
        return 0

    def calculate_fullmove_number(self) -> int:
        """计算完整移动数"""
        return (len(self.move_history) // 2) + 1

    def get_board_statistics(self) -> Dict:
        """获取棋盘统计信息"""
        stats = {
            'total_pieces': sum(1 for piece in self.current_board_position if piece != 0),
            'white_pieces': sum(1 for piece in self.current_board_position if piece > 0),
            'black_pieces': sum(1 for piece in self.current_board_position if piece < 0),
            'moves_played': len(self.move_history),
            'game_started': self.game_started,
            'board_detected': self.board_detected
        }
        return stats

    def reset_game(self):
        """重置游戏状态"""
        self.current_board_position = [0] * 64
        self.move_history = []
        self.game_started = False
        self.get_logger().info("游戏状态已重置")

    def set_starting_position(self):
        """设置标准开局位置"""
        self.current_board_position = self.standard_starting_position.copy()
        self.game_started = True
        self.move_history = []
        self.get_logger().info("已设置标准开局位置")


def main(args=None):
    rclpy.init(args=args)
    node = VisionCoordinator()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()