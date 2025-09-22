#!/usr/bin/env python3

"""
国际象棋仿真视觉节点
从Gazebo仿真环境中检测棋盘状态和棋子位置
"""

import rclpy
from rclpy.node import Node
import cv2
import numpy as np
from cv_bridge import CvBridge
import chess

from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import Point
from chess_interfaces.msg import BoardState
from chess_interfaces.srv import DetectBoard

import time
from typing import List, Tuple, Optional, Dict


class InternationalChessVisionSim(Node):
    """国际象棋仿真视觉处理节点"""

    def __init__(self):
        super().__init__('国际象棋仿真视觉')

        # 声明参数
        self.declare_parameter('simulation_mode', True)
        self.declare_parameter('debug_mode', False)
        self.declare_parameter('detection_threshold', 0.8)
        self.declare_parameter('board_squares', 64)
        self.declare_parameter('camera_topic', '/camera/image_raw')
        self.declare_parameter('depth_topic', '/camera/depth/image_raw')
        self.declare_parameter('camera_info_topic', '/camera/camera_info')

        # 获取参数
        self.simulation_mode = self.get_parameter('simulation_mode').value
        self.debug_mode = self.get_parameter('debug_mode').value
        self.detection_threshold = self.get_parameter('detection_threshold').value
        self.board_squares = self.get_parameter('board_squares').value
        self.camera_topic = self.get_parameter('camera_topic').value
        self.depth_topic = self.get_parameter('depth_topic').value
        self.camera_info_topic = self.get_parameter('camera_info_topic').value

        # OpenCV bridge
        self.cv_bridge = CvBridge()

        # 相机订阅
        self.image_subscription = self.create_subscription(
            Image, self.camera_topic, self._image_callback, 10
        )

        self.depth_subscription = self.create_subscription(
            Image, self.depth_topic, self._depth_callback, 10
        )

        self.camera_info_subscription = self.create_subscription(
            CameraInfo, self.camera_info_topic, self._camera_info_callback, 10
        )

        # 发布棋盘状态
        self.board_state_publisher = self.create_publisher(
            BoardState, '棋盘状态', 10
        )

        # 棋盘检测服务
        self.detect_service = self.create_service(
            DetectBoard, '检测棋盘', self._detect_board_callback
        )

        # 调试图像发布（如果启用调试模式）
        if self.debug_mode:
            self.debug_image_publisher = self.create_publisher(
                Image, '调试图像', 10
            )

        # 数据存储
        self.latest_image = None
        self.latest_depth = None
        self.camera_info = None
        self.last_detection_time = None
        self.current_board_state = None

        # 棋盘检测参数
        self.board_size = 0.4  # 仿真中的棋盘尺寸 (40cm)
        self.square_size = self.board_size / 8  # 每格5cm

        # 国际象棋棋子类型映射 (仿真环境特定)
        self.piece_colors = {
            'white': 1,
            'black': -1
        }

        self.piece_types = {
            'king': 1,
            'queen': 2,
            'rook': 3,
            'bishop': 4,
            'knight': 5,
            'pawn': 6
        }

        # 标准开局位置 (用于仿真初始化验证)
        self.starting_position = [
            -3, -5, -4, -2, -1, -4, -5, -3,  # 8th rank (黑方)
            -6, -6, -6, -6, -6, -6, -6, -6,  # 7th rank (黑兵)
            0,  0,  0,  0,  0,  0,  0,  0,   # 6th rank
            0,  0,  0,  0,  0,  0,  0,  0,   # 5th rank
            0,  0,  0,  0,  0,  0,  0,  0,   # 4th rank
            0,  0,  0,  0,  0,  0,  0,  0,   # 3rd rank
            6,  6,  6,  6,  6,  6,  6,  6,   # 2nd rank (白兵)
            3,  5,  4,  2,  1,  4,  5,  3    # 1st rank (白方)
        ]

        self.get_logger().info(f'国际象棋仿真视觉节点已启动 - 调试模式: {"开启" if self.debug_mode else "关闭"}')

    def _image_callback(self, msg: Image):
        """RGB图像回调"""
        try:
            self.latest_image = self.cv_bridge.imgmsg_to_cv2(msg, "bgr8")

            # 定期发布棋盘状态检测
            current_time = time.time()
            if (self.last_detection_time is None or
                current_time - self.last_detection_time > 2.0):  # 每2秒检测一次
                self._detect_and_publish_board_state()
                self.last_detection_time = current_time

        except Exception as e:
            self.get_logger().error(f'图像处理错误: {e}')

    def _depth_callback(self, msg: Image):
        """深度图像回调"""
        try:
            self.latest_depth = self.cv_bridge.imgmsg_to_cv2(msg, "passthrough")
        except Exception as e:
            self.get_logger().error(f'深度图像处理错误: {e}')

    def _camera_info_callback(self, msg: CameraInfo):
        """相机信息回调"""
        self.camera_info = msg

    def _detect_board_callback(self, request: DetectBoard.Request,
                             response: DetectBoard.Response) -> DetectBoard.Response:
        """棋盘检测服务回调"""

        start_time = time.time()

        try:
            if self.latest_image is None:
                response.success = False
                response.error_message = "没有可用的图像数据"
                return response

            # 检测棋盘
            board_state = self._detect_board_from_image(
                self.latest_image,
                force_detection=request.force_detection,
                use_depth=request.use_depth,
                detection_mode=request.detection_mode
            )

            if board_state is not None:
                response.success = True
                response.board_state = board_state
                response.confidence = board_state.detection_confidence
                response.detection_time = int((time.time() - start_time) * 1000)

                if request.detection_mode == DetectBoard.Request.DEBUG_MODE:
                    response.debug_info = self._generate_debug_info(board_state)

            else:
                response.success = False
                response.error_message = "棋盘检测失败"

        except Exception as e:
            response.success = False
            response.error_message = f"检测过程出错: {str(e)}"
            self.get_logger().error(f'棋盘检测服务错误: {e}')

        response.detection_time = int((time.time() - start_time) * 1000)
        return response

    def _detect_and_publish_board_state(self):
        """检测并发布棋盘状态"""
        if self.latest_image is None:
            return

        board_state = self._detect_board_from_image(self.latest_image)
        if board_state is not None:
            self.current_board_state = board_state
            self.board_state_publisher.publish(board_state)

            if self.debug_mode:
                self._publish_debug_image()

    def _detect_board_from_image(self, image: np.ndarray,
                                force_detection: bool = False,
                                use_depth: bool = True,
                                detection_mode: int = 0) -> Optional[BoardState]:
        """从图像检测棋盘状态"""

        try:
            # 在仿真模式下，我们简化检测逻辑
            # 实际应用中这里会有复杂的计算机视觉算法

            board_state = BoardState()
            board_state.timestamp = self.get_clock().now().to_msg()

            if self.simulation_mode:
                # 仿真模式：基于预设逻辑生成棋盘状态
                board_state = self._simulate_board_detection(image)
            else:
                # 真实模式：运行完整的视觉检测算法
                board_state = self._real_board_detection(image, use_depth)

            # 设置检测置信度
            board_state.detection_confidence = self.detection_threshold
            board_state.board_detected = True

            return board_state

        except Exception as e:
            self.get_logger().error(f'棋盘检测错误: {e}')
            return None

    def _simulate_board_detection(self, image: np.ndarray) -> BoardState:
        """仿真模式下的棋盘检测"""

        board_state = BoardState()
        board_state.timestamp = self.get_clock().now().to_msg()

        # 使用标准开局位置（仿真环境初始状态）
        board_state.board_squares = self.starting_position.copy()

        # 设置FEN位置
        board_state.fen_notation = chess.STARTING_FEN
        board_state.white_to_move = True

        # 计算每个格子的3D位置（相对于机械臂base_link）
        positions = []
        for rank in range(8):
            for file in range(8):
                # 计算世界坐标位置
                x = -0.175 + file * self.square_size  # -17.5cm到17.5cm
                y = -0.175 + (7-rank) * self.square_size  # 从白方视角
                z = 0.02  # 棋盘表面高度

                point = Point()
                point.x = float(x)
                point.y = float(y)
                point.z = float(z)
                positions.append(point)

        board_state.square_positions = positions
        board_state.detection_confidence = 0.95
        board_state.board_detected = True
        board_state.piece_count = 32  # 开局32个棋子

        return board_state

    def _real_board_detection(self, image: np.ndarray, use_depth: bool) -> BoardState:
        """真实模式下的棋盘检测（完整算法）"""

        # 这里实现完整的视觉检测算法
        # 包括：棋盘角点检测、棋子分类、位置计算等

        board_state = BoardState()
        board_state.timestamp = self.get_clock().now().to_msg()

        # 简化实现：检测棋盘角点
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

        # 棋盘角点检测
        pattern_size = (7, 7)  # 内部角点
        found, corners = cv2.findChessboardCorners(gray, pattern_size, None)

        if found:
            # 精确化角点
            criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
            corners = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), criteria)

            # 计算棋盘位置和方向
            board_state.board_detected = True
            board_state.detection_confidence = 0.8

            # TODO: 实现棋子识别和位置计算
            board_state.board_squares = [0] * 64  # 暂时设为空棋盘
            board_state.fen_notation = "8/8/8/8/8/8/8/8 w - - 0 1"  # 空棋盘FEN

        else:
            board_state.board_detected = False
            board_state.detection_confidence = 0.0

        return board_state

    def _publish_debug_image(self):
        """发布调试图像"""
        if self.latest_image is None or not self.debug_mode:
            return

        try:
            debug_image = self.latest_image.copy()

            # 在图像上绘制检测结果
            if self.current_board_state and self.current_board_state.board_detected:
                # 绘制棋盘格子
                self._draw_board_overlay(debug_image)

                # 添加状态文本
                cv2.putText(debug_image,
                          f"检测置信度: {self.current_board_state.detection_confidence:.2f}",
                          (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)

                cv2.putText(debug_image,
                          f"棋子数量: {self.current_board_state.piece_count}",
                          (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)

            # 发布调试图像
            debug_msg = self.cv_bridge.cv2_to_imgmsg(debug_image, "bgr8")
            debug_msg.header.stamp = self.get_clock().now().to_msg()
            self.debug_image_publisher.publish(debug_msg)

        except Exception as e:
            self.get_logger().error(f'调试图像发布错误: {e}')

    def _draw_board_overlay(self, image: np.ndarray):
        """在图像上绘制棋盘网格覆盖"""

        # 简化：在图像中心绘制8x8网格
        height, width = image.shape[:2]
        center_x, center_y = width // 2, height // 2
        grid_size = min(width, height) // 3

        # 绘制网格线
        for i in range(9):
            # 垂直线
            x = center_x - grid_size//2 + (i * grid_size // 8)
            cv2.line(image, (x, center_y - grid_size//2),
                    (x, center_y + grid_size//2), (255, 255, 0), 2)

            # 水平线
            y = center_y - grid_size//2 + (i * grid_size // 8)
            cv2.line(image, (center_x - grid_size//2, y),
                    (center_x + grid_size//2, y), (255, 255, 0), 2)

    def _generate_debug_info(self, board_state: BoardState) -> str:
        """生成调试信息"""

        info_lines = [
            f"检测时间: {time.strftime('%H:%M:%S')}",
            f"棋盘检测: {'成功' if board_state.board_detected else '失败'}",
            f"置信度: {board_state.detection_confidence:.3f}",
            f"棋子数量: {board_state.piece_count}",
            f"FEN位置: {board_state.fen_notation}",
            f"轮到: {'白方' if board_state.white_to_move else '黑方'}",
        ]

        return "\n".join(info_lines)


def main(args=None):
    rclpy.init(args=args)

    try:
        vision_node = InternationalChessVisionSim()
        rclpy.spin(vision_node)
    except KeyboardInterrupt:
        pass
    finally:
        if 'vision_node' in locals():
            vision_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()