#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Point, Pose, Quaternion
from chess_interfaces.msg import BoardState
from chess_interfaces.srv import DetectBoard
import cv2
from cv_bridge import CvBridge
import numpy as np
import time


class BoardDetector(Node):
    """棋盘检测和棋子识别节点"""

    def __init__(self):
        super().__init__('board_detector')

        # 初始化CV Bridge
        self.bridge = CvBridge()

        # 声明参数
        self.declare_parameter('aruco_dict_type', cv2.aruco.DICT_6X6_250)
        self.declare_parameter('board_size', 90)  # 象棋棋盘格子数
        self.declare_parameter('detection_threshold', 0.7)
        self.declare_parameter('debug_mode', False)

        # 获取参数
        self.aruco_dict_type = self.get_parameter('aruco_dict_type').value
        self.board_size = self.get_parameter('board_size').value
        self.detection_threshold = self.get_parameter('detection_threshold').value
        self.debug_mode = self.get_parameter('debug_mode').value

        # ArUco检测器
        self.aruco_dict = cv2.aruco.getPredefinedDictionary(self.aruco_dict_type)
        self.aruco_params = cv2.aruco.DetectorParameters()

        # 订阅相机图像
        self.image_sub = self.create_subscription(
            Image, 'camera/rgb/image_raw', self._image_callback, 10)
        self.depth_sub = self.create_subscription(
            Image, 'camera/depth/image_raw', self._depth_callback, 10)

        # 发布棋盘状态
        self.board_state_pub = self.create_publisher(
            BoardState, 'chess/board_state', 10)

        # 服务
        self.detect_service = self.create_service(
            DetectBoard, 'chess/detect_board', self._detect_board_callback)

        # 状态变量
        self.current_rgb = None
        self.current_depth = None
        self.last_board_state = None
        self.camera_matrix = np.array([
            [500.0, 0.0, 320.0],
            [0.0, 500.0, 240.0],
            [0.0, 0.0, 1.0]
        ])
        self.dist_coeffs = np.zeros((4, 1))

        # 象棋标准棋盘布局（初始状态）
        self.initial_board = self._create_initial_board()

        self.get_logger().info('Board detector initialized')

    def _create_initial_board(self):
        """创建象棋初始棋盘状态"""
        # 象棋90格棋盘（10x9）
        board = np.zeros(90, dtype=np.int8)

        # 红方棋子 (1-7)
        # 1=帅, 2=车, 3=马, 4=炮, 5=相, 6=士, 7=兵
        red_positions = {
            81: 2, 82: 3, 83: 5, 84: 6, 85: 1, 86: 6, 87: 5, 88: 3, 89: 2,  # 第9行
            73: 4, 77: 4,  # 第8行炮
            63: 7, 65: 7, 67: 7, 69: 7, 71: 7  # 第7行兵
        }

        # 黑方棋子 (8-14)
        # 8=将, 9=车, 10=马, 11=炮, 12=象, 13=士, 14=卒
        black_positions = {
            0: 9, 1: 10, 2: 12, 3: 13, 4: 8, 5: 13, 6: 12, 7: 10, 8: 9,  # 第0行
            16: 11, 20: 11,  # 第1行炮
            18: 14, 20: 14, 22: 14, 24: 14, 26: 14  # 第2行卒
        }

        # 设置棋子位置
        for pos, piece in red_positions.items():
            board[pos] = piece
        for pos, piece in black_positions.items():
            board[pos] = piece

        return board

    def _image_callback(self, msg):
        """RGB图像回调"""
        try:
            self.current_rgb = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        except Exception as e:
            self.get_logger().error(f'Error converting RGB image: {str(e)}')

    def _depth_callback(self, msg):
        """深度图像回调"""
        try:
            self.current_depth = self.bridge.imgmsg_to_cv2(msg, '16UC1')
        except Exception as e:
            self.get_logger().error(f'Error converting depth image: {str(e)}')

    def _detect_board_callback(self, request, response):
        """棋盘检测服务回调"""
        start_time = time.time()

        try:
            if self.current_rgb is None:
                response.success = False
                response.error_message = "No RGB image available"
                return response

            # 执行检测
            board_state = self._detect_board_state(
                force_detection=request.force_detection,
                use_depth=request.use_depth,
                detect_aruco=request.detect_aruco,
                detection_mode=request.detection_mode
            )

            if board_state is not None:
                response.success = True
                response.board_state = board_state
                response.confidence = board_state.detection_confidence
                response.board_changed = self._has_board_changed(board_state)

                # 计算检测时间
                detection_time = int((time.time() - start_time) * 1000)
                response.detection_time = detection_time

                # 发布棋盘状态
                self.board_state_pub.publish(board_state)
                self.last_board_state = board_state

                self.get_logger().info(f'Board detection completed in {detection_time}ms')
            else:
                response.success = False
                response.error_message = "Board detection failed"

        except Exception as e:
            self.get_logger().error(f'Error in board detection: {str(e)}')
            response.success = False
            response.error_message = str(e)

        return response

    def _detect_board_state(self, force_detection=False, use_depth=True,
                           detect_aruco=True, detection_mode=0):
        """检测棋盘状态"""
        try:
            board_state = BoardState()
            board_state.timestamp = self.get_clock().now().to_msg()

            # ArUco标记检测
            aruco_detected = False
            aruco_pose = Pose()

            if detect_aruco and self.current_rgb is not None:
                aruco_detected, aruco_pose = self._detect_aruco_markers()

            board_state.aruco_detected = aruco_detected
            board_state.aruco_pose = aruco_pose

            # 棋盘检测
            if aruco_detected or force_detection:
                # 检测棋盘格子
                square_positions = self._detect_board_squares()
                board_state.square_positions = square_positions

                # 识别棋子
                piece_positions = self._recognize_pieces()
                board_state.board_state = piece_positions

                # 计算置信度
                confidence = self._calculate_confidence(piece_positions)
                board_state.detection_confidence = confidence
                board_state.board_detected = confidence > self.detection_threshold

                # 统计棋子数量
                piece_count = np.count_nonzero(piece_positions)
                board_state.piece_count = piece_count

                # 棋盘朝向（简化计算）
                board_state.board_orientation = 0.0

            else:
                # 使用上次检测结果或默认状态
                if self.last_board_state is not None:
                    board_state.board_state = self.last_board_state.board_state
                    board_state.square_positions = self.last_board_state.square_positions
                else:
                    board_state.board_state = self.initial_board.tolist()
                    board_state.square_positions = self._generate_default_positions()

                board_state.detection_confidence = 0.5
                board_state.board_detected = False
                board_state.piece_count = len([p for p in board_state.board_state if p > 0])

            return board_state

        except Exception as e:
            self.get_logger().error(f'Error in board state detection: {str(e)}')
            return None

    def _detect_aruco_markers(self):
        """检测ArUco标记"""
        try:
            gray = cv2.cvtColor(self.current_rgb, cv2.COLOR_BGR2GRAY)
            corners, ids, _ = cv2.aruco.detectMarkers(
                gray, self.aruco_dict, parameters=self.aruco_params)

            if ids is not None and len(ids) > 0:
                # 估算姿态
                rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(
                    corners, 0.05, self.camera_matrix, self.dist_coeffs)

                # 构造Pose消息
                pose = Pose()
                if tvecs is not None and len(tvecs) > 0:
                    pose.position.x = float(tvecs[0][0][0])
                    pose.position.y = float(tvecs[0][0][1])
                    pose.position.z = float(tvecs[0][0][2])

                    # 简化的旋转表示
                    pose.orientation.w = 1.0

                return True, pose

        except Exception as e:
            self.get_logger().error(f'Error detecting ArUco markers: {str(e)}')

        return False, Pose()

    def _detect_board_squares(self):
        """检测棋盘格子的3D位置"""
        positions = []

        # 生成90个格子的默认位置（简化版本）
        for row in range(10):
            for col in range(9):
                point = Point()
                # 假设棋盘在机械臂坐标系中的位置
                point.x = 0.3 + col * 0.05  # 30cm起始，5cm间距
                point.y = 0.2 + row * 0.05  # 20cm起始，5cm间距
                point.z = 0.01  # 1cm高度
                positions.append(point)

        return positions

    def _recognize_pieces(self):
        """识别棋子（简化版本）"""
        # 返回初始棋盘状态（实际应该使用计算机视觉算法）
        return self.initial_board.tolist()

    def _calculate_confidence(self, piece_positions):
        """计算检测置信度"""
        # 简化的置信度计算
        total_pieces = len([p for p in piece_positions if p > 0])
        expected_pieces = 32  # 象棋标准棋子数量

        if total_pieces == 0:
            return 0.0

        confidence = min(1.0, total_pieces / expected_pieces)
        return confidence

    def _generate_default_positions(self):
        """生成默认格子位置"""
        return self._detect_board_squares()

    def _has_board_changed(self, new_state):
        """检查棋盘是否发生变化"""
        if self.last_board_state is None:
            return True

        old_board = self.last_board_state.board_state
        new_board = new_state.board_state

        return old_board != new_board


def main(args=None):
    rclpy.init(args=args)

    try:
        detector = BoardDetector()
        rclpy.spin(detector)
    except KeyboardInterrupt:
        pass
    finally:
        if 'detector' in locals():
            detector.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()