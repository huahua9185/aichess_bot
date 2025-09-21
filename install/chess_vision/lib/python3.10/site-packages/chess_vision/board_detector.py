#!/usr/bin/env python3
"""
棋盘检测节点
使用计算机视觉技术检测和校准棋盘位置
支持ArUco标记识别和棋盘格检测
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import Point, Pose, PoseStamped
from std_msgs.msg import Header
from chess_interfaces.msg import BoardState
from chess_interfaces.srv import DetectBoard
from cv_bridge import CvBridge
import cv2
import numpy as np
import tf2_ros
from tf2_geometry_msgs import do_transform_pose
import math


class BoardDetector(Node):
    """棋盘检测节点"""

    def __init__(self):
        super().__init__('board_detector')

        # 参数声明
        self.declare_parameter('camera_topic', '/camera')
        self.declare_parameter('depth_topic', '/depth_camera')
        self.declare_parameter('camera_info_topic', '/camera/camera_info')
        self.declare_parameter('board_frame', 'board_frame')
        self.declare_parameter('camera_frame', 'camera_link')
        self.declare_parameter('detection_rate', 5.0)  # Hz
        self.declare_parameter('aruco_dict_id', cv2.aruco.DICT_6X6_250)
        self.declare_parameter('aruco_marker_size', 0.05)  # 5cm

        # 获取参数
        self.camera_topic = self.get_parameter('camera_topic').value
        self.depth_topic = self.get_parameter('depth_topic').value
        self.camera_info_topic = self.get_parameter('camera_info_topic').value
        self.board_frame = self.get_parameter('board_frame').value
        self.camera_frame = self.get_parameter('camera_frame').value
        self.detection_rate = self.get_parameter('detection_rate').value
        self.aruco_dict_id = self.get_parameter('aruco_dict_id').value
        self.aruco_marker_size = self.get_parameter('aruco_marker_size').value

        # CV Bridge
        self.bridge = CvBridge()

        # 相机参数
        self.camera_matrix = None
        self.dist_coeffs = None
        self.camera_info_received = False

        # ArUco检测器
        self.aruco_dict = cv2.aruco.getPredefinedDictionary(self.aruco_dict_id)
        self.aruco_params = cv2.aruco.DetectorParameters()

        # 棋盘状态
        self.board_detected = False
        self.board_corners = None
        self.board_pose = None
        self.square_size = 0.055  # 5.5cm每格

        # 图像数据
        self.current_rgb_image = None
        self.current_depth_image = None

        # TF2
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)

        # 订阅者
        self.rgb_sub = self.create_subscription(
            Image, self.camera_topic, self.rgb_callback, 10)
        self.depth_sub = self.create_subscription(
            Image, self.depth_topic, self.depth_callback, 10)
        self.camera_info_sub = self.create_subscription(
            CameraInfo, self.camera_info_topic, self.camera_info_callback, 10)

        # 发布者
        self.board_state_pub = self.create_publisher(
            BoardState, '/chess/board_state', 10)
        self.debug_image_pub = self.create_publisher(
            Image, '/chess/debug_image', 10)

        # 服务
        self.detect_board_service = self.create_service(
            DetectBoard, '/chess/detect_board', self.detect_board_callback)

        # 定时器
        self.detection_timer = self.create_timer(
            1.0 / self.detection_rate, self.detection_callback)

        self.get_logger().info("棋盘检测节点已启动")

    def camera_info_callback(self, msg):
        """相机信息回调"""
        self.camera_matrix = np.array(msg.k).reshape(3, 3)
        self.dist_coeffs = np.array(msg.d)
        self.camera_info_received = True
        self.get_logger().info("相机标定参数已接收")

    def rgb_callback(self, msg):
        """RGB图像回调"""
        try:
            self.current_rgb_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        except Exception as e:
            self.get_logger().error(f"RGB图像转换失败: {e}")

    def depth_callback(self, msg):
        """深度图像回调"""
        try:
            self.current_depth_image = self.bridge.imgmsg_to_cv2(msg, 'passthrough')
        except Exception as e:
            self.get_logger().error(f"深度图像转换失败: {e}")

    def detection_callback(self):
        """定时检测回调"""
        if not self.camera_info_received or self.current_rgb_image is None:
            return

        # 检测棋盘
        self.detect_chessboard()

        # 发布棋盘状态
        if self.board_detected:
            self.publish_board_state()

    def detect_chessboard(self):
        """检测棋盘位置"""
        if self.current_rgb_image is None:
            return

        gray = cv2.cvtColor(self.current_rgb_image, cv2.COLOR_BGR2GRAY)

        # 方法1: ArUco标记检测
        board_found = self.detect_aruco_markers(gray)

        # 方法2: 棋盘格检测（如果ArUco失败）
        if not board_found:
            board_found = self.detect_chessboard_corners(gray)

        # 如果检测成功，计算3D位置
        if board_found and self.current_depth_image is not None:
            self.calculate_board_pose()

        # 发布调试图像
        self.publish_debug_image()

    def detect_aruco_markers(self, gray_image):
        """使用ArUco标记检测棋盘"""
        detector = cv2.aruco.ArucoDetector(self.aruco_dict, self.aruco_params)
        corners, ids, _ = detector.detectMarkers(gray_image)

        if ids is not None and len(ids) >= 4:
            # 检测到足够的标记
            self.get_logger().info(f"检测到 {len(ids)} 个ArUco标记")

            # 假设4个角上的标记ID为0,1,2,3
            required_ids = [0, 1, 2, 3]
            detected_corners = {}

            for i, marker_id in enumerate(ids.flatten()):
                if marker_id in required_ids:
                    detected_corners[marker_id] = corners[i][0]

            if len(detected_corners) >= 4:
                # 获取棋盘四个角点
                self.board_corners = self.get_board_corners_from_aruco(detected_corners)
                self.board_detected = True
                return True

        return False

    def detect_chessboard_corners(self, gray_image):
        """使用棋盘格检测"""
        # 检测9x9的内部角点（8x8棋盘）
        board_size = (7, 7)  # 内部角点数量

        found, corners = cv2.findChessboardCorners(
            gray_image, board_size,
            cv2.CALIB_CB_ADAPTIVE_THRESH + cv2.CALIB_CB_NORMALIZE_IMAGE)

        if found:
            # 亚像素精度优化
            criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
            corners = cv2.cornerSubPix(gray_image, corners, (11, 11), (-1, -1), criteria)

            # 提取外边界角点
            self.board_corners = self.extract_board_corners(corners, board_size)
            self.board_detected = True

            self.get_logger().info("通过棋盘格检测到棋盘")
            return True

        return False

    def get_board_corners_from_aruco(self, detected_corners):
        """从ArUco标记获取棋盘角点"""
        # 按ID顺序排列：0=左上，1=右上，2=右下，3=左下
        corner_order = [0, 1, 2, 3]
        board_corners = []

        for marker_id in corner_order:
            if marker_id in detected_corners:
                # ArUco标记的中心点
                marker_center = np.mean(detected_corners[marker_id], axis=0)
                board_corners.append(marker_center)

        return np.array(board_corners, dtype=np.float32)

    def extract_board_corners(self, corners, board_size):
        """从棋盘格角点提取外边界角点"""
        corners = corners.reshape(-1, 2)
        rows, cols = board_size

        # 棋盘四个角的索引
        top_left = 0
        top_right = cols - 1
        bottom_left = (rows - 1) * cols
        bottom_right = rows * cols - 1

        board_corners = np.array([
            corners[top_left],      # 左上
            corners[top_right],     # 右上
            corners[bottom_right],  # 右下
            corners[bottom_left]    # 左下
        ], dtype=np.float32)

        return board_corners

    def calculate_board_pose(self):
        """计算棋盘在3D空间中的位置和姿态"""
        if self.board_corners is None or self.camera_matrix is None:
            return

        # 定义棋盘在其自身坐标系中的3D点（8x8棋盘，边长5.5cm）
        board_size_meters = 8 * self.square_size
        object_points = np.array([
            [0, 0, 0],                              # 左上
            [board_size_meters, 0, 0],             # 右上
            [board_size_meters, board_size_meters, 0],  # 右下
            [0, board_size_meters, 0]              # 左下
        ], dtype=np.float32)

        # 使用PnP算法求解位姿
        success, rvec, tvec = cv2.solvePnP(
            object_points, self.board_corners,
            self.camera_matrix, self.dist_coeffs)

        if success:
            # 转换为旋转矩阵
            rotation_matrix, _ = cv2.Rodrigues(rvec)

            # 创建4x4变换矩阵
            transform_matrix = np.eye(4)
            transform_matrix[:3, :3] = rotation_matrix
            transform_matrix[:3, 3] = tvec.flatten()

            # 存储位姿信息
            self.board_pose = {
                'translation': tvec.flatten(),
                'rotation': rvec.flatten(),
                'transform_matrix': transform_matrix
            }

            # 广播TF变换
            self.broadcast_board_transform()

    def broadcast_board_transform(self):
        """广播棋盘坐标系变换"""
        if self.board_pose is None:
            return

        from geometry_msgs.msg import TransformStamped
        from tf2_ros import TransformBroadcaster
        import tf_transformations

        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = self.camera_frame
        t.child_frame_id = self.board_frame

        # 设置平移
        tvec = self.board_pose['translation']
        t.transform.translation.x = float(tvec[0])
        t.transform.translation.y = float(tvec[1])
        t.transform.translation.z = float(tvec[2])

        # 设置旋转（从旋转向量转换为四元数）
        rvec = self.board_pose['rotation']
        rotation_matrix = cv2.Rodrigues(rvec)[0]
        quaternion = tf_transformations.quaternion_from_matrix(
            np.vstack([np.hstack([rotation_matrix, [[0], [0], [0]]]), [0, 0, 0, 1]]))

        t.transform.rotation.x = quaternion[0]
        t.transform.rotation.y = quaternion[1]
        t.transform.rotation.z = quaternion[2]
        t.transform.rotation.w = quaternion[3]

        self.tf_broadcaster.sendTransform(t)

    def calculate_square_positions(self):
        """计算64个棋盘格的3D位置"""
        if self.board_pose is None:
            return None

        square_positions = []
        transform_matrix = self.board_pose['transform_matrix']

        # 计算每个格子的中心位置
        for row in range(8):
            for col in range(8):
                # 格子中心在棋盘坐标系中的位置
                x = (col + 0.5) * self.square_size
                y = (row + 0.5) * self.square_size
                z = 0.0

                # 转换到相机坐标系
                board_point = np.array([x, y, z, 1])
                camera_point = transform_matrix @ board_point

                # 创建Point消息
                point = Point()
                point.x = float(camera_point[0])
                point.y = float(camera_point[1])
                point.z = float(camera_point[2])
                square_positions.append(point)

        return square_positions

    def publish_board_state(self):
        """发布棋盘状态"""
        if not self.board_detected:
            return

        msg = BoardState()
        msg.header = Header()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.camera_frame

        # 初始化棋盘状态（空棋盘）
        msg.board_squares = [0] * 64  # 0表示空格

        # 计算格子3D位置
        square_positions = self.calculate_square_positions()
        if square_positions:
            msg.square_positions_3d = square_positions
        else:
            msg.square_positions_3d = [Point()] * 64

        # 游戏状态（初始化为标准开局）
        msg.white_to_move = True
        msg.castling_rights = [True, True, True, True]  # KQkq
        msg.en_passant_square = -1
        msg.halfmove_clock = 0
        msg.fullmove_number = 1

        self.board_state_pub.publish(msg)

    def publish_debug_image(self):
        """发布调试图像"""
        if self.current_rgb_image is None:
            return

        debug_image = self.current_rgb_image.copy()

        # 绘制检测到的棋盘角点
        if self.board_corners is not None:
            for i, corner in enumerate(self.board_corners):
                cv2.circle(debug_image, tuple(corner.astype(int)), 10, (0, 255, 0), -1)
                cv2.putText(debug_image, str(i), tuple(corner.astype(int)),
                           cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)

            # 绘制棋盘边界
            cv2.polylines(debug_image, [self.board_corners.astype(int)], True, (0, 255, 0), 3)

        # 添加状态文本
        status_text = "棋盘已检测" if self.board_detected else "正在检测棋盘"
        cv2.putText(debug_image, status_text, (10, 30),
                   cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)

        try:
            debug_msg = self.bridge.cv2_to_imgmsg(debug_image, 'bgr8')
            debug_msg.header.stamp = self.get_clock().now().to_msg()
            self.debug_image_pub.publish(debug_msg)
        except Exception as e:
            self.get_logger().error(f"调试图像发布失败: {e}")

    def detect_board_callback(self, request, response):
        """检测棋盘服务回调"""
        self.get_logger().info("收到棋盘检测请求")

        # 强制执行一次检测
        if self.current_rgb_image is not None:
            self.detect_chessboard()

        response.success = self.board_detected
        response.message = "棋盘检测成功" if self.board_detected else "棋盘检测失败"

        if self.board_detected and self.board_pose is not None:
            # 返回棋盘位姿
            pose = Pose()
            tvec = self.board_pose['translation']
            rvec = self.board_pose['rotation']

            pose.position.x = float(tvec[0])
            pose.position.y = float(tvec[1])
            pose.position.z = float(tvec[2])

            # 转换旋转向量为四元数
            import tf_transformations
            rotation_matrix = cv2.Rodrigues(rvec)[0]
            quaternion = tf_transformations.quaternion_from_matrix(
                np.vstack([np.hstack([rotation_matrix, [[0], [0], [0]]]), [0, 0, 0, 1]]))

            pose.orientation.x = quaternion[0]
            pose.orientation.y = quaternion[1]
            pose.orientation.z = quaternion[2]
            pose.orientation.w = quaternion[3]

            response.board_pose = pose

        return response


def main(args=None):
    rclpy.init(args=args)
    node = BoardDetector()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()