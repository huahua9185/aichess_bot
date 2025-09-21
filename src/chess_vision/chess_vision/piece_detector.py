#!/usr/bin/env python3
"""
棋子检测节点
使用计算机视觉技术识别棋盘上的棋子
支持颜色分割、形状识别和深度学习方法
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from chess_interfaces.msg import BoardState, PieceDetection
from cv_bridge import CvBridge
import cv2
import numpy as np
from typing import List, Tuple, Optional


class PieceDetector(Node):
    """棋子检测节点"""

    def __init__(self):
        super().__init__('piece_detector')

        # 参数声明
        self.declare_parameter('detection_method', 'color_shape')  # color_shape, ml, hybrid
        self.declare_parameter('detection_rate', 2.0)  # Hz
        self.declare_parameter('min_piece_area', 100)
        self.declare_parameter('max_piece_area', 5000)
        self.declare_parameter('piece_height_threshold', 0.01)  # 1cm

        # 获取参数
        self.detection_method = self.get_parameter('detection_method').value
        self.detection_rate = self.get_parameter('detection_rate').value
        self.min_piece_area = self.get_parameter('min_piece_area').value
        self.max_piece_area = self.get_parameter('max_piece_area').value
        self.piece_height_threshold = self.get_parameter('piece_height_threshold').value

        # CV Bridge
        self.bridge = CvBridge()

        # 图像数据
        self.current_rgb_image = None
        self.current_depth_image = None
        self.current_board_state = None

        # 棋子类型映射
        self.piece_types = {
            1: 'white_pawn', 2: 'white_rook', 3: 'white_knight',
            4: 'white_bishop', 5: 'white_queen', 6: 'white_king',
            -1: 'black_pawn', -2: 'black_rook', -3: 'black_knight',
            -4: 'black_bishop', -5: 'black_queen', -6: 'black_king'
        }

        # 颜色阈值 (HSV)
        self.white_piece_hsv_lower = np.array([0, 0, 200])
        self.white_piece_hsv_upper = np.array([180, 30, 255])
        self.black_piece_hsv_lower = np.array([0, 0, 0])
        self.black_piece_hsv_upper = np.array([180, 255, 50])

        # 订阅者
        self.rgb_sub = self.create_subscription(
            Image, '/camera', self.rgb_callback, 10)
        self.depth_sub = self.create_subscription(
            Image, '/depth_camera', self.depth_callback, 10)
        self.board_state_sub = self.create_subscription(
            BoardState, '/chess/board_state', self.board_state_callback, 10)

        # 发布者
        self.piece_detection_pub = self.create_publisher(
            PieceDetection, '/chess/piece_detection', 10)
        self.debug_image_pub = self.create_publisher(
            Image, '/chess/piece_debug_image', 10)

        # 定时器
        self.detection_timer = self.create_timer(
            1.0 / self.detection_rate, self.detection_callback)

        self.get_logger().info(f"棋子检测节点已启动，使用方法: {self.detection_method}")

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

    def board_state_callback(self, msg):
        """棋盘状态回调"""
        self.current_board_state = msg

    def detection_callback(self):
        """定时检测回调"""
        if (self.current_rgb_image is None or
            self.current_depth_image is None or
            self.current_board_state is None):
            return

        # 检测棋子
        detected_pieces = self.detect_pieces()

        # 发布检测结果
        self.publish_piece_detection(detected_pieces)

        # 发布调试图像
        self.publish_debug_image(detected_pieces)

    def detect_pieces(self) -> List[dict]:
        """检测棋子"""
        if self.detection_method == 'color_shape':
            return self.detect_pieces_color_shape()
        elif self.detection_method == 'ml':
            return self.detect_pieces_ml()
        elif self.detection_method == 'hybrid':
            return self.detect_pieces_hybrid()
        else:
            self.get_logger().warning(f"未知的检测方法: {self.detection_method}")
            return []

    def detect_pieces_color_shape(self) -> List[dict]:
        """基于颜色和形状的棋子检测"""
        detected_pieces = []

        # 获取棋盘ROI
        board_mask = self.get_board_mask()
        if board_mask is None:
            return detected_pieces

        # 转换到HSV颜色空间
        hsv_image = cv2.cvtColor(self.current_rgb_image, cv2.COLOR_BGR2HSV)

        # 检测白子
        white_pieces = self.detect_pieces_by_color(
            hsv_image, board_mask,
            self.white_piece_hsv_lower, self.white_piece_hsv_upper,
            is_white=True)
        detected_pieces.extend(white_pieces)

        # 检测黑子
        black_pieces = self.detect_pieces_by_color(
            hsv_image, board_mask,
            self.black_piece_hsv_lower, self.black_piece_hsv_upper,
            is_white=False)
        detected_pieces.extend(black_pieces)

        return detected_pieces

    def detect_pieces_by_color(self, hsv_image, board_mask,
                              lower_thresh, upper_thresh, is_white=True) -> List[dict]:
        """根据颜色检测棋子"""
        # 颜色阈值分割
        color_mask = cv2.inRange(hsv_image, lower_thresh, upper_thresh)

        # 结合棋盘掩码
        combined_mask = cv2.bitwise_and(color_mask, board_mask)

        # 形态学操作
        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
        combined_mask = cv2.morphologyEx(combined_mask, cv2.MORPH_CLOSE, kernel)
        combined_mask = cv2.morphologyEx(combined_mask, cv2.MORPH_OPEN, kernel)

        # 查找轮廓
        contours, _ = cv2.findContours(combined_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        detected_pieces = []
        for contour in contours:
            area = cv2.contourArea(contour)

            # 面积过滤
            if self.min_piece_area <= area <= self.max_piece_area:
                # 计算中心点
                M = cv2.moments(contour)
                if M['m00'] > 0:
                    cx = int(M['m10'] / M['m00'])
                    cy = int(M['m01'] / M['m00'])

                    # 获取对应的棋盘格
                    square_index = self.pixel_to_square(cx, cy)
                    if square_index is not None:
                        # 使用深度信息验证
                        if self.validate_piece_with_depth(cx, cy):
                            # 识别棋子类型
                            piece_type = self.classify_piece_shape(contour, is_white)

                            piece_info = {
                                'square_index': square_index,
                                'piece_type': piece_type,
                                'confidence': self.calculate_confidence(contour, area),
                                'center': (cx, cy),
                                'contour': contour,
                                'color': 'white' if is_white else 'black'
                            }
                            detected_pieces.append(piece_info)

        return detected_pieces

    def get_board_mask(self) -> Optional[np.ndarray]:
        """获取棋盘区域掩码"""
        if (self.current_board_state is None or
            len(self.current_board_state.square_positions_3d) != 64):
            return None

        # 创建棋盘掩码
        height, width = self.current_rgb_image.shape[:2]
        mask = np.zeros((height, width), dtype=np.uint8)

        # 投影64个格子的3D位置到图像平面
        board_points = []
        for i in range(0, 64, 8):  # 每行8个格子，取4个角
            for j in [0, 7]:  # 只取第一列和最后一列
                square_pos = self.current_board_state.square_positions_3d[i + j]
                pixel_pos = self.project_3d_to_pixel(square_pos)
                if pixel_pos:
                    board_points.append(pixel_pos)

        if len(board_points) >= 4:
            # 使用凸包创建棋盘掩码
            board_points = np.array(board_points, dtype=np.int32)
            hull = cv2.convexHull(board_points)
            cv2.fillPoly(mask, [hull], 255)

            # 膨胀掩码以包含边缘棋子
            kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (20, 20))
            mask = cv2.dilate(mask, kernel, iterations=1)

        return mask

    def project_3d_to_pixel(self, point_3d) -> Optional[Tuple[int, int]]:
        """将3D点投影到像素坐标"""
        # 这里需要相机内参，暂时使用简化投影
        # 实际实现中应该使用相机标定参数

        # 假设简单的透视投影
        if point_3d.z > 0:
            # 假设焦距为500像素
            fx = fy = 500
            cx = self.current_rgb_image.shape[1] // 2
            cy = self.current_rgb_image.shape[0] // 2

            u = int(fx * point_3d.x / point_3d.z + cx)
            v = int(fy * point_3d.y / point_3d.z + cy)

            if (0 <= u < self.current_rgb_image.shape[1] and
                0 <= v < self.current_rgb_image.shape[0]):
                return (u, v)

        return None

    def pixel_to_square(self, x, y) -> Optional[int]:
        """将像素坐标转换为棋盘格索引"""
        if self.current_board_state is None:
            return None

        # 找到最近的棋盘格
        min_distance = float('inf')
        closest_square = None

        for i, square_pos in enumerate(self.current_board_state.square_positions_3d):
            pixel_pos = self.project_3d_to_pixel(square_pos)
            if pixel_pos:
                distance = np.sqrt((x - pixel_pos[0])**2 + (y - pixel_pos[1])**2)
                if distance < min_distance:
                    min_distance = distance
                    closest_square = i

        # 如果距离太远，认为不在棋盘上
        if min_distance > 50:  # 50像素阈值
            return None

        return closest_square

    def validate_piece_with_depth(self, x, y) -> bool:
        """使用深度信息验证棋子存在"""
        if self.current_depth_image is None:
            return True  # 如果没有深度信息，直接通过

        # 获取深度值
        depth_value = self.current_depth_image[y, x]

        # 检查深度值是否合理（棋子应该在棋盘上方）
        if depth_value > 0 and depth_value < 2.0:  # 2米内
            return True

        return False

    def classify_piece_shape(self, contour, is_white=True) -> int:
        """根据形状分类棋子类型"""
        # 计算形状特征
        area = cv2.contourArea(contour)
        perimeter = cv2.arcLength(contour, True)

        # 计算圆形度
        if perimeter > 0:
            circularity = 4 * np.pi * area / (perimeter * perimeter)
        else:
            circularity = 0

        # 计算宽高比
        x, y, w, h = cv2.boundingRect(contour)
        aspect_ratio = float(w) / h if h > 0 else 0

        # 简化的分类逻辑
        # 实际应用中可以使用更复杂的特征或机器学习模型

        if circularity > 0.7:  # 圆形 - 可能是兵
            piece_type = 1 if is_white else -1  # 兵
        elif aspect_ratio > 1.5:  # 宽形状 - 可能是车
            piece_type = 2 if is_white else -2  # 车
        elif area > 1000:  # 大面积 - 可能是王或后
            piece_type = 6 if is_white else -6  # 王
        else:  # 其他形状
            piece_type = 3 if is_white else -3  # 马

        return piece_type

    def calculate_confidence(self, contour, area) -> float:
        """计算检测置信度"""
        # 基于轮廓质量计算置信度
        perimeter = cv2.arcLength(contour, True)
        if perimeter > 0:
            circularity = 4 * np.pi * area / (perimeter * perimeter)
            confidence = min(circularity, 1.0)
        else:
            confidence = 0.0

        return confidence

    def detect_pieces_ml(self) -> List[dict]:
        """基于机器学习的棋子检测"""
        # 这里是机器学习方法的占位符
        # 实际实现中可以使用YOLO、SSD等目标检测模型
        self.get_logger().info("机器学习检测方法尚未实现")
        return []

    def detect_pieces_hybrid(self) -> List[dict]:
        """混合检测方法"""
        # 结合颜色形状和机器学习的方法
        color_results = self.detect_pieces_color_shape()
        ml_results = self.detect_pieces_ml()

        # 融合结果
        return color_results  # 暂时只返回颜色检测结果

    def publish_piece_detection(self, detected_pieces):
        """发布棋子检测结果"""
        msg = PieceDetection()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'camera_link'

        # 初始化64格棋盘
        msg.detected_pieces = [0] * 64
        msg.confidence_scores = [0.0] * 64

        # 填充检测到的棋子
        for piece in detected_pieces:
            square_idx = piece['square_index']
            if 0 <= square_idx < 64:
                msg.detected_pieces[square_idx] = piece['piece_type']
                msg.confidence_scores[square_idx] = piece['confidence']

        msg.detection_count = len(detected_pieces)
        msg.detection_method = self.detection_method

        self.piece_detection_pub.publish(msg)

    def publish_debug_image(self, detected_pieces):
        """发布调试图像"""
        if self.current_rgb_image is None:
            return

        debug_image = self.current_rgb_image.copy()

        # 绘制检测到的棋子
        for piece in detected_pieces:
            center = piece['center']
            contour = piece['contour']
            piece_type = piece['piece_type']
            confidence = piece['confidence']

            # 绘制轮廓
            color = (0, 255, 0) if piece['color'] == 'white' else (255, 0, 0)
            cv2.drawContours(debug_image, [contour], -1, color, 2)

            # 绘制中心点
            cv2.circle(debug_image, center, 5, color, -1)

            # 添加标签
            label = f"{self.piece_types.get(piece_type, 'unknown')} ({confidence:.2f})"
            cv2.putText(debug_image, label, (center[0] - 50, center[1] - 20),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 1)

        # 添加统计信息
        info_text = f"检测到 {len(detected_pieces)} 个棋子"
        cv2.putText(debug_image, info_text, (10, 30),
                   cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)

        try:
            debug_msg = self.bridge.cv2_to_imgmsg(debug_image, 'bgr8')
            debug_msg.header.stamp = self.get_clock().now().to_msg()
            self.debug_image_pub.publish(debug_msg)
        except Exception as e:
            self.get_logger().error(f"调试图像发布失败: {e}")


def main(args=None):
    rclpy.init(args=args)
    node = PieceDetector()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()