#!/usr/bin/env python3
"""
象棋机器人仿真相机节点
用于发布模拟的RGB和深度图像数据
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from std_msgs.msg import Header
from chess_interfaces.msg import BoardState
import cv2
import numpy as np
from cv_bridge import CvBridge
import time

class SimCameraNode(Node):
    """仿真相机节点"""

    def __init__(self):
        super().__init__('sim_camera_node')

        # 声明参数
        self.declare_parameter('camera_frame_id', 'camera_link')
        self.declare_parameter('publish_rate', 10.0)  # 10Hz
        self.declare_parameter('image_width', 640)
        self.declare_parameter('image_height', 480)

        # 获取参数
        self.camera_frame_id = self.get_parameter('camera_frame_id').value
        self.publish_rate = self.get_parameter('publish_rate').value
        self.image_width = self.get_parameter('image_width').value
        self.image_height = self.get_parameter('image_height').value

        # CV Bridge
        self.bridge = CvBridge()

        # 发布器
        self.rgb_pub = self.create_publisher(
            Image, 'camera/rgb/image_raw', 10)
        self.depth_pub = self.create_publisher(
            Image, 'camera/depth/image_raw', 10)
        self.camera_info_pub = self.create_publisher(
            CameraInfo, 'camera/camera_info', 10)
        self.board_state_pub = self.create_publisher(
            BoardState, 'board_state', 10)

        # 定时器
        self.timer = self.create_timer(
            1.0 / self.publish_rate, self.publish_camera_data)

        # 状态变量
        self.sequence_id = 0

        self.get_logger().info(f'仿真相机节点启动完成 - 频率: {self.publish_rate}Hz')

    def publish_camera_data(self):
        """发布相机数据"""
        try:
            # 创建时间戳
            now = self.get_clock().now().to_msg()

            # 生成模拟RGB图像（象棋棋盘）
            rgb_image = self.generate_chessboard_image()

            # 生成模拟深度图像
            depth_image = self.generate_depth_image()

            # 转换为ROS消息
            rgb_msg = self.bridge.cv2_to_imgmsg(rgb_image, 'bgr8')
            depth_msg = self.bridge.cv2_to_imgmsg(depth_image, '16UC1')

            # 设置消息头
            rgb_msg.header = Header()
            rgb_msg.header.stamp = now
            rgb_msg.header.frame_id = self.camera_frame_id

            depth_msg.header = Header()
            depth_msg.header.stamp = now
            depth_msg.header.frame_id = self.camera_frame_id

            # 发布图像
            self.rgb_pub.publish(rgb_msg)
            self.depth_pub.publish(depth_msg)

            # 发布相机信息
            camera_info_msg = self.generate_camera_info(now)
            self.camera_info_pub.publish(camera_info_msg)

            # 发布模拟棋盘状态
            board_state_msg = self.generate_board_state(now)
            self.board_state_pub.publish(board_state_msg)

            self.sequence_id += 1

            if self.sequence_id % 50 == 0:  # 每5秒打印一次
                self.get_logger().info(f'已发布 {self.sequence_id} 帧图像数据')

        except Exception as e:
            self.get_logger().error(f'发布相机数据时出错: {e}')

    def generate_chessboard_image(self):
        """生成象棋棋盘图像"""
        # 创建8x8的象棋棋盘图像
        square_size = 60
        board_size = square_size * 8

        # 创建棋盘背景
        chessboard = np.zeros((self.image_height, self.image_width, 3), dtype=np.uint8)

        # 计算棋盘在图像中的位置（居中）
        start_x = (self.image_width - board_size) // 2
        start_y = (self.image_height - board_size) // 2

        # 绘制象棋棋盘格子
        for row in range(8):
            for col in range(8):
                x1 = start_x + col * square_size
                y1 = start_y + row * square_size
                x2 = x1 + square_size
                y2 = y1 + square_size

                # 交替颜色
                if (row + col) % 2 == 0:
                    color = (240, 217, 181)  # 浅色格子
                else:
                    color = (181, 136, 99)   # 深色格子

                cv2.rectangle(chessboard, (x1, y1), (x2, y2), color, -1)
                cv2.rectangle(chessboard, (x1, y1), (x2, y2), (0, 0, 0), 1)

        # 添加一些模拟的棋子（简单的圆形）
        piece_positions = [
            (1, 1), (1, 6), (6, 1), (6, 6),  # 角落的棋子
            (3, 3), (3, 4), (4, 3), (4, 4)   # 中心的棋子
        ]

        for row, col in piece_positions:
            center_x = start_x + col * square_size + square_size // 2
            center_y = start_y + row * square_size + square_size // 2
            radius = square_size // 3

            # 随机选择白色或黑色棋子
            if (row + col) % 2 == 0:
                color = (255, 255, 255)  # 白色棋子
            else:
                color = (50, 50, 50)     # 黑色棋子

            cv2.circle(chessboard, (center_x, center_y), radius, color, -1)
            cv2.circle(chessboard, (center_x, center_y), radius, (0, 0, 0), 2)

        return chessboard

    def generate_depth_image(self):
        """生成深度图像"""
        # 创建模拟深度图像（单位：毫米）
        depth_image = np.full((self.image_height, self.image_width),
                             800, dtype=np.uint16)  # 800mm背景距离

        # 棋盘区域稍微近一些
        square_size = 60
        board_size = square_size * 8
        start_x = (self.image_width - board_size) // 2
        start_y = (self.image_height - board_size) // 2

        # 棋盘表面距离
        depth_image[start_y:start_y+board_size, start_x:start_x+board_size] = 750

        # 棋子更近一些
        piece_positions = [
            (1, 1), (1, 6), (6, 1), (6, 6),
            (3, 3), (3, 4), (4, 3), (4, 4)
        ]

        for row, col in piece_positions:
            center_x = start_x + col * square_size + square_size // 2
            center_y = start_y + row * square_size + square_size // 2
            radius = square_size // 3

            cv2.circle(depth_image, (center_x, center_y), radius, 720, -1)

        return depth_image

    def generate_camera_info(self, timestamp):
        """生成相机标定信息"""
        camera_info = CameraInfo()
        camera_info.header.stamp = timestamp
        camera_info.header.frame_id = self.camera_frame_id

        camera_info.width = self.image_width
        camera_info.height = self.image_height

        # 模拟相机内参矩阵
        fx = fy = 525.0  # 焦距
        cx = self.image_width / 2.0   # 主点x
        cy = self.image_height / 2.0  # 主点y

        camera_info.k = [fx, 0.0, cx,
                         0.0, fy, cy,
                         0.0, 0.0, 1.0]

        camera_info.d = [0.0, 0.0, 0.0, 0.0, 0.0]  # 畸变参数

        camera_info.p = [fx, 0.0, cx, 0.0,
                         0.0, fy, cy, 0.0,
                         0.0, 0.0, 1.0, 0.0]

        return camera_info

    def generate_board_state(self, timestamp):
        """生成模拟棋盘状态"""
        board_state = BoardState()
        board_state.header.stamp = timestamp
        board_state.header.frame_id = 'board_frame'

        # 初始化棋盘状态（简化的象棋初始布局）
        board_state.board_squares = [0] * 64

        # 设置一些棋子位置（简化版本）
        board_state.board_squares[0] = 3   # 黑车
        board_state.board_squares[7] = 3   # 黑车
        board_state.board_squares[56] = -3 # 白车
        board_state.board_squares[63] = -3 # 白车

        board_state.board_squares[27] = 1  # 黑王
        board_state.board_squares[35] = -1 # 白王

        # 游戏状态
        board_state.white_to_move = True
        board_state.castling_rights = [True, True, True, True]
        board_state.en_passant_square = -1
        board_state.halfmove_clock = 0
        board_state.fullmove_number = 1

        # 生成格子的3D位置（相对于棋盘中心）
        square_size = 0.08  # 8cm per square
        positions_3d = []

        from geometry_msgs.msg import Point
        for row in range(8):
            for col in range(8):
                point = Point()
                point.x = float((col - 3.5) * square_size)
                point.y = float((row - 3.5) * square_size)
                point.z = 0.0
                positions_3d.append(point)

        board_state.square_positions_3d = positions_3d

        return board_state


def main(args=None):
    """主函数"""
    rclpy.init(args=args)

    try:
        node = SimCameraNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f'节点运行出错: {e}')
    finally:
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()