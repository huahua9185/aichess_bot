#!/usr/bin/env python3
"""
从Gazebo仿真环境收集训练数据
用于训练象棋棋子识别CNN模型
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from chess_interfaces.msg import BoardState
from cv_bridge import CvBridge
import cv2
import numpy as np
import os
import time
import json
import argparse
from datetime import datetime


class TrainingDataCollector(Node):
    """训练数据收集器"""

    def __init__(self, save_dir: str):
        super().__init__('training_data_collector')

        self.save_dir = save_dir
        self.bridge = CvBridge()

        # 创建保存目录
        os.makedirs(self.save_dir, exist_ok=True)

        # 创建数据子目录
        self.images_dir = os.path.join(self.save_dir, 'images')
        self.labels_dir = os.path.join(self.save_dir, 'labels')
        self.squares_dir = os.path.join(self.save_dir, 'squares')

        os.makedirs(self.images_dir, exist_ok=True)
        os.makedirs(self.labels_dir, exist_ok=True)
        os.makedirs(self.squares_dir, exist_ok=True)

        # 订阅
        self.image_sub = self.create_subscription(
            Image, 'camera/rgb/image_raw', self._image_callback, 10)
        self.board_state_sub = self.create_subscription(
            BoardState, 'chess/board_state', self._board_state_callback, 10)

        # 状态变量
        self.current_image = None
        self.current_board_state = None
        self.sample_count = 0
        self.collection_active = True

        # 类别标签映射
        self.piece_labels = {
            0: 'empty',      # 空格
            1: 'red_king',   # 红帅
            2: 'red_rook',   # 红车
            3: 'red_horse',  # 红马
            4: 'red_cannon', # 红炮
            5: 'red_bishop', # 红相
            6: 'red_advisor',# 红士
            7: 'red_pawn',   # 红兵
            8: 'black_king', # 黑将
            9: 'black_rook', # 黑车
            10: 'black_horse',# 黑马
            11: 'black_cannon',# 黑炮
            12: 'black_bishop',# 黑象
            13: 'black_advisor',# 黑士
            14: 'black_pawn'  # 黑卒
        }

        self.get_logger().info(f'Training data collector initialized. Saving to: {save_dir}')

    def _image_callback(self, msg):
        """图像回调"""
        try:
            self.current_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        except Exception as e:
            self.get_logger().error(f'Error converting image: {e}')

    def _board_state_callback(self, msg):
        """棋盘状态回调"""
        self.current_board_state = msg

        # 如果有图像和棋盘状态，收集数据
        if self.current_image is not None and self.collection_active:
            self._collect_sample()

    def _collect_sample(self):
        """收集一个训练样本"""
        try:
            timestamp = int(time.time() * 1000)

            # 保存整体图像
            image_filename = f'board_{timestamp}_{self.sample_count:06d}.jpg'
            image_path = os.path.join(self.images_dir, image_filename)
            cv2.imwrite(image_path, self.current_image)

            # 保存标签信息
            label_data = {
                'timestamp': timestamp,
                'sample_id': self.sample_count,
                'board_state': self.current_board_state.board_state,
                'piece_count': self.current_board_state.piece_count,
                'confidence': self.current_board_state.detection_confidence,
                'aruco_detected': self.current_board_state.aruco_detected,
                'square_positions': self._serialize_points(self.current_board_state.square_positions)
            }

            label_filename = f'board_{timestamp}_{self.sample_count:06d}.json'
            label_path = os.path.join(self.labels_dir, label_filename)

            with open(label_path, 'w') as f:
                json.dump(label_data, f, indent=2)

            # 提取单个格子图像
            self._extract_square_images(timestamp)

            self.sample_count += 1

            if self.sample_count % 10 == 0:
                self.get_logger().info(f'Collected {self.sample_count} samples')

        except Exception as e:
            self.get_logger().error(f'Error collecting sample: {e}')

    def _serialize_points(self, points):
        """序列化Point消息列表"""
        return [{'x': p.x, 'y': p.y, 'z': p.z} for p in points]

    def _extract_square_images(self, timestamp):
        """提取单个格子的图像"""
        if not self.current_board_state.square_positions:
            return

        h, w = self.current_image.shape[:2]

        # 计算格子在图像中的像素位置（简化）
        pixel_positions = self._calculate_pixel_positions()

        for i, (piece_type, pixel_pos) in enumerate(zip(
            self.current_board_state.board_state, pixel_positions)):

            if pixel_pos is None:
                continue

            # 提取格子区域
            square_img = self._extract_square_region(pixel_pos)

            if square_img is not None:
                # 按类别保存
                piece_label = self.piece_labels.get(piece_type, 'unknown')
                piece_dir = os.path.join(self.squares_dir, piece_label)
                os.makedirs(piece_dir, exist_ok=True)

                square_filename = f'{piece_label}_{timestamp}_{i:02d}.png'
                square_path = os.path.join(piece_dir, square_filename)
                cv2.imwrite(square_path, square_img)

    def _calculate_pixel_positions(self):
        """计算格子在图像中的像素位置"""
        if self.current_image is None:
            return [None] * 90

        h, w = self.current_image.shape[:2]
        positions = []

        # 假设棋盘占图像中央区域的60%
        board_w, board_h = int(w * 0.6), int(h * 0.6)
        start_x = (w - board_w) // 2
        start_y = (h - board_h) // 2

        step_x = board_w // 9  # 9列
        step_y = board_h // 10  # 10行

        for row in range(10):
            for col in range(9):
                x = start_x + col * step_x + step_x // 2
                y = start_y + row * step_y + step_y // 2
                positions.append((x, y))

        return positions

    def _extract_square_region(self, pixel_pos, square_size=64):
        """提取格子区域图像"""
        x, y = pixel_pos
        half_size = square_size // 2

        x1 = max(0, x - half_size)
        y1 = max(0, y - half_size)
        x2 = min(self.current_image.shape[1], x + half_size)
        y2 = min(self.current_image.shape[0], y + half_size)

        square_img = self.current_image[y1:y2, x1:x2]

        # 调整大小到固定尺寸
        if square_img.size > 0:
            square_img = cv2.resize(square_img, (square_size, square_size))
            return square_img

        return None

    def stop_collection(self):
        """停止数据收集"""
        self.collection_active = False
        self.get_logger().info(f'Data collection stopped. Collected {self.sample_count} samples')

    def generate_dataset_info(self):
        """生成数据集信息文件"""
        info = {
            'dataset_name': 'chess_pieces_gazebo',
            'created_at': datetime.now().isoformat(),
            'total_samples': self.sample_count,
            'classes': self.piece_labels,
            'image_size': '64x64',
            'data_source': 'Gazebo simulation',
            'directories': {
                'images': 'images/',
                'labels': 'labels/',
                'squares': 'squares/'
            }
        }

        info_path = os.path.join(self.save_dir, 'dataset_info.json')
        with open(info_path, 'w') as f:
            json.dump(info, f, indent=2)

        self.get_logger().info(f'Dataset info saved to {info_path}')


def create_training_validation_split(data_dir: str, train_ratio: float = 0.8):
    """创建训练和验证数据集分割"""
    squares_dir = os.path.join(data_dir, 'squares')

    if not os.path.exists(squares_dir):
        print("No squares directory found")
        return

    # 创建训练和验证目录
    train_dir = os.path.join(data_dir, 'train')
    val_dir = os.path.join(data_dir, 'val')

    os.makedirs(train_dir, exist_ok=True)
    os.makedirs(val_dir, exist_ok=True)

    # 处理每个类别
    for class_name in os.listdir(squares_dir):
        class_path = os.path.join(squares_dir, class_name)
        if not os.path.isdir(class_path):
            continue

        # 获取所有图像文件
        images = [f for f in os.listdir(class_path) if f.endswith('.png')]

        if not images:
            continue

        # 随机分割
        np.random.shuffle(images)
        split_idx = int(len(images) * train_ratio)

        train_images = images[:split_idx]
        val_images = images[split_idx:]

        # 创建类别目录
        train_class_dir = os.path.join(train_dir, class_name)
        val_class_dir = os.path.join(val_dir, class_name)

        os.makedirs(train_class_dir, exist_ok=True)
        os.makedirs(val_class_dir, exist_ok=True)

        # 移动文件
        for img in train_images:
            src = os.path.join(class_path, img)
            dst = os.path.join(train_class_dir, img)
            os.system(f'cp "{src}" "{dst}"')

        for img in val_images:
            src = os.path.join(class_path, img)
            dst = os.path.join(val_class_dir, img)
            os.system(f'cp "{src}" "{dst}"')

        print(f'Class {class_name}: {len(train_images)} train, {len(val_images)} val')


def main():
    parser = argparse.ArgumentParser(description='收集CNN训练数据')
    parser.add_argument('--save_dir', type=str, default='/home/jetson/chess_training_data',
                       help='数据保存目录')
    parser.add_argument('--duration', type=int, default=60,
                       help='收集时长（秒）')
    parser.add_argument('--create_split', action='store_true',
                       help='创建训练/验证分割')

    args = parser.parse_args()

    if args.create_split:
        create_training_validation_split(args.save_dir)
        return

    rclpy.init()

    try:
        collector = TrainingDataCollector(args.save_dir)

        print(f"开始收集训练数据，持续 {args.duration} 秒...")
        print("确保Gazebo仿真正在运行，并且相机话题有数据")

        start_time = time.time()
        while rclpy.ok() and (time.time() - start_time) < args.duration:
            rclpy.spin_once(collector, timeout_sec=0.1)

        collector.stop_collection()
        collector.generate_dataset_info()

        print("数据收集完成!")
        print(f"总共收集了 {collector.sample_count} 个样本")
        print(f"数据保存在: {args.save_dir}")
        print("\n使用 --create_split 选项创建训练/验证分割")

    except KeyboardInterrupt:
        print("\n数据收集被中断")
    finally:
        if 'collector' in locals():
            collector.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()