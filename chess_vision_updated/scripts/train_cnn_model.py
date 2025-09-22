#!/usr/bin/env python3
"""
CNN模型训练脚本
用于训练象棋棋子识别模型
"""

import os
import sys
import argparse
import logging
from datetime import datetime

# 添加包路径
sys.path.append('/home/jetson/chess_robot_ws/src/chess_vision')

from chess_vision.piece_recognition_model import (
    PieceRecognitionEngine,
    DataGenerator,
    train_model
)


def setup_logging():
    """设置日志"""
    logging.basicConfig(
        level=logging.INFO,
        format='%(asctime)s - %(levelname)s - %(message)s',
        handlers=[
            logging.FileHandler(f'training_{datetime.now().strftime("%Y%m%d_%H%M%S")}.log'),
            logging.StreamHandler()
        ]
    )


def create_sample_data(data_dir: str):
    """创建示例训练数据"""
    print("创建示例训练数据...")

    generator = DataGenerator(data_dir)
    generator.generate_synthetic_data(num_samples_per_class=50)

    print(f"示例数据已创建在: {data_dir}")


def main():
    parser = argparse.ArgumentParser(description='训练象棋棋子识别CNN模型')
    parser.add_argument('--data_dir', type=str, default='/home/jetson/chess_data',
                       help='训练数据目录')
    parser.add_argument('--model_save_path', type=str,
                       default='/home/jetson/chess_robot_ws/src/chess_vision/models/chess_cnn.pth',
                       help='模型保存路径')
    parser.add_argument('--epochs', type=int, default=30,
                       help='训练轮数')
    parser.add_argument('--batch_size', type=int, default=16,
                       help='批次大小')
    parser.add_argument('--learning_rate', type=float, default=0.001,
                       help='学习率')
    parser.add_argument('--create_data', action='store_true',
                       help='是否创建示例数据')
    parser.add_argument('--test_model', action='store_true',
                       help='测试训练好的模型')

    args = parser.parse_args()

    setup_logging()

    # 创建模型保存目录
    os.makedirs(os.path.dirname(args.model_save_path), exist_ok=True)

    if args.create_data:
        create_sample_data(args.data_dir)
        return

    if args.test_model:
        test_trained_model(args.model_save_path)
        return

    # 检查数据目录
    train_dir = os.path.join(args.data_dir, 'train')
    if not os.path.exists(train_dir) or len(os.listdir(train_dir)) == 0:
        print("警告: 训练数据不存在")
        print("使用 --create_data 选项创建示例数据")
        print("或者准备您自己的训练数据")
        return

    # 开始训练
    print("开始训练CNN模型...")
    print(f"数据目录: {args.data_dir}")
    print(f"模型保存路径: {args.model_save_path}")
    print(f"训练参数: epochs={args.epochs}, batch_size={args.batch_size}, lr={args.learning_rate}")

    try:
        train_model(
            data_dir=args.data_dir,
            save_path=args.model_save_path,
            epochs=args.epochs,
            batch_size=args.batch_size,
            learning_rate=args.learning_rate
        )
        print("训练完成!")

    except Exception as e:
        logging.error(f"训练过程中出现错误: {e}")
        print(f"训练失败: {e}")


def test_trained_model(model_path: str):
    """测试训练好的模型"""
    if not os.path.exists(model_path):
        print(f"模型文件不存在: {model_path}")
        return

    print("测试训练好的模型...")

    try:
        # 创建识别引擎
        engine = PieceRecognitionEngine(model_path=model_path)

        # 创建测试图像
        import cv2
        import numpy as np

        test_image = np.ones((64, 64, 3), dtype=np.uint8) * 255
        cv2.circle(test_image, (32, 32), 20, (255, 0, 0), -1)

        # 进行预测
        piece_type, confidence = engine.predict_piece(test_image)
        piece_name = engine.get_piece_name(piece_type)

        print(f"预测结果: {piece_name} (置信度: {confidence:.3f})")

        # 验证棋盘位置
        test_board = [0] * 90  # 空棋盘
        test_board[0] = 9   # 黑车
        test_board[4] = 8   # 黑将
        test_board[85] = 1  # 红帅
        test_board[89] = 2  # 红车

        is_valid, error_msg = engine.validate_chess_position(test_board)
        print(f"棋盘验证: {'有效' if is_valid else '无效'}")
        if not is_valid:
            print(f"错误: {error_msg}")

        print("模型测试完成")

    except Exception as e:
        print(f"模型测试失败: {e}")


if __name__ == '__main__':
    main()