#!/usr/bin/env python3
"""
测试CNN模型基本功能
"""

import sys
import os
import numpy as np
import cv2

# 添加路径
sys.path.append('/home/jetson/chess_robot_ws/src/chess_vision')

def test_cnn_import():
    """测试模型导入"""
    try:
        from chess_vision.piece_recognition_model import (
            ChessPieceCNN, PieceRecognitionEngine, DataGenerator
        )
        print("✓ CNN模型导入成功")
        return True
    except Exception as e:
        print(f"✗ CNN模型导入失败: {e}")
        return False

def test_model_creation():
    """测试模型创建"""
    try:
        from chess_vision.piece_recognition_model import ChessPieceCNN

        # 测试CPU模式
        model = ChessPieceCNN(num_classes=15, input_size=64)
        print("✓ CNN模型创建成功")
        print(f"  模型参数数量: {sum(p.numel() for p in model.parameters())}")
        return True
    except Exception as e:
        print(f"✗ CNN模型创建失败: {e}")
        return False

def test_recognition_engine():
    """测试识别引擎"""
    try:
        from chess_vision.piece_recognition_model import PieceRecognitionEngine

        # 使用CPU
        engine = PieceRecognitionEngine(model_path=None, device='cpu')
        print("✓ 识别引擎创建成功")

        # 创建测试图像
        test_image = np.ones((64, 64, 3), dtype=np.uint8) * 255
        cv2.circle(test_image, (32, 32), 20, (255, 0, 0), -1)

        # 测试预测
        piece_type, confidence = engine.predict_piece(test_image)
        print(f"✓ 预测测试成功: 类型={piece_type}, 置信度={confidence:.3f}")

        # 测试棋盘验证
        test_board = [0] * 90
        test_board[4] = 8   # 黑将
        test_board[85] = 1  # 红帅

        is_valid, error_msg = engine.validate_chess_position(test_board)
        print(f"✓ 棋盘验证测试: 有效={is_valid}")

        return True
    except Exception as e:
        print(f"✗ 识别引擎测试失败: {e}")
        return False

def test_data_generator():
    """测试数据生成器"""
    try:
        from chess_vision.piece_recognition_model import DataGenerator

        test_dir = '/tmp/test_chess_data'
        generator = DataGenerator(test_dir)
        print("✓ 数据生成器创建成功")

        # 清理测试目录
        os.system(f'rm -rf {test_dir}')
        return True
    except Exception as e:
        print(f"✗ 数据生成器测试失败: {e}")
        return False

def check_dependencies():
    """检查依赖"""
    print("检查依赖包...")

    deps = [
        ('torch', 'PyTorch'),
        ('torchvision', 'TorchVision'),
        ('cv2', 'OpenCV'),
        ('numpy', 'NumPy')
    ]

    all_ok = True
    for module, name in deps:
        try:
            __import__(module)
            print(f"✓ {name} 可用")
        except ImportError:
            print(f"✗ {name} 未安装")
            all_ok = False

    return all_ok

def main():
    print("=" * 50)
    print("CNN模型功能测试")
    print("=" * 50)

    tests = [
        ("依赖检查", check_dependencies),
        ("模型导入测试", test_cnn_import),
        ("模型创建测试", test_model_creation),
        ("识别引擎测试", test_recognition_engine),
        ("数据生成器测试", test_data_generator)
    ]

    passed = 0
    total = len(tests)

    for test_name, test_func in tests:
        print(f"\n{test_name}...")
        try:
            if test_func():
                passed += 1
            else:
                print(f"测试 '{test_name}' 失败")
        except Exception as e:
            print(f"测试 '{test_name}' 出错: {e}")

    print("\n" + "=" * 50)
    print(f"测试结果: {passed}/{total} 通过")

    if passed == total:
        print("✓ 所有测试通过! CNN模块准备就绪")
        print("\n下一步:")
        print("1. 运行训练管道: python3 training_pipeline.py --quick_test")
        print("2. 或者创建合成数据: python3 train_cnn_model.py --create_data")
    else:
        print("✗ 部分测试失败，请检查依赖和代码")

    print("=" * 50)

if __name__ == '__main__':
    main()