#!/usr/bin/env python3
"""
完整的CNN训练管道
从数据收集到模型训练和评估的完整流程
"""

import os
import sys
import argparse
import subprocess
import time
import signal
from pathlib import Path

# 添加包路径
current_dir = Path(__file__).parent.parent
sys.path.append(str(current_dir))

from chess_vision.piece_recognition_model import train_model


class TrainingPipeline:
    """完整训练管道"""

    def __init__(self, base_dir: str = '/home/jetson/chess_training_data'):
        self.base_dir = Path(base_dir)
        self.model_dir = Path('/home/jetson/chess_robot_ws/src/chess_vision/models')
        self.model_path = self.model_dir / 'chess_cnn.pth'

        # 创建目录
        self.base_dir.mkdir(exist_ok=True)
        self.model_dir.mkdir(exist_ok=True)

        self.gazebo_process = None
        self.collector_process = None

    def step1_setup_simulation(self):
        """步骤1: 启动Gazebo仿真环境"""
        print("=" * 60)
        print("步骤1: 启动Gazebo仿真环境")
        print("=" * 60)

        # 检查是否已经有Gazebo运行
        check_cmd = "pgrep -f gazebo"
        result = subprocess.run(check_cmd, shell=True, capture_output=True)

        if result.returncode == 0:
            print("Gazebo仿真已经在运行")
        else:
            print("启动Gazebo仿真...")
            # 启动Gazebo仿真
            cmd = [
                'bash', '-c',
                'source /home/jetson/chess_robot_ws/install/setup.bash && '
                'ros2 launch chess_simulation chess_sim_full.launch.py'
            ]

            self.gazebo_process = subprocess.Popen(
                cmd, stdout=subprocess.PIPE, stderr=subprocess.PIPE
            )

            # 等待Gazebo启动
            print("等待Gazebo启动...")
            time.sleep(30)

        print("Gazebo仿真环境准备就绪")

    def step2_collect_data(self, duration: int = 120):
        """步骤2: 收集训练数据"""
        print("=" * 60)
        print("步骤2: 收集训练数据")
        print("=" * 60)

        data_dir = self.base_dir / 'raw_data'
        print(f"数据将保存到: {data_dir}")

        # 启动数据收集
        collect_script = current_dir / 'scripts' / 'collect_training_data.py'
        cmd = [
            'python3', str(collect_script),
            '--save_dir', str(data_dir),
            '--duration', str(duration)
        ]

        print(f"开始收集数据，持续 {duration} 秒...")
        self.collector_process = subprocess.run(cmd)

        print("数据收集完成")

    def step3_prepare_dataset(self):
        """步骤3: 准备训练数据集"""
        print("=" * 60)
        print("步骤3: 准备训练数据集")
        print("=" * 60)

        data_dir = self.base_dir / 'raw_data'

        if not data_dir.exists():
            print("未找到原始数据目录，跳过数据准备")
            return False

        # 创建训练/验证分割
        collect_script = current_dir / 'scripts' / 'collect_training_data.py'
        cmd = [
            'python3', str(collect_script),
            '--save_dir', str(data_dir),
            '--create_split'
        ]

        result = subprocess.run(cmd)

        if result.returncode == 0:
            print("数据集准备完成")
            return True
        else:
            print("数据集准备失败")
            return False

    def step4_generate_synthetic_data(self):
        """步骤4: 生成合成数据（如果需要）"""
        print("=" * 60)
        print("步骤4: 生成合成数据")
        print("=" * 60)

        # 检查是否有足够的真实数据
        train_dir = self.base_dir / 'raw_data' / 'train'
        if not train_dir.exists():
            print("未找到训练数据，生成合成数据...")

            train_script = current_dir / 'scripts' / 'train_cnn_model.py'
            cmd = [
                'python3', str(train_script),
                '--data_dir', str(self.base_dir / 'synthetic_data'),
                '--create_data'
            ]

            result = subprocess.run(cmd)

            if result.returncode == 0:
                print("合成数据生成完成")
                return str(self.base_dir / 'synthetic_data')
            else:
                print("合成数据生成失败")
                return None
        else:
            print("已有真实训练数据，跳过合成数据生成")
            return str(self.base_dir / 'raw_data')

    def step5_train_model(self, data_dir: str, epochs: int = 30):
        """步骤5: 训练CNN模型"""
        print("=" * 60)
        print("步骤5: 训练CNN模型")
        print("=" * 60)

        print(f"使用数据目录: {data_dir}")
        print(f"模型将保存到: {self.model_path}")

        try:
            train_model(
                data_dir=data_dir,
                save_path=str(self.model_path),
                epochs=epochs,
                batch_size=16,
                learning_rate=0.001
            )
            print("模型训练完成")
            return True
        except Exception as e:
            print(f"模型训练失败: {e}")
            return False

    def step6_test_model(self):
        """步骤6: 测试训练好的模型"""
        print("=" * 60)
        print("步骤6: 测试模型")
        print("=" * 60)

        if not self.model_path.exists():
            print("模型文件不存在，无法测试")
            return False

        train_script = current_dir / 'scripts' / 'train_cnn_model.py'
        cmd = [
            'python3', str(train_script),
            '--model_save_path', str(self.model_path),
            '--test_model'
        ]

        result = subprocess.run(cmd)

        if result.returncode == 0:
            print("模型测试完成")
            return True
        else:
            print("模型测试失败")
            return False

    def step7_integrate_model(self):
        """步骤7: 集成模型到系统中"""
        print("=" * 60)
        print("步骤7: 集成模型到系统")
        print("=" * 60)

        # 更新setup.py以包含新的节点
        setup_py_path = current_dir / 'setup.py'

        if setup_py_path.exists():
            print("更新setup.py配置...")
            # 这里可以添加自动更新setup.py的逻辑

        # 重新构建包
        print("重新构建ROS2包...")
        build_cmd = [
            'bash', '-c',
            'cd /home/jetson/chess_robot_ws && '
            'source /opt/ros/humble/setup.bash && '
            'colcon build --packages-select chess_vision --symlink-install'
        ]

        result = subprocess.run(build_cmd, capture_output=True, text=True)

        if result.returncode == 0:
            print("包构建成功")
            print("模型已集成到系统中")
            print(f"模型路径: {self.model_path}")
            return True
        else:
            print("包构建失败:")
            print(result.stderr)
            return False

    def cleanup(self):
        """清理资源"""
        print("清理资源...")

        if self.gazebo_process:
            try:
                self.gazebo_process.terminate()
                self.gazebo_process.wait(timeout=10)
            except:
                self.gazebo_process.kill()

    def run_full_pipeline(self, skip_simulation=False, skip_data_collection=False,
                         data_collection_duration=120, training_epochs=30):
        """运行完整训练管道"""
        print("开始CNN模型训练管道")
        print(f"工作目录: {self.base_dir}")

        try:
            # 步骤1: 启动仿真（可选）
            if not skip_simulation:
                self.step1_setup_simulation()

            # 步骤2: 收集数据（可选）
            data_dir = None
            if not skip_data_collection:
                self.step2_collect_data(data_collection_duration)
                if self.step3_prepare_dataset():
                    data_dir = str(self.base_dir / 'raw_data')

            # 步骤4: 生成合成数据（如果需要）
            if not data_dir:
                data_dir = self.step4_generate_synthetic_data()

            if not data_dir:
                print("无法获取训练数据，退出")
                return False

            # 步骤5: 训练模型
            if not self.step5_train_model(data_dir, training_epochs):
                return False

            # 步骤6: 测试模型
            if not self.step6_test_model():
                print("模型测试失败，但训练已完成")

            # 步骤7: 集成模型
            if not self.step7_integrate_model():
                print("模型集成失败，但训练已完成")

            print("=" * 60)
            print("训练管道完成!")
            print(f"模型保存位置: {self.model_path}")
            print("现在可以在增强棋盘检测器中使用训练好的模型")
            print("=" * 60)

            return True

        except KeyboardInterrupt:
            print("\n训练被中断")
            return False
        except Exception as e:
            print(f"训练管道出错: {e}")
            return False
        finally:
            self.cleanup()


def main():
    parser = argparse.ArgumentParser(description='CNN模型完整训练管道')
    parser.add_argument('--data_dir', type=str, default='/home/jetson/chess_training_data',
                       help='训练数据基础目录')
    parser.add_argument('--skip_simulation', action='store_true',
                       help='跳过Gazebo仿真启动')
    parser.add_argument('--skip_data_collection', action='store_true',
                       help='跳过数据收集，使用现有数据')
    parser.add_argument('--collection_duration', type=int, default=120,
                       help='数据收集持续时间（秒）')
    parser.add_argument('--epochs', type=int, default=30,
                       help='训练轮数')
    parser.add_argument('--quick_test', action='store_true',
                       help='快速测试模式（较少数据和训练轮数）')

    args = parser.parse_args()

    if args.quick_test:
        args.collection_duration = 30
        args.epochs = 5

    pipeline = TrainingPipeline(args.data_dir)

    success = pipeline.run_full_pipeline(
        skip_simulation=args.skip_simulation,
        skip_data_collection=args.skip_data_collection,
        data_collection_duration=args.collection_duration,
        training_epochs=args.epochs
    )

    if success:
        print("训练管道成功完成!")
        print("\n下一步:")
        print("1. 启动增强棋盘检测器:")
        print("   ros2 run chess_vision enhanced_board_detector")
        print("2. 或者在launch文件中使用enhanced_board_detector节点")
    else:
        print("训练管道失败")
        return 1

    return 0


if __name__ == '__main__':
    exit(main())