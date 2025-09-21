#!/usr/bin/env python3
"""
象棋机器人系统集成测试脚本
测试各个核心模块的通信和协调
"""

import subprocess
import time
import signal
import sys
import os

class ChessSystemIntegrationTest:
    def __init__(self):
        self.processes = []
        self.test_results = {}

    def start_node(self, package, executable, name, params=None):
        """启动ROS2节点"""
        cmd = ['ros2', 'run', package, executable]
        if params:
            for param in params:
                cmd.extend(['--ros-args', '-p', param])

        print(f"启动节点: {name}")
        try:
            env = os.environ.copy()
            env['ROS_DOMAIN_ID'] = '0'

            process = subprocess.Popen(
                cmd,
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                env=env,
                preexec_fn=os.setsid
            )
            self.processes.append((process, name))
            time.sleep(2)  # 等待节点启动

            # 检查进程是否还在运行
            if process.poll() is None:
                print(f"✅ {name} 启动成功")
                self.test_results[name] = "SUCCESS"
                return True
            else:
                print(f"❌ {name} 启动失败")
                self.test_results[name] = "FAILED"
                return False

        except Exception as e:
            print(f"❌ {name} 启动异常: {e}")
            self.test_results[name] = "ERROR"
            return False

    def run_python_node(self, script_path, name):
        """运行Python节点"""
        cmd = ['python3', script_path]
        print(f"启动Python节点: {name}")

        try:
            env = os.environ.copy()
            env['ROS_DOMAIN_ID'] = '0'

            process = subprocess.Popen(
                cmd,
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                env=env,
                preexec_fn=os.setsid
            )
            self.processes.append((process, name))
            time.sleep(3)  # 等待节点启动

            # 检查进程是否还在运行
            if process.poll() is None:
                print(f"✅ {name} 启动成功")
                self.test_results[name] = "SUCCESS"
                return True
            else:
                stdout, stderr = process.communicate()
                if "已启动" in stderr.decode() or "已启动" in stdout.decode():
                    print(f"✅ {name} 启动成功")
                    self.test_results[name] = "SUCCESS"
                    return True
                else:
                    print(f"❌ {name} 启动失败")
                    print(f"错误输出: {stderr.decode()[:200]}")
                    self.test_results[name] = "FAILED"
                    return False

        except Exception as e:
            print(f"❌ {name} 启动异常: {e}")
            self.test_results[name] = "ERROR"
            return False

    def check_node_communication(self):
        """检查节点间通信"""
        print("\n🔍 检查节点通信...")

        # 检查话题
        try:
            result = subprocess.run(['ros2', 'topic', 'list'],
                                  capture_output=True, text=True, timeout=5)
            topics = result.stdout.strip().split('\n')

            expected_topics = [
                '/chess/board_state',
                '/chess/game_state',
                '/chess/system_status',
                '/camera/rgb/image_raw',
                '/camera/depth/image_raw'
            ]

            found_topics = []
            for topic in expected_topics:
                if topic in topics:
                    found_topics.append(topic)
                    print(f"✅ 话题存在: {topic}")
                else:
                    print(f"❌ 话题缺失: {topic}")

            self.test_results['topics'] = f"{len(found_topics)}/{len(expected_topics)}"

        except Exception as e:
            print(f"❌ 话题检查失败: {e}")
            self.test_results['topics'] = "ERROR"

    def cleanup(self):
        """清理所有进程"""
        print("\n🧹 清理进程...")
        for process, name in self.processes:
            try:
                # 发送SIGTERM到进程组
                os.killpg(os.getpgid(process.pid), signal.SIGTERM)
                process.wait(timeout=3)
                print(f"✅ {name} 已停止")
            except subprocess.TimeoutExpired:
                # 如果SIGTERM不起作用，使用SIGKILL
                os.killpg(os.getpgid(process.pid), signal.SIGKILL)
                print(f"🔪 {name} 强制停止")
            except Exception as e:
                print(f"⚠️  停止{name}时出错: {e}")

    def run_integration_test(self):
        """运行集成测试"""
        print("🚀 开始象棋机器人系统集成测试\n")

        # 设置ROS环境
        os.system("source install/setup.bash")

        try:
            # 1. 启动相机节点
            self.start_node('chess_camera', 'sim_camera_node', 'Camera')

            # 2. 启动核心Python节点
            self.run_python_node('src/chess_game/chess_game/game_engine.py', 'GameEngine')
            self.run_python_node('src/chess_game/chess_game/chess_ai.py', 'ChessAI')
            self.run_python_node('src/chess_arm/chess_arm/arm_controller.py', 'ArmController')
            self.run_python_node('src/chess_coordinator/chess_coordinator/system_coordinator.py', 'SystemCoordinator')

            # 3. 等待系统稳定
            print("\n⏱️  等待系统稳定...")
            time.sleep(5)

            # 4. 检查通信
            self.check_node_communication()

            # 5. 让系统运行一段时间
            print("\n🔄 系统运行测试中...")
            time.sleep(10)

            # 6. 测试结果
            self.print_results()

        except KeyboardInterrupt:
            print("\n⚡ 用户中断测试")
        except Exception as e:
            print(f"\n❌ 测试异常: {e}")
        finally:
            self.cleanup()

    def print_results(self):
        """打印测试结果"""
        print("\n" + "="*50)
        print("📊 集成测试结果")
        print("="*50)

        success_count = 0
        total_count = 0

        for name, result in self.test_results.items():
            total_count += 1
            if result == "SUCCESS":
                success_count += 1
                print(f"✅ {name}: {result}")
            else:
                print(f"❌ {name}: {result}")

        print("-"*50)
        print(f"📈 成功率: {success_count}/{total_count} ({success_count/total_count*100:.1f}%)")

        if success_count >= total_count * 0.8:  # 80%成功率
            print("🎉 集成测试基本通过！")
            return True
        else:
            print("⚠️  集成测试需要改进")
            return False

def main():
    """主函数"""
    # 信号处理
    def signal_handler(sig, frame):
        print('\n⚡ 收到中断信号，正在清理...')
        test.cleanup()
        sys.exit(0)

    signal.signal(signal.SIGINT, signal_handler)
    signal.signal(signal.SIGTERM, signal_handler)

    # 运行测试
    test = ChessSystemIntegrationTest()
    success = test.run_integration_test()

    sys.exit(0 if success else 1)

if __name__ == '__main__':
    main()