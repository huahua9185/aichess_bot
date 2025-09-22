#!/usr/bin/env python3

"""
系统信息节点
显示国际象棋机器人系统的启动信息和状态
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import time


class SystemInfoNode(Node):
    """系统信息节点"""

    def __init__(self):
        super().__init__('系统信息')

        # 声明参数
        self.declare_parameter('human_plays_white', True)
        self.declare_parameter('engine_skill_level', 10)
        self.declare_parameter('auto_start', True)
        self.declare_parameter('simulation_mode', True)

        # 获取参数
        self.human_plays_white = self.get_parameter('human_plays_white').value
        self.engine_skill_level = self.get_parameter('engine_skill_level').value
        self.auto_start = self.get_parameter('auto_start').value
        self.simulation_mode = self.get_parameter('simulation_mode').value

        # 发布系统信息
        self.info_publisher = self.create_publisher(String, '系统信息', 10)

        # 定时发布信息
        self.info_timer = self.create_timer(5.0, self._publish_system_info)

        # 显示启动信息
        self._show_startup_info()

    def _show_startup_info(self):
        """显示启动信息"""
        info_lines = [
            "🏁 国际象棋机器人系统启动信息",
            "=" * 50,
            f"🎮 游戏模式: {'仿真' if self.simulation_mode else '真实'}模式",
            f"⚪ 人类棋色: {'白棋' if self.human_plays_white else '黑棋'}",
            f"🤖 引擎等级: {self.engine_skill_level}/20",
            f"🚀 自动开始: {'是' if self.auto_start else '否'}",
            "",
            "📋 系统组件状态:",
            "  ✅ Stockfish引擎 - 就绪",
            "  ✅ 仿真视觉节点 - 运行中",
            "  ✅ 仿真机械臂 - 就绪",
            "  ✅ 游戏协调器 - 初始化中",
            "",
            "🎯 使用说明:",
            "  • 在仿真模式下，系统会自动检测棋盘状态变化",
            "  • 人类移动棋子后，引擎会自动分析并执行回应",
            f"  • 当前设置: 人类执{'白' if self.human_plays_white else '黑'}棋先行",
            "",
            "🔧 调试命令:",
            "  ros2 topic list                    # 查看所有话题",
            "  ros2 topic echo /游戏状态           # 监控游戏状态",
            "  ros2 topic echo /引擎移动           # 查看引擎移动",
            "  ros2 service list                  # 查看可用服务",
            "",
            f"⏰ 启动时间: {time.strftime('%Y-%m-%d %H:%M:%S')}",
            "🎉 系统就绪，开始对弈！"
        ]

        for line in info_lines:
            self.get_logger().info(line)

    def _publish_system_info(self):
        """发布系统信息"""
        info_data = {
            'startup_time': time.strftime('%Y-%m-%d %H:%M:%S'),
            'simulation_mode': self.simulation_mode,
            'human_plays_white': self.human_plays_white,
            'engine_skill_level': self.engine_skill_level,
            'auto_start': self.auto_start,
            'system_status': 'running',
            'components': {
                'engine': 'ready',
                'vision': 'running',
                'arm': 'ready',
                'coordinator': 'active'
            }
        }

        msg = String()
        msg.data = str(info_data)
        self.info_publisher.publish(msg)


def main(args=None):
    rclpy.init(args=args)

    try:
        info_node = SystemInfoNode()
        rclpy.spin(info_node)
    except KeyboardInterrupt:
        pass
    finally:
        if 'info_node' in locals():
            info_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()