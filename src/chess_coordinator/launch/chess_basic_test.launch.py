#!/usr/bin/env python3
"""
象棋机器人基础测试启动文件
启动核心系统进行基本功能测试
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition


def generate_launch_description():
    """生成启动描述"""

    # 声明启动参数
    simulation_mode_arg = DeclareLaunchArgument(
        'simulation_mode',
        default_value='true',
        description='是否在仿真模式下运行'
    )

    auto_start_arg = DeclareLaunchArgument(
        'auto_start',
        default_value='false',
        description='是否自动开始游戏'
    )

    log_level_arg = DeclareLaunchArgument(
        'log_level',
        default_value='info',
        description='日志级别'
    )

    # 获取参数
    simulation_mode = LaunchConfiguration('simulation_mode')
    auto_start = LaunchConfiguration('auto_start')
    log_level = LaunchConfiguration('log_level')

    return LaunchDescription([
        # 参数声明
        simulation_mode_arg,
        auto_start_arg,
        log_level_arg,

        # 启动信息
        LogInfo(msg='启动象棋机器人基础测试系统...'),

        # 系统协调器节点
        Node(
            package='chess_coordinator',
            executable='system_coordinator',
            name='system_coordinator',
            parameters=[{
                'auto_start': auto_start,
                'simulation_mode': simulation_mode,
                'health_check_interval': 5.0,
                'startup_timeout': 30.0,
                'move_timeout': 60.0,
                'max_errors_per_subsystem': 3
            }],
            arguments=['--ros-args', '--log-level', log_level],
            output='screen'
        ),

        # 相机节点（仿真模式）
        Node(
            package='chess_camera',
            executable='sim_camera_node',
            name='sim_camera_node',
            parameters=[{
                'frame_rate': 10.0,
                'image_width': 640,
                'image_height': 480
            }],
            condition=IfCondition(simulation_mode),
            arguments=['--ros-args', '--log-level', log_level],
            output='screen'
        ),

        # 游戏引擎节点
        Node(
            package='chess_game',
            executable='game_engine',
            name='game_engine',
            parameters=[{
                'game_mode': 'human_vs_ai',
                'ai_color': 'black',
                'auto_start': False
            }],
            arguments=['--ros-args', '--log-level', log_level],
            output='screen'
        ),

        # AI节点
        Node(
            package='chess_game',
            executable='chess_ai',
            name='chess_ai',
            parameters=[{
                'engine_path': '/usr/bin/stockfish',
                'default_skill_level': 10,
                'thinking_time': 3.0
            }],
            arguments=['--ros-args', '--log-level', log_level],
            output='screen'
        ),

        # 机械臂控制器节点
        Node(
            package='chess_arm',
            executable='arm_controller',
            name='arm_controller',
            parameters=[{
                'simulation_mode': simulation_mode,
                'max_velocity_scaling': 0.3,
                'max_acceleration_scaling': 0.3
            }],
            arguments=['--ros-args', '--log-level', log_level],
            output='screen'
        ),

        LogInfo(msg='象棋机器人基础测试系统启动完成')
    ])