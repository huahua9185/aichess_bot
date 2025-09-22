#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    """启动基础象棋系统节点进行测试"""

    # 声明参数
    debug_mode_arg = DeclareLaunchArgument(
        'debug_mode',
        default_value='true',
        description='Enable debug mode for all nodes'
    )

    simulation_mode_arg = DeclareLaunchArgument(
        'simulation_mode',
        default_value='true',
        description='Run in simulation mode (no real hardware)'
    )

    # 获取参数
    debug_mode = LaunchConfiguration('debug_mode')
    simulation_mode = LaunchConfiguration('simulation_mode')

    # 启动节点
    nodes = [
        # 游戏引擎节点
        Node(
            package='chess_game',
            executable='game_engine',
            name='game_engine',
            parameters=[{
                'auto_start': False,
                'difficulty_level': 3,
                'max_thinking_time': 5.0,
                'human_plays_red': True
            }],
            output='screen'
        ),

        # 棋盘检测节点
        Node(
            package='chess_vision',
            executable='board_detector',
            name='board_detector',
            parameters=[{
                'debug_mode': debug_mode,
                'detection_threshold': 0.6,
                'board_size': 90
            }],
            output='screen'
        ),

        # 系统协调器节点
        Node(
            package='chess_coordinator',
            executable='system_coordinator',
            name='system_coordinator',
            parameters=[{
                'auto_start': False,
                'detection_interval': 3.0,
                'move_timeout': 45.0
            }],
            output='screen'
        )
    ]

    # 注释掉条件相机节点，仿真模式下不启动相机
    # camera_node = Node(
    #     package='chess_camera',
    #     executable='camera_node',
    #     name='camera_node',
    #     parameters=[{
    #         'camera_index': 0,
    #         'frame_rate': 15,
    #         'width': 640,
    #         'height': 480,
    #         'auto_exposure': True
    #     }],
    #     output='screen'
    # )
    # 仿真模式下不需要相机节点

    return LaunchDescription([
        debug_mode_arg,
        simulation_mode_arg,
    ] + nodes)