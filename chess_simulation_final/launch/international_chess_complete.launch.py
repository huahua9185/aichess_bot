#!/usr/bin/env python3

"""
国际象棋机器人完整系统启动文件
启动所有仿真节点，实现完整的人机对弈功能
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    """生成启动描述"""

    # 声明参数
    declared_arguments = []

    declared_arguments.append(
        DeclareLaunchArgument(
            'debug_mode',
            default_value='false',
            description='启用调试模式'
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            'auto_start',
            default_value='true',
            description='自动开始游戏'
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            'human_plays_white',
            default_value='true',
            description='人类执白棋'
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            'engine_skill_level',
            default_value='10',
            description='引擎技能等级 (1-20)'
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            'use_gazebo',
            default_value='false',
            description='启用Gazebo仿真环境'
        )
    )

    # 获取参数
    debug_mode = LaunchConfiguration('debug_mode')
    auto_start = LaunchConfiguration('auto_start')
    human_plays_white = LaunchConfiguration('human_plays_white')
    engine_skill_level = LaunchConfiguration('engine_skill_level')
    use_gazebo = LaunchConfiguration('use_gazebo')

    # 节点列表
    nodes = []

    # 系统启动信息
    nodes.append(
        LogInfo(msg=['启动国际象棋机器人完整仿真系统...'])
    )

    # 1. 国际象棋引擎节点
    nodes.append(
        Node(
            package='chess_game',
            executable='international_chess_engine',
            name='国际象棋引擎',
            parameters=[{
                'engine_path': '/usr/games/stockfish',
                'default_depth': 15,
                'default_time': 5.0,
                'skill_level': engine_skill_level,
                'threads': 2,
                'memory_mb': 256,
                'simulation_mode': True
            }],
            output='screen',
            respawn=True,
            respawn_delay=2.0
        )
    )

    # 2. 仿真视觉节点
    nodes.append(
        Node(
            package='chess_vision',
            executable='vision_sim_node',
            name='国际象棋仿真视觉',
            parameters=[{
                'simulation_mode': True,
                'debug_mode': debug_mode,
                'detection_threshold': 0.9,
                'board_squares': 64,
                'camera_topic': '/camera/image_raw',
                'depth_topic': '/camera/depth/image_raw'
            }],
            output='screen',
            respawn=True,
            respawn_delay=2.0
        )
    )

    # 3. 仿真机械臂节点
    nodes.append(
        Node(
            package='chess_arm',
            executable='chess_arm_sim',
            name='国际象棋仿真机械臂',
            parameters=[{
                'simulation_mode': True,
                'max_velocity': 0.5,
                'max_acceleration': 0.3,
                'safe_height': 0.15,
                'pick_height': 0.05,
                'place_height': 0.02,
                'gripper_force': 10.0
            }],
            output='screen',
            respawn=True,
            respawn_delay=2.0
        )
    )

    # 4. 游戏协调器节点
    nodes.append(
        Node(
            package='chess_coordinator',
            executable='chess_coordinator',
            name='国际象棋协调器',
            parameters=[{
                'auto_start': auto_start,
                'human_plays_white': human_plays_white,
                'simulation_mode': True,
                'move_timeout': 300.0,  # 5分钟思考时间
                'board_detection_interval': 2.0,
                'engine_depth': 15,
                'engine_skill_level': engine_skill_level,
                'debug_mode': debug_mode
            }],
            output='screen',
            respawn=True,
            respawn_delay=2.0
        )
    )

    # 5. Gazebo仿真环境 (可选)
    nodes.append(
        Node(
            package='chess_simulation',
            executable='gazebo_world',
            name='国际象棋Gazebo世界',
            parameters=[{
                'world_file': 'international_chess_world.sdf',
                'gui': 'true',
                'verbose': debug_mode
            }],
            output='screen',
            condition=IfCondition(use_gazebo)
        )
    )

    # 6. RViz可视化 (调试模式)
    rviz_config_path = PathJoinSubstitution([
        FindPackageShare('chess_simulation'),
        'config', 'chess_robot.rviz'
    ])

    nodes.append(
        Node(
            package='rviz2',
            executable='rviz2',
            name='国际象棋RViz',
            arguments=['-d', rviz_config_path],
            parameters=[{
                'use_sim_time': True
            }],
            output='screen',
            condition=IfCondition(debug_mode)
        )
    )

    # 7. 系统状态监控节点
    nodes.append(
        Node(
            package='rqt_gui',
            executable='rqt_gui',
            name='系统监控GUI',
            arguments=['--perspective-file', PathJoinSubstitution([
                FindPackageShare('chess_simulation'),
                'config', 'chess_monitor.perspective'
            ])],
            condition=IfCondition(debug_mode)
        )
    )

    # 8. 启动信息节点
    nodes.append(
        Node(
            package='chess_simulation',
            executable='system_info_node',
            name='系统信息',
            parameters=[{
                'human_plays_white': human_plays_white,
                'engine_skill_level': engine_skill_level,
                'auto_start': auto_start,
                'simulation_mode': True
            }],
            output='screen'
        )
    )

    return LaunchDescription(declared_arguments + nodes)


if __name__ == '__main__':
    generate_launch_description()