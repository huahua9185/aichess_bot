#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    """启动完整的国际象棋机器人仿真系统"""

    # 声明参数
    debug_mode_arg = DeclareLaunchArgument(
        'debug_mode',
        default_value='true',
        description='Enable debug mode'
    )

    difficulty_level_arg = DeclareLaunchArgument(
        'difficulty_level',
        default_value='3',
        description='AI difficulty level (1-10)'
    )

    auto_start_arg = DeclareLaunchArgument(
        'auto_start',
        default_value='false',
        description='Auto start the game'
    )

    # 获取参数
    debug_mode = LaunchConfiguration('debug_mode')
    difficulty_level = LaunchConfiguration('difficulty_level')
    auto_start = LaunchConfiguration('auto_start')

    # 启动Gazebo世界
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('chess_simulation'),
                'launch', 'gazebo_world.launch.py'
            ])
        ]),
        launch_arguments={
            'world_file': 'international_chess_world.sdf',
            'gui': 'true',
            'verbose': debug_mode
        }.items()
    )

    # 机械臂控制节点
    arm_controller = Node(
        package='chess_arm',
        executable='arm_controller',
        name='arm_controller',
        parameters=[{
            'simulation_mode': True,
            'max_velocity': 0.5,
            'max_acceleration': 0.3,
            'safe_height': 0.15
        }],
        output='screen'
    )

    # 视觉处理节点 (国际象棋仿真)
    board_detector = Node(
        package='chess_vision',
        executable='vision_sim_node',
        name='chess_vision_sim',
        parameters=[{
            'debug_mode': debug_mode,
            'detection_threshold': 0.8,
            'board_squares': 64,  # 国际象棋64格
            'simulation_mode': True
        }],
        output='screen'
    )

    # 国际象棋Stockfish引擎节点
    stockfish_engine = Node(
        package='chess_game',
        executable='stockfish_engine',
        name='stockfish_engine',
        parameters=[{
            'engine_path': '/usr/games/stockfish',
            'depth': 15,
            'time_limit': 5.0,
            'skill_level': difficulty_level,
            'chess_variant': 'standard',  # 标准国际象棋
            'simulation_mode': True
        }],
        output='screen'
    )

    # 国际象棋游戏协调器节点
    chess_coordinator = Node(
        package='chess_coordinator',
        executable='chess_coordinator',
        name='chess_coordinator',
        parameters=[{
            'auto_start': auto_start,
            'human_plays_white': True,
            'simulation_mode': True,
            'move_timeout': 300.0,  # 5分钟思考时间
            'board_detection_interval': 2.0
        }],
        output='screen'
    )

    # RViz可视化（可选）
    rviz_config = PathJoinSubstitution([
        FindPackageShare('chess_simulation'),
        'config', 'chess_robot.rviz'
    ])

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config],
        parameters=[{
            'use_sim_time': True
        }],
        output='screen'
    )

    return LaunchDescription([
        debug_mode_arg,
        difficulty_level_arg,
        auto_start_arg,
        gazebo_launch,
        arm_controller,
        board_detector,
        stockfish_engine,
        chess_coordinator,
        # rviz  # 取消注释以启用RViz
    ])