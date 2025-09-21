#!/usr/bin/env python3
"""
象棋视觉系统启动文件
启动所有视觉处理节点
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # 获取包路径
    pkg_chess_vision = get_package_share_directory('chess_vision')

    # 参数声明
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation time'
    )

    config_file_arg = DeclareLaunchArgument(
        'config_file',
        default_value=os.path.join(pkg_chess_vision, 'config', 'vision_params.yaml'),
        description='Vision configuration file'
    )

    debug_mode_arg = DeclareLaunchArgument(
        'debug_mode',
        default_value='true',
        description='Enable debug image publishing'
    )

    detection_method_arg = DeclareLaunchArgument(
        'detection_method',
        default_value='color_shape',
        description='Piece detection method: color_shape, ml, hybrid'
    )

    # 棋盘检测节点
    board_detector_node = Node(
        package='chess_vision',
        executable='board_detector',
        name='board_detector',
        parameters=[
            LaunchConfiguration('config_file'),
            {'use_sim_time': LaunchConfiguration('use_sim_time')}
        ],
        output='screen'
    )

    # 棋子检测节点
    piece_detector_node = Node(
        package='chess_vision',
        executable='piece_detector',
        name='piece_detector',
        parameters=[
            LaunchConfiguration('config_file'),
            {'use_sim_time': LaunchConfiguration('use_sim_time')},
            {'detection_method': LaunchConfiguration('detection_method')}
        ],
        output='screen'
    )

    # 视觉协调器节点
    vision_coordinator_node = Node(
        package='chess_vision',
        executable='vision_coordinator',
        name='vision_coordinator',
        parameters=[
            LaunchConfiguration('config_file'),
            {'use_sim_time': LaunchConfiguration('use_sim_time')}
        ],
        output='screen'
    )

    # RViz可视化（调试模式）
    rviz_config_file = os.path.join(pkg_chess_vision, 'config', 'chess_vision.rviz')
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_file],
        condition=IfCondition(LaunchConfiguration('debug_mode')),
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}]
    )

    return LaunchDescription([
        use_sim_time_arg,
        config_file_arg,
        debug_mode_arg,
        detection_method_arg,
        board_detector_node,
        piece_detector_node,
        vision_coordinator_node,
        rviz_node,
    ])