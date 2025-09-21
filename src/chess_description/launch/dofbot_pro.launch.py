#!/usr/bin/env python3
"""
DOFBOT Pro机械臂启动文件
可在仿真或真实硬件模式下运行
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, Command
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # 获取包路径
    pkg_chess_description = get_package_share_directory('chess_description')

    # 参数声明
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation time'
    )

    simulation_mode_arg = DeclareLaunchArgument(
        'simulation_mode',
        default_value='true',
        description='Whether to run in simulation mode'
    )

    # URDF文件路径
    urdf_file = os.path.join(pkg_chess_description, 'urdf', 'dofbot_pro.urdf.xacro')

    # 机器人描述
    robot_description = Command(['xacro ', urdf_file])

    # Robot State Publisher
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[
            {'robot_description': robot_description},
            {'use_sim_time': LaunchConfiguration('use_sim_time')}
        ]
    )

    # Joint State Publisher (仅在非仿真模式下)
    joint_state_publisher = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        condition=UnlessCondition(LaunchConfiguration('simulation_mode')),
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}]
    )

    # Joint State Publisher GUI (调试用)
    joint_state_publisher_gui = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui',
        condition=UnlessCondition(LaunchConfiguration('simulation_mode')),
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}]
    )

    # 控制器管理器 (仅在仿真模式下)
    controller_manager = Node(
        package='controller_manager',
        executable='ros2_control_node',
        condition=IfCondition(LaunchConfiguration('simulation_mode')),
        parameters=[
            {'robot_description': robot_description},
            os.path.join(pkg_chess_description, 'config', 'dofbot_pro_controllers.yaml'),
            {'use_sim_time': LaunchConfiguration('use_sim_time')}
        ],
        output='screen'
    )

    # 启动控制器
    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster'],
        condition=IfCondition(LaunchConfiguration('simulation_mode')),
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}]
    )

    arm_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['arm_controller'],
        condition=IfCondition(LaunchConfiguration('simulation_mode')),
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}]
    )

    return LaunchDescription([
        use_sim_time_arg,
        simulation_mode_arg,
        robot_state_publisher,
        joint_state_publisher,
        joint_state_publisher_gui,
        controller_manager,
        joint_state_broadcaster_spawner,
        arm_controller_spawner,
    ])