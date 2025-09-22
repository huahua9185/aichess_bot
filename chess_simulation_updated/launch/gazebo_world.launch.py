#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import os


def generate_launch_description():
    """启动Gazebo仿真世界"""

    # 声明参数
    world_file_arg = DeclareLaunchArgument(
        'world_file',
        default_value='chess_world.sdf',
        description='Gazebo world file to load'
    )

    gui_arg = DeclareLaunchArgument(
        'gui',
        default_value='true',
        description='Start Gazebo with GUI'
    )

    verbose_arg = DeclareLaunchArgument(
        'verbose',
        default_value='false',
        description='Enable verbose output'
    )

    # 获取包路径
    pkg_chess_simulation = FindPackageShare('chess_simulation')

    # 设置Gazebo模型路径
    gazebo_model_path = PathJoinSubstitution([
        pkg_chess_simulation, 'models'
    ])

    set_gazebo_model_path = SetEnvironmentVariable(
        'GZ_SIM_RESOURCE_PATH',
        gazebo_model_path
    )

    # 世界文件路径
    world_file_path = PathJoinSubstitution([
        pkg_chess_simulation, 'worlds', LaunchConfiguration('world_file')
    ])

    # 启动Gazebo
    gz_sim = Node(
        package='ros_gz_sim',
        executable='create',
        name='gazebo',
        arguments=[
            '-world', world_file_path,
            '-name', 'chess_world',
            '-server_only', 'false' if LaunchConfiguration('gui') == 'true' else 'true',
            '-v', '4' if LaunchConfiguration('verbose') == 'true' else '1'
        ],
        output='screen',
        parameters=[{
            'use_sim_time': True
        }]
    )

    # ROS-Gazebo桥接
    bridge_config = PathJoinSubstitution([
        pkg_chess_simulation, 'config', 'bridge_config.yaml'
    ])

    ros_gz_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='ros_gz_bridge',
        arguments=[
            '--ros-args',
            '-p', f'config_file:={bridge_config}'
        ],
        output='screen'
    )

    # 机器人状态发布器
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        parameters=[{
            'robot_description': PathJoinSubstitution([
                FindPackageShare('chess_description'),
                'urdf', 'dofbot_pro.urdf.xacro'
            ]),
            'use_sim_time': True
        }],
        output='screen'
    )

    # 关节状态发布器
    joint_state_publisher = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        parameters=[{
            'use_sim_time': True
        }],
        output='screen'
    )

    return LaunchDescription([
        world_file_arg,
        gui_arg,
        verbose_arg,
        set_gazebo_model_path,
        gz_sim,
        ros_gz_bridge,
        robot_state_publisher,
        joint_state_publisher
    ])