#!/usr/bin/env python3
"""
Gazebo仿真世界启动文件
启动象棋机器人仿真环境
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable, IncludeLaunchDescription, ExecuteProcess
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # 获取包路径
    pkg_chess_simulation = get_package_share_directory('chess_simulation')

    # 参数声明
    world_file_arg = DeclareLaunchArgument(
        'world_file',
        default_value=os.path.join(pkg_chess_simulation, 'worlds', 'chess_world.sdf'),
        description='SDF world file to load'
    )

    gui_arg = DeclareLaunchArgument(
        'gui',
        default_value='true',
        description='Whether to start Gazebo GUI'
    )

    verbose_arg = DeclareLaunchArgument(
        'verbose',
        default_value='false',
        description='Whether to run Gazebo in verbose mode'
    )

    # 环境变量设置
    set_gz_resource_path = SetEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH',
        value=os.path.join(pkg_chess_simulation, 'models')
    )

    # Gazebo 仿真启动（有GUI）
    gazebo_sim_gui = ExecuteProcess(
        cmd=['gz', 'sim', LaunchConfiguration('world_file'), '-r'],
        condition=IfCondition(LaunchConfiguration('gui')),
        output='screen'
    )

    # Gazebo 仿真启动（无GUI）
    gazebo_sim_headless = ExecuteProcess(
        cmd=['gz', 'sim', LaunchConfiguration('world_file'), '-r', '-s'],
        condition=UnlessCondition(LaunchConfiguration('gui')),
        output='screen'
    )

    # 桥接Gazebo和ROS2的话题
    gz_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/camera@sensor_msgs/msg/Image[gz.msgs.Image',
            '/depth_camera@sensor_msgs/msg/Image[gz.msgs.Image',
            '/camera/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo',
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
        ],
        output='screen'
    )

    return LaunchDescription([
        world_file_arg,
        gui_arg,
        verbose_arg,
        set_gz_resource_path,
        gazebo_sim_gui,
        gazebo_sim_headless,
        gz_bridge,
    ])