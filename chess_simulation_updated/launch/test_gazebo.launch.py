#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import ExecuteProcess, DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import os

def generate_launch_description():
    """简单的Gazebo测试启动文件"""

    # 声明参数
    gui = LaunchConfiguration('gui', default='true')

    gui_arg = DeclareLaunchArgument(
        'gui',
        default_value='true',
        description='Start Gazebo with GUI'
    )

    # 启动Ignition Gazebo
    gz_sim = ExecuteProcess(
        cmd=['ign', 'gazebo', '-v', '4'],
        output='screen'
    )

    # Clock publisher
    clock_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='clock_bridge',
        arguments=[
            '/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock'
        ],
        output='screen'
    )

    return LaunchDescription([
        gui_arg,
        gz_sim,
        clock_bridge
    ])