#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node

def generate_launch_description():
    """简单的Gazebo测试启动文件"""

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
        gz_sim,
        clock_bridge
    ])