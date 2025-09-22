#!/usr/bin/env python3
"""
象棋机器人Web查看器启动文件
同时启动Gazebo仿真和Web服务器
"""

import os
import time
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument, SetEnvironmentVariable, IncludeLaunchDescription,
    ExecuteProcess, TimerAction, GroupAction
)
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
    gui_arg = DeclareLaunchArgument(
        'gui',
        default_value='true',
        description='Whether to start Gazebo GUI (set to false for headless mode)'
    )

    world_file_arg = DeclareLaunchArgument(
        'world_file',
        default_value=os.path.join(pkg_chess_simulation, 'worlds', 'chess_world.sdf'),
        description='SDF world file to load'
    )

    web_port_arg = DeclareLaunchArgument(
        'web_port',
        default_value='8080',
        description='Port for web server'
    )

    verbose_arg = DeclareLaunchArgument(
        'verbose',
        default_value='false',
        description='Whether to run in verbose mode'
    )

    auto_start_simulation_arg = DeclareLaunchArgument(
        'auto_start_simulation',
        default_value='true',
        description='Whether to automatically start Gazebo simulation'
    )

    # 环境变量设置
    set_ros_domain = SetEnvironmentVariable(
        name='ROS_DOMAIN_ID',
        value='0'
    )

    set_gz_resource_path = SetEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH',
        value=os.path.join(pkg_chess_simulation, 'models')
    )

    set_display = SetEnvironmentVariable(
        name='DISPLAY',
        value=':0'
    )

    # Gazebo仿真启动（有GUI）
    gazebo_sim_gui = ExecuteProcess(
        cmd=[
            'gz', 'sim', LaunchConfiguration('world_file'),
            '-r',  # 自动运行
            '-v', '4'  # 详细输出
        ],
        condition=IfCondition(LaunchConfiguration('gui')),
        output='screen',
        name='gazebo_simulation'
    )

    # Gazebo仿真启动（无GUI）
    gazebo_sim_headless = ExecuteProcess(
        cmd=[
            'gz', 'sim', LaunchConfiguration('world_file'),
            '-r', '-s',  # 无GUI运行
            '-v', '2'
        ],
        condition=UnlessCondition(LaunchConfiguration('gui')),
        output='screen',
        name='gazebo_simulation_headless'
    )

    # ROS-Gazebo桥接
    gz_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            # 相机图像桥接
            '/camera/rgb/image_raw@sensor_msgs/msg/Image[gz.msgs.Image',
            '/camera/depth/image_raw@sensor_msgs/msg/Image[gz.msgs.Image',
            '/camera/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo',
            # 时钟同步
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
            # 机械臂控制(如果需要)
            '/joint_states@sensor_msgs/msg/JointState[gz.msgs.Model',
        ],
        output='screen',
        name='ros_gz_bridge'
    )

    # 仿真相机节点（作为备选）
    sim_camera = Node(
        package='chess_camera',
        executable='sim_camera_node',
        name='sim_camera_node',
        output='screen',
        parameters=[{
            'frame_rate': 30.0,
            'image_width': 640,
            'image_height': 480,
            'camera_frame': 'camera_link'
        }]
    )

    # Web服务器启动（延迟5秒启动，等待仿真稳定）
    web_server = TimerAction(
        period=5.0,  # 延迟5秒
        actions=[
            ExecuteProcess(
                cmd=[
                    'python3',
                    os.path.join('/home/jetson/aichess/web_viewer', 'gazebo_web_server.py')
                ],
                cwd='/home/jetson/aichess',
                output='screen',
                name='web_server',
                additional_env={
                    'ROS_DOMAIN_ID': '0',
                    'PYTHONPATH': '/home/jetson/aichess/src'
                }
            )
        ]
    )

    # 系统状态监控节点（可选）
    system_monitor = TimerAction(
        period=3.0,
        actions=[
            Node(
                package='chess_coordinator',
                executable='system_coordinator',
                name='system_coordinator',
                output='screen',
                parameters=[{
                    'web_mode': True,
                    'auto_start': True
                }]
            )
        ]
    )

    # 信息显示
    info_display = ExecuteProcess(
        cmd=['echo', '''
🌐 象棋机器人Web查看器启动中...
=================================

📡 仿真环境正在启动，请稍候...
🖥️  Web界面将在5秒后可用

📱 访问地址:
   http://localhost:8080
   http://192.168.8.88:8080
   http://10.0.0.119:8080

🎮 功能特性:
   ✅ 实时Gazebo仿真视频流
   ✅ 远程仿真控制
   ✅ 系统状态监控
   ✅ 移动设备支持

⚡ 快捷键:
   Ctrl+C : 停止所有服务
   F11    : 全屏显示

=================================
'''],
        output='screen',
        name='info_display'
    )

    # 清理和退出处理
    cleanup_handler = ExecuteProcess(
        cmd=['bash', '-c', '''
        cleanup() {
            echo "🧹 清理进程中..."
            pkill -f "gz sim"
            pkill -f "gazebo_web_server.py"
            pkill -f "ros_gz_bridge"
            echo "✅ 清理完成"
            exit 0
        }
        trap cleanup SIGINT SIGTERM
        wait
        '''],
        output='screen',
        name='cleanup_handler'
    )

    return LaunchDescription([
        # 参数声明
        gui_arg,
        world_file_arg,
        web_port_arg,
        verbose_arg,
        auto_start_simulation_arg,

        # 环境变量
        set_ros_domain,
        set_gz_resource_path,
        set_display,

        # 信息显示
        info_display,

        # Gazebo仿真
        GroupAction([
            gazebo_sim_gui,
            gazebo_sim_headless,
        ], condition=IfCondition(LaunchConfiguration('auto_start_simulation'))),

        # ROS2节点
        gz_bridge,
        sim_camera,

        # Web服务器
        web_server,

        # 系统监控
        system_monitor,

        # 清理处理
        cleanup_handler,
    ])