#!/usr/bin/env python3
"""
完整的象棋仿真世界启动文件
包含Gazebo世界、机械臂和相机系统
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, Command
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # 获取包路径
    pkg_chess_simulation = get_package_share_directory('chess_simulation')
    pkg_chess_description = get_package_share_directory('chess_description')

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

    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation time'
    )

    # 环境变量设置
    set_gz_resource_path = SetEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH',
        value=os.path.join(pkg_chess_simulation, 'models')
    )

    # URDF文件路径
    urdf_file = os.path.join(pkg_chess_description, 'urdf', 'dofbot_pro.urdf.xacro')

    # 机器人描述
    robot_description = Command(['xacro ', urdf_file])

    # Gazebo 仿真启动
    gazebo_sim = Node(
        package='ros_gz_sim',
        executable='gz_sim',
        arguments=[
            LaunchConfiguration('world_file'),
            '-r',  # 自动开始仿真
        ],
        condition=IfCondition(LaunchConfiguration('gui')),
        output='screen'
    )

    # 无头模式Gazebo（当gui=false时）
    gazebo_sim_headless = Node(
        package='ros_gz_sim',
        executable='gz_sim',
        arguments=[
            LaunchConfiguration('world_file'),
            '-r',  # 自动开始仿真
            '-s',  # 无GUI模式
        ],
        condition=IfCondition(LaunchConfiguration('gui')),
        output='screen'
    )

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

    # 在Gazebo中生成机器人
    spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-topic', 'robot_description',
            '-name', 'dofbot_pro',
            '-x', '0.3',  # 机器人位于棋盘旁边
            '-y', '0.0',
            '-z', '0.76',  # 桌子高度 + 基座高度
        ],
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
            '/joint_states@sensor_msgs/msg/JointState[gz.msgs.Model',
            # 机械臂关节控制话题
            '/dofbot_pro/joint_trajectory_controller/joint_trajectory@trajectory_msgs/msg/JointTrajectory]gz.msgs.JointTrajectory',
        ],
        output='screen'
    )

    # 控制器管理器
    controller_manager = Node(
        package='controller_manager',
        executable='ros2_control_node',
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
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}]
    )

    arm_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['arm_controller'],
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}]
    )

    return LaunchDescription([
        world_file_arg,
        gui_arg,
        use_sim_time_arg,
        set_gz_resource_path,
        gazebo_sim,
        robot_state_publisher,
        spawn_robot,
        gz_bridge,
        controller_manager,
        joint_state_broadcaster_spawner,
        arm_controller_spawner,
    ])