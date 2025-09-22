#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from chess_interfaces.msg import ChessMove
from chess_interfaces.srv import PlanMove, ExecuteMove
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Pose, Point, Quaternion
import numpy as np
import threading
import time
from enum import Enum


class ArmState(Enum):
    """机械臂状态枚举"""
    IDLE = 0
    MOVING = 1
    HOLDING = 2
    ERROR = 3


class ArmController(Node):
    """DOFBOT Pro机械臂控制节点"""

    def __init__(self):
        super().__init__('arm_controller')

        # 声明参数
        self.declare_parameter('simulation_mode', True)
        self.declare_parameter('max_velocity', 0.5)
        self.declare_parameter('max_acceleration', 0.3)
        self.declare_parameter('home_position', [0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
        self.declare_parameter('safe_height', 0.15)

        # 获取参数
        self.simulation_mode = self.get_parameter('simulation_mode').value
        self.max_velocity = self.get_parameter('max_velocity').value
        self.max_acceleration = self.get_parameter('max_acceleration').value
        self.home_position = self.get_parameter('home_position').value
        self.safe_height = self.get_parameter('safe_height').value

        # 机械臂状态
        self.arm_state = ArmState.IDLE
        self.current_joint_positions = [0.0] * 6
        self.target_joint_positions = [0.0] * 6
        self.is_gripper_open = True
        self.current_pose = Pose()

        # DH参数 (DOFBOT Pro)
        self.dh_params = {
            'a': [0.0, 0.0, 0.12, 0.10, 0.0, 0.0],      # 连杆长度
            'd': [0.08, 0.0, 0.0, 0.0, 0.0, 0.04],      # 连杆偏移
            'alpha': [0.0, np.pi/2, 0.0, 0.0, np.pi/2, 0.0],  # 连杆扭转角
            'theta_offset': [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]    # 关节偏移
        }

        # 关节限制
        self.joint_limits = {
            'lower': [-np.pi, -np.pi/2, -np.pi, -np.pi, -np.pi/2, -np.pi],
            'upper': [np.pi, np.pi/2, np.pi, np.pi, np.pi/2, np.pi]
        }

        # 发布者
        self.joint_cmd_pub = self.create_publisher(
            JointTrajectory, '/dofbot_pro/joint_trajectory_controller/joint_trajectory', 10)
        self.joint_state_pub = self.create_publisher(
            JointState, '/joint_states', 10)

        # 订阅者
        self.joint_state_sub = self.create_subscription(
            JointState, '/dofbot_pro/joint_states', self._joint_state_callback, 10)

        # 服务
        self.plan_service = self.create_service(
            PlanMove, 'chess/plan_move', self._plan_move_callback)
        self.execute_service = self.create_service(
            ExecuteMove, 'chess/execute_move', self._execute_move_callback)

        # 定时器
        self.state_timer = self.create_timer(0.1, self._update_state)

        # 线程锁
        self.state_lock = threading.Lock()

        self.get_logger().info(f'Arm controller initialized in {"simulation" if self.simulation_mode else "hardware"} mode')

        # 移动到初始位置
        self._move_to_home()

    def _joint_state_callback(self, msg):
        """关节状态回调"""
        if len(msg.position) >= 6:
            self.current_joint_positions = list(msg.position[:6])
            self.current_pose = self._forward_kinematics(self.current_joint_positions)

    def _plan_move_callback(self, request, response):
        """运动规划服务回调"""
        try:
            self.get_logger().info(f'Planning move: {request.target_move.move_description}')

            # 解析移动请求
            move = request.target_move
            start_pos = self._get_board_position(move.from_index)
            end_pos = self._get_board_position(move.to_index)

            # 生成轨迹
            trajectory = self._plan_trajectory(
                start_pos, end_pos,
                request.planning_mode,
                request.velocity_scaling,
                request.pick_height,
                request.place_height,
                request.safe_height
            )

            if trajectory:
                response.success = True
                response.joint_trajectory = trajectory
                response.execution_time = self._calculate_execution_time(trajectory)
                response.path_length = self._calculate_path_length(trajectory)
                response.collision_free = True
                response.joint_limits_ok = self._check_joint_limits(trajectory)
                response.trajectory_quality = 0.8

                self.get_logger().info(f'Move planned successfully, execution time: {response.execution_time:.2f}s')
            else:
                response.success = False
                response.error_message = "Failed to plan trajectory"

        except Exception as e:
            self.get_logger().error(f'Error planning move: {str(e)}')
            response.success = False
            response.error_message = str(e)

        return response

    def _execute_move_callback(self, request, response):
        """移动执行服务回调"""
        try:
            self.get_logger().info(f'Executing move: {request.chess_move.move_description}')

            with self.state_lock:
                if self.arm_state != ArmState.IDLE:
                    response.success = False
                    response.error_message = "Arm is busy"
                    return response

                self.arm_state = ArmState.MOVING

            # 执行轨迹
            start_time = time.time()
            success = self._execute_trajectory(
                request.trajectory,
                request.velocity_scaling,
                request.wait_for_completion
            )

            execution_time = time.time() - start_time

            if success:
                response.success = True
                response.actual_duration = execution_time
                response.execution_status = ExecuteMove.Response.EXEC_SUCCESS
                response.tracking_error = 0.01  # 模拟跟踪误差
                response.final_position_error = 0.005
                response.final_orientation_error = 0.02

                self.get_logger().info(f'Move executed successfully in {execution_time:.2f}s')
            else:
                response.success = False
                response.execution_status = ExecuteMove.Response.EXEC_UNKNOWN_ERROR
                response.error_message = "Trajectory execution failed"

            with self.state_lock:
                self.arm_state = ArmState.IDLE

        except Exception as e:
            self.get_logger().error(f'Error executing move: {str(e)}')
            response.success = False
            response.error_message = str(e)
            with self.state_lock:
                self.arm_state = ArmState.IDLE

        return response

    def _plan_trajectory(self, start_pos, end_pos, planning_mode,
                        velocity_scaling, pick_height, place_height, safe_height):
        """规划轨迹"""
        try:
            # 关键点位置
            keypoints = []

            # 1. 移动到起始位置上方
            start_above = Point()
            start_above.x = start_pos.x
            start_above.y = start_pos.y
            start_above.z = start_pos.z + safe_height
            keypoints.append(start_above)

            # 2. 下降到拾取高度
            pick_pos = Point()
            pick_pos.x = start_pos.x
            pick_pos.y = start_pos.y
            pick_pos.z = start_pos.z + pick_height
            keypoints.append(pick_pos)

            # 3. 上升到安全高度
            keypoints.append(start_above)

            # 4. 移动到目标位置上方
            end_above = Point()
            end_above.x = end_pos.x
            end_above.y = end_pos.y
            end_above.z = end_pos.z + safe_height
            keypoints.append(end_above)

            # 5. 下降到放置高度
            place_pos = Point()
            place_pos.x = end_pos.x
            place_pos.y = end_pos.y
            place_pos.z = end_pos.z + place_height
            keypoints.append(place_pos)

            # 6. 上升到安全高度
            keypoints.append(end_above)

            # 将笛卡尔路径点转换为关节轨迹
            trajectory = self._cartesian_to_joint_trajectory(keypoints, velocity_scaling)

            return trajectory

        except Exception as e:
            self.get_logger().error(f'Error planning trajectory: {str(e)}')
            return None

    def _cartesian_to_joint_trajectory(self, cartesian_points, velocity_scaling):
        """将笛卡尔路径点转换为关节轨迹"""
        trajectory = JointTrajectory()
        trajectory.joint_names = [
            'joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6'
        ]

        current_time = 0.0
        dt = 0.5  # 每个点之间的时间间隔

        for i, point in enumerate(cartesian_points):
            # 逆运动学计算
            joint_positions = self._inverse_kinematics(point)

            if joint_positions is not None:
                traj_point = JointTrajectoryPoint()
                traj_point.positions = joint_positions
                traj_point.velocities = [0.0] * 6
                traj_point.accelerations = [0.0] * 6
                traj_point.time_from_start.sec = int(current_time)
                traj_point.time_from_start.nanosec = int((current_time % 1) * 1e9)

                trajectory.points.append(traj_point)
                current_time += dt

        return trajectory

    def _inverse_kinematics(self, target_pos):
        """逆运动学求解（简化版本）"""
        # 简化的逆运动学解算
        # 实际应用中应该使用更精确的算法

        # 计算基座旋转角
        joint1 = np.arctan2(target_pos.y, target_pos.x)

        # 计算其他关节角度（简化计算）
        r = np.sqrt(target_pos.x**2 + target_pos.y**2)
        z = target_pos.z - self.dh_params['d'][0]

        # 肩部和肘部角度计算
        l1 = self.dh_params['a'][2]  # 上臂长度
        l2 = self.dh_params['a'][3]  # 前臂长度

        d = np.sqrt(r**2 + z**2)
        if d > (l1 + l2):
            self.get_logger().warn("Target position out of reach")
            return None

        # 余弦定理
        cos_joint3 = (d**2 - l1**2 - l2**2) / (2 * l1 * l2)
        cos_joint3 = np.clip(cos_joint3, -1, 1)
        joint3 = np.arccos(cos_joint3)

        alpha = np.arctan2(z, r)
        beta = np.arctan2(l2 * np.sin(joint3), l1 + l2 * np.cos(joint3))
        joint2 = alpha - beta

        # 腕部关节（简化为零位置）
        joint4 = 0.0
        joint5 = -(joint2 + joint3)  # 保持末端水平
        joint6 = 0.0

        joint_angles = [joint1, joint2, joint3, joint4, joint5, joint6]

        # 检查关节限制
        for i, angle in enumerate(joint_angles):
            if angle < self.joint_limits['lower'][i] or angle > self.joint_limits['upper'][i]:
                self.get_logger().warn(f"Joint {i+1} angle {angle:.3f} exceeds limits")
                return None

        return joint_angles

    def _forward_kinematics(self, joint_positions):
        """正运动学计算"""
        # 简化的正运动学
        pose = Pose()

        # DH变换
        T = np.eye(4)
        for i in range(6):
            theta = joint_positions[i] + self.dh_params['theta_offset'][i]
            d = self.dh_params['d'][i]
            a = self.dh_params['a'][i]
            alpha = self.dh_params['alpha'][i]

            # DH变换矩阵
            T_i = np.array([
                [np.cos(theta), -np.sin(theta)*np.cos(alpha), np.sin(theta)*np.sin(alpha), a*np.cos(theta)],
                [np.sin(theta), np.cos(theta)*np.cos(alpha), -np.cos(theta)*np.sin(alpha), a*np.sin(theta)],
                [0, np.sin(alpha), np.cos(alpha), d],
                [0, 0, 0, 1]
            ])

            T = T @ T_i

        # 提取位置和姿态
        pose.position.x = T[0, 3]
        pose.position.y = T[1, 3]
        pose.position.z = T[2, 3]

        # 简化的四元数表示
        pose.orientation.w = 1.0
        pose.orientation.x = 0.0
        pose.orientation.y = 0.0
        pose.orientation.z = 0.0

        return pose

    def _get_board_position(self, board_index):
        """获取棋盘格子的3D位置"""
        row = board_index // 9
        col = board_index % 9

        pos = Point()
        # 棋盘在机械臂坐标系中的位置
        pos.x = 0.25 + col * 0.03  # 25cm起始，3cm间距
        pos.y = -0.12 + row * 0.027  # 居中，2.7cm间距
        pos.z = 0.02  # 2cm高度

        return pos

    def _execute_trajectory(self, trajectory, velocity_scaling, wait_completion):
        """执行轨迹"""
        try:
            if self.simulation_mode:
                # 仿真模式：发布轨迹命令
                trajectory.header.stamp = self.get_clock().now().to_msg()
                self.joint_cmd_pub.publish(trajectory)

                if wait_completion:
                    # 等待轨迹执行完成
                    total_time = 0.0
                    if trajectory.points:
                        last_point = trajectory.points[-1]
                        total_time = last_point.time_from_start.sec + last_point.time_from_start.nanosec * 1e-9

                    time.sleep(total_time * 1.2)  # 添加一些缓冲时间

                return True
            else:
                # 硬件模式：实际控制机械臂
                # 这里应该添加真实的硬件控制代码
                self.get_logger().warn("Hardware mode not implemented yet")
                return False

        except Exception as e:
            self.get_logger().error(f'Error executing trajectory: {str(e)}')
            return False

    def _calculate_execution_time(self, trajectory):
        """计算轨迹执行时间"""
        if not trajectory.points:
            return 0.0

        last_point = trajectory.points[-1]
        return last_point.time_from_start.sec + last_point.time_from_start.nanosec * 1e-9

    def _calculate_path_length(self, trajectory):
        """计算路径长度"""
        if len(trajectory.points) < 2:
            return 0.0

        total_length = 0.0
        for i in range(1, len(trajectory.points)):
            prev_pos = trajectory.points[i-1].positions
            curr_pos = trajectory.points[i].positions

            # 计算关节空间距离
            joint_distance = np.linalg.norm(np.array(curr_pos) - np.array(prev_pos))
            total_length += joint_distance

        return total_length

    def _check_joint_limits(self, trajectory):
        """检查关节限制"""
        for point in trajectory.points:
            for i, pos in enumerate(point.positions):
                if pos < self.joint_limits['lower'][i] or pos > self.joint_limits['upper'][i]:
                    return False
        return True

    def _move_to_home(self):
        """移动到初始位置"""
        self.get_logger().info('Moving to home position')
        home_trajectory = self._create_simple_trajectory(self.home_position, 2.0)
        if home_trajectory:
            self._execute_trajectory(home_trajectory, 0.3, True)

    def _create_simple_trajectory(self, target_positions, duration):
        """创建简单轨迹"""
        trajectory = JointTrajectory()
        trajectory.joint_names = [
            'joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6'
        ]

        # 起始点
        start_point = JointTrajectoryPoint()
        start_point.positions = self.current_joint_positions
        start_point.time_from_start.sec = 0
        trajectory.points.append(start_point)

        # 目标点
        end_point = JointTrajectoryPoint()
        end_point.positions = target_positions
        end_point.time_from_start.sec = int(duration)
        end_point.time_from_start.nanosec = int((duration % 1) * 1e9)
        trajectory.points.append(end_point)

        return trajectory

    def _update_state(self):
        """更新状态"""
        # 发布当前关节状态（仿真模式）
        if self.simulation_mode:
            joint_state = JointState()
            joint_state.header.stamp = self.get_clock().now().to_msg()
            joint_state.name = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6']
            joint_state.position = self.current_joint_positions
            joint_state.velocity = [0.0] * 6
            joint_state.effort = [0.0] * 6
            self.joint_state_pub.publish(joint_state)


def main(args=None):
    rclpy.init(args=args)

    try:
        controller = ArmController()
        rclpy.spin(controller)
    except KeyboardInterrupt:
        pass
    finally:
        if 'controller' in locals():
            controller.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()