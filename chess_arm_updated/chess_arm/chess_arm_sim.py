#!/usr/bin/env python3

"""
国际象棋仿真机械臂控制节点
在仿真环境中模拟DOFBOT Pro机械臂的运动和抓取操作
"""

import rclpy
from rclpy.node import Node
import numpy as np
import time
import threading
from typing import Dict, List, Tuple, Optional

from chess_interfaces.msg import ChessMove
from chess_interfaces.srv import PlanMove, ExecuteMove
from geometry_msgs.msg import Point, Pose, PoseStamped
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray


class ChessArmSim(Node):
    """国际象棋仿真机械臂控制节点"""

    def __init__(self):
        super().__init__('国际象棋仿真机械臂')

        # 声明参数
        self.declare_parameter('simulation_mode', True)
        self.declare_parameter('max_velocity', 0.5)
        self.declare_parameter('max_acceleration', 0.3)
        self.declare_parameter('safe_height', 0.15)
        self.declare_parameter('pick_height', 0.05)
        self.declare_parameter('place_height', 0.02)
        self.declare_parameter('gripper_force', 10.0)

        # 获取参数
        self.simulation_mode = self.get_parameter('simulation_mode').value
        self.max_velocity = self.get_parameter('max_velocity').value
        self.max_acceleration = self.get_parameter('max_acceleration').value
        self.safe_height = self.get_parameter('safe_height').value
        self.pick_height = self.get_parameter('pick_height').value
        self.place_height = self.get_parameter('place_height').value
        self.gripper_force = self.get_parameter('gripper_force').value

        # 机械臂关节配置 (DOFBOT Pro 6DOF)
        self.joint_names = [
            'base_joint',
            'shoulder_joint',
            'elbow_joint',
            'wrist1_joint',
            'wrist2_joint',
            'gripper_joint'
        ]

        # 关节限制 (弧度)
        self.joint_limits = {
            'base_joint': (-np.pi, np.pi),
            'shoulder_joint': (-np.pi/2, np.pi/2),
            'elbow_joint': (-np.pi/2, np.pi/2),
            'wrist1_joint': (-np.pi, np.pi),
            'wrist2_joint': (-np.pi/2, np.pi/2),
            'gripper_joint': (0.0, 0.04)  # 夹爪开合距离
        }

        # 当前关节状态
        self.current_joint_positions = [0.0] * 6
        self.current_joint_velocities = [0.0] * 6
        self.target_joint_positions = [0.0] * 6
        self.is_moving = False
        self.movement_lock = threading.Lock()

        # 棋盘坐标系配置
        self.board_center = Point()
        self.board_center.x = 0.0  # 相对于机械臂base_link
        self.board_center.y = 0.0
        self.board_center.z = 0.02  # 棋盘表面高度
        self.square_size = 0.05  # 5cm每格

        # 服务
        self.plan_service = self.create_service(
            PlanMove, '规划移动', self._plan_move_callback
        )

        self.execute_service = self.create_service(
            ExecuteMove, '执行移动', self._execute_move_callback
        )

        # 发布器
        self.joint_state_publisher = self.create_publisher(
            JointState, 'joint_states', 10
        )

        self.joint_command_publisher = self.create_publisher(
            Float64MultiArray, '/arm_controller/commands', 10
        )

        # 关节状态发布定时器
        self.joint_timer = self.create_timer(0.1, self._publish_joint_states)

        # 运动控制定时器
        self.motion_timer = self.create_timer(0.02, self._update_motion)

        self.get_logger().info('国际象棋仿真机械臂控制器已启动')

    def _plan_move_callback(self, request: PlanMove.Request,
                          response: PlanMove.Response) -> PlanMove.Response:
        """移动规划服务回调"""

        try:
            chess_move = request.chess_move
            if not chess_move:
                response.success = False
                response.error_message = "无效的移动请求"
                return response

            # 解析移动
            from_square = chess_move.from_square_index
            to_square = chess_move.to_square_index

            # 计算3D位置
            from_pos = self._square_to_position(from_square)
            to_pos = self._square_to_position(to_square)

            # 生成移动轨迹
            trajectory = self._plan_chess_move_trajectory(
                from_pos, to_pos, chess_move.move_type
            )

            if trajectory:
                response.success = True
                response.joint_trajectory = trajectory
                response.execution_time = self._estimate_execution_time(trajectory)
                response.waypoint_count = len(trajectory.points)

                self.get_logger().info(
                    f'规划成功: {chess_move.from_square} -> {chess_move.to_square}'
                )
            else:
                response.success = False
                response.error_message = "路径规划失败"

        except Exception as e:
            response.success = False
            response.error_message = f"规划过程出错: {str(e)}"
            self.get_logger().error(f'移动规划失败: {e}')

        return response

    def _execute_move_callback(self, request: ExecuteMove.Request,
                             response: ExecuteMove.Response) -> ExecuteMove.Response:
        """移动执行服务回调"""

        if self.is_moving:
            response.success = False
            response.error_message = "机械臂正在执行其他动作"
            return response

        try:
            trajectory = request.trajectory
            if not trajectory or not trajectory.points:
                response.success = False
                response.error_message = "无效的轨迹数据"
                return response

            # 执行轨迹
            start_time = time.time()
            success = self._execute_trajectory(trajectory)

            if success:
                response.success = True
                response.actual_duration = time.time() - start_time

                self.get_logger().info(
                    f'移动执行成功，用时: {response.actual_duration:.2f}秒'
                )
            else:
                response.success = False
                response.error_message = "轨迹执行失败"

        except Exception as e:
            response.success = False
            response.error_message = f"执行过程出错: {str(e)}"
            self.get_logger().error(f'移动执行失败: {e}')

        return response

    def _square_to_position(self, square_index: int) -> Point:
        """将棋盘格子索引转换为3D位置"""
        if not (0 <= square_index <= 63):
            raise ValueError(f"无效的格子索引: {square_index}")

        # 计算行列
        rank = square_index // 8  # 0-7
        file = square_index % 8   # 0-7

        position = Point()
        position.x = self.board_center.x + (-0.175 + file * self.square_size)
        position.y = self.board_center.y + (-0.175 + (7-rank) * self.square_size)
        position.z = self.board_center.z

        return position

    def _plan_chess_move_trajectory(self, from_pos: Point, to_pos: Point,
                                  move_type: int) -> Optional[JointTrajectory]:
        """规划国际象棋移动轨迹"""

        try:
            trajectory = JointTrajectory()
            trajectory.header.stamp = self.get_clock().now().to_msg()
            trajectory.joint_names = self.joint_names

            waypoints = []
            time_stamps = []

            # 1. 移动到起始位置上方
            pick_above = Point()
            pick_above.x = from_pos.x
            pick_above.y = from_pos.y
            pick_above.z = from_pos.z + self.safe_height

            waypoints.append(self._inverse_kinematics(pick_above, gripper_open=True))
            time_stamps.append(2.0)

            # 2. 下降到棋子高度
            pick_pos = Point()
            pick_pos.x = from_pos.x
            pick_pos.y = from_pos.y
            pick_pos.z = from_pos.z + self.pick_height

            waypoints.append(self._inverse_kinematics(pick_pos, gripper_open=True))
            time_stamps.append(3.0)

            # 3. 抓取棋子
            waypoints.append(self._inverse_kinematics(pick_pos, gripper_open=False))
            time_stamps.append(4.0)

            # 4. 提起棋子
            waypoints.append(self._inverse_kinematics(pick_above, gripper_open=False))
            time_stamps.append(5.0)

            # 5. 移动到目标位置上方
            place_above = Point()
            place_above.x = to_pos.x
            place_above.y = to_pos.y
            place_above.z = to_pos.z + self.safe_height

            waypoints.append(self._inverse_kinematics(place_above, gripper_open=False))
            time_stamps.append(7.0)

            # 6. 下降到放置高度
            place_pos = Point()
            place_pos.x = to_pos.x
            place_pos.y = to_pos.y
            place_pos.z = to_pos.z + self.place_height

            waypoints.append(self._inverse_kinematics(place_pos, gripper_open=False))
            time_stamps.append(8.0)

            # 7. 释放棋子
            waypoints.append(self._inverse_kinematics(place_pos, gripper_open=True))
            time_stamps.append(9.0)

            # 8. 提起并回到安全位置
            waypoints.append(self._inverse_kinematics(place_above, gripper_open=True))
            time_stamps.append(10.0)

            # 构建轨迹点
            for i, (positions, timestamp) in enumerate(zip(waypoints, time_stamps)):
                point = JointTrajectoryPoint()
                point.positions = positions
                point.velocities = [0.0] * 6
                point.accelerations = [0.0] * 6
                point.time_from_start.sec = int(timestamp)
                point.time_from_start.nanosec = int((timestamp - int(timestamp)) * 1e9)
                trajectory.points.append(point)

            return trajectory

        except Exception as e:
            self.get_logger().error(f'轨迹规划失败: {e}')
            return None

    def _inverse_kinematics(self, target_pos: Point, gripper_open: bool = True) -> List[float]:
        """简化的逆运动学计算"""

        # 这是一个简化的逆运动学实现
        # 实际应用中需要完整的几何计算

        x, y, z = target_pos.x, target_pos.y, target_pos.z

        # 基座旋转角度
        base_angle = np.arctan2(y, x)

        # 简化的臂长计算
        r = np.sqrt(x*x + y*y)

        # 肩部和肘部角度（简化计算）
        shoulder_angle = np.arctan2(z - 0.1, r - 0.1)  # 考虑基座高度
        elbow_angle = -np.pi/4  # 固定肘部角度

        # 腕部角度
        wrist1_angle = 0.0
        wrist2_angle = -(shoulder_angle + elbow_angle)  # 保持末端水平

        # 夹爪状态
        gripper_angle = 0.01 if gripper_open else 0.035

        positions = [
            base_angle,
            shoulder_angle,
            elbow_angle,
            wrist1_angle,
            wrist2_angle,
            gripper_angle
        ]

        # 检查关节限制
        for i, (pos, joint_name) in enumerate(zip(positions, self.joint_names)):
            min_pos, max_pos = self.joint_limits[joint_name]
            positions[i] = max(min_pos, min(max_pos, pos))

        return positions

    def _execute_trajectory(self, trajectory: JointTrajectory) -> bool:
        """执行关节轨迹"""

        with self.movement_lock:
            if self.is_moving:
                return False

            self.is_moving = True

        try:
            # 在仿真模式下，直接设置目标位置
            if self.simulation_mode:
                for point in trajectory.points:
                    if point.positions:
                        self.target_joint_positions = point.positions.copy()

                        # 等待到达目标位置
                        timeout = 5.0  # 5秒超时
                        start_time = time.time()

                        while (time.time() - start_time < timeout and
                               not self._is_at_target_position()):
                            time.sleep(0.1)

            return True

        except Exception as e:
            self.get_logger().error(f'轨迹执行失败: {e}')
            return False

        finally:
            with self.movement_lock:
                self.is_moving = False

    def _is_at_target_position(self, tolerance: float = 0.05) -> bool:
        """检查是否到达目标位置"""
        for current, target in zip(self.current_joint_positions, self.target_joint_positions):
            if abs(current - target) > tolerance:
                return False
        return True

    def _estimate_execution_time(self, trajectory: JointTrajectory) -> float:
        """估算轨迹执行时间"""
        if not trajectory.points:
            return 0.0

        last_point = trajectory.points[-1]
        return last_point.time_from_start.sec + last_point.time_from_start.nanosec / 1e9

    def _update_motion(self):
        """更新关节运动"""
        with self.movement_lock:
            if not self.is_moving:
                return

            # 简单的关节插值运动
            for i in range(len(self.current_joint_positions)):
                error = self.target_joint_positions[i] - self.current_joint_positions[i]
                if abs(error) > 0.001:  # 最小误差阈值
                    # 使用比例控制
                    self.current_joint_positions[i] += error * 0.1

    def _publish_joint_states(self):
        """发布关节状态"""
        joint_state = JointState()
        joint_state.header.stamp = self.get_clock().now().to_msg()
        joint_state.name = self.joint_names
        joint_state.position = self.current_joint_positions.copy()
        joint_state.velocity = self.current_joint_velocities.copy()

        self.joint_state_publisher.publish(joint_state)

        # 同时发布控制命令（用于Gazebo）
        command_msg = Float64MultiArray()
        command_msg.data = self.current_joint_positions.copy()
        self.joint_command_publisher.publish(command_msg)

    # 公共接口方法
    def move_to_home_position(self):
        """移动到初始位置"""
        home_positions = [0.0, 0.0, 0.0, 0.0, 0.0, 0.01]  # 夹爪微开

        with self.movement_lock:
            self.target_joint_positions = home_positions
            self.is_moving = True

    def emergency_stop(self):
        """紧急停止"""
        with self.movement_lock:
            self.target_joint_positions = self.current_joint_positions.copy()
            self.is_moving = False

        self.get_logger().warn('机械臂紧急停止')

    def get_current_pose(self) -> Dict:
        """获取当前末端位姿"""
        # 简化的正运动学
        return {
            'position': {
                'x': 0.3,  # 简化值
                'y': 0.0,
                'z': 0.2
            },
            'joints': self.current_joint_positions.copy()
        }


def main(args=None):
    rclpy.init(args=args)

    try:
        arm_sim = ChessArmSim()
        rclpy.spin(arm_sim)
    except KeyboardInterrupt:
        pass
    finally:
        if 'arm_sim' in locals():
            arm_sim.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()