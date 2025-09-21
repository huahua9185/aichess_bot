#!/usr/bin/env python3
"""
DOFBOT Pro 6DOF机械臂控制器节点
负责机械臂的运动控制、轨迹规划和执行
支持仿真模式和真实硬件模式
"""

import rclpy
from rclpy.node import Node
from chess_interfaces.msg import ChessMove
from chess_interfaces.srv import PlanMove, ExecuteMove
from geometry_msgs.msg import Pose, Point, Quaternion
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from std_msgs.msg import String, Header
import json
import time
import threading
import numpy as np
from typing import List, Dict, Optional, Tuple
from enum import Enum


class ArmMode(Enum):
    """机械臂模式"""
    SIMULATION = "simulation"
    HARDWARE = "hardware"


class ArmState(Enum):
    """机械臂状态"""
    IDLE = "idle"
    MOVING = "moving"
    HOLDING = "holding"
    ERROR = "error"


class ArmController(Node):
    """DOFBOT Pro机械臂控制器"""

    def __init__(self):
        super().__init__('arm_controller')

        # 参数声明
        self.declare_parameter('simulation_mode', True)
        self.declare_parameter('robot_description', 'dofbot_pro')
        self.declare_parameter('planning_group', 'arm')
        self.declare_parameter('end_effector_link', 'gripper_center')
        self.declare_parameter('reference_frame', 'base_link')
        self.declare_parameter('max_velocity_scaling', 0.3)
        self.declare_parameter('max_acceleration_scaling', 0.3)
        self.declare_parameter('planning_timeout', 10.0)
        self.declare_parameter('execution_timeout', 30.0)

        # 获取参数
        self.simulation_mode = self.get_parameter('simulation_mode').value
        self.robot_description = self.get_parameter('robot_description').value
        self.planning_group = self.get_parameter('planning_group').value
        self.end_effector_link = self.get_parameter('end_effector_link').value
        self.reference_frame = self.get_parameter('reference_frame').value
        self.max_velocity_scaling = self.get_parameter('max_velocity_scaling').value
        self.max_acceleration_scaling = self.get_parameter('max_acceleration_scaling').value
        self.planning_timeout = self.get_parameter('planning_timeout').value
        self.execution_timeout = self.get_parameter('execution_timeout').value

        # 机械臂状态
        self.arm_mode = ArmMode.SIMULATION if self.simulation_mode else ArmMode.HARDWARE
        self.current_state = ArmState.IDLE
        self.current_joint_states = None
        self.current_pose = None
        self.is_gripper_open = True

        # 象棋相关参数
        self.board_height = 0.02  # 棋盘高度 (m)
        self.piece_height = 0.03  # 棋子高度 (m)
        self.square_size = 0.055  # 方格大小 (m)
        self.board_origin = Point(x=0.4, y=-0.22, z=self.board_height)  # 棋盘原点坐标

        # 预定义位置
        self.home_position = [0.0, -0.5, 0.0, 0.0, 0.0, 0.0]  # 关节角度
        self.capture_zone = Point(x=0.6, y=0.3, z=0.1)  # 吃子区域
        self.safe_height = 0.1  # 安全高度

        # 运动轨迹
        self.current_trajectory = None
        self.trajectory_start_time = None

        # 线程锁
        self.state_lock = threading.Lock()

        # 订阅者
        self.move_request_sub = self.create_subscription(
            ChessMove, '/chess/move_request', self.move_request_callback, 10)
        self.joint_states_sub = self.create_subscription(
            JointState, '/joint_states', self.joint_states_callback, 10)

        # 发布者
        self.joint_trajectory_pub = self.create_publisher(
            JointTrajectory, '/arm_controller/joint_trajectory', 10)
        self.status_pub = self.create_publisher(
            String, '/chess/move_status', 10)
        self.arm_status_pub = self.create_publisher(
            String, '/chess/arm_status', 10)

        # 服务
        self.plan_move_service = self.create_service(
            PlanMove, '/chess/plan_move', self.plan_move_callback)
        self.execute_move_service = self.create_service(
            ExecuteMove, '/chess/execute_move', self.execute_move_callback)

        # 定时器
        self.status_timer = self.create_timer(1.0, self.publish_status)

        # 初始化机械臂
        self.initialize_arm()

        self.get_logger().info(f"机械臂控制器已启动 - 模式: {self.arm_mode.value}")

    def initialize_arm(self):
        """初始化机械臂"""
        try:
            if self.arm_mode == ArmMode.SIMULATION:
                # 仿真模式初始化
                self.get_logger().info("初始化仿真模式机械臂")
                self.move_to_home_position()
            else:
                # 硬件模式初始化
                self.get_logger().info("初始化硬件模式机械臂")
                # 这里添加硬件初始化代码
                pass

        except Exception as e:
            self.get_logger().error(f"机械臂初始化失败: {e}")
            self.current_state = ArmState.ERROR

    def move_request_callback(self, msg):
        """移动请求回调"""
        with self.state_lock:
            if self.current_state != ArmState.IDLE:
                self.get_logger().warning(f"机械臂忙碌中，当前状态: {self.current_state.value}")
                return

            self.get_logger().info(f"收到移动请求: {msg.move_notation}")
            self.execute_chess_move_async(msg)

    def joint_states_callback(self, msg):
        """关节状态回调"""
        self.current_joint_states = msg
        # 更新当前末端位姿
        self.update_current_pose()

    def execute_chess_move_async(self, move: ChessMove):
        """异步执行象棋移动"""
        move_thread = threading.Thread(
            target=self.execute_chess_move,
            args=(move,)
        )
        move_thread.daemon = True
        move_thread.start()

    def execute_chess_move(self, move: ChessMove):
        """执行象棋移动"""
        try:
            self.current_state = ArmState.MOVING
            start_time = time.time()

            # 计算起始和目标位置
            from_pos = self.square_index_to_position(move.from_square)
            to_pos = self.square_index_to_position(move.to_square)

            self.get_logger().info(f"执行移动: {move.from_square} -> {move.to_square}")

            # 如果是吃子，先移除目标位置的棋子
            if move.is_capture:
                self.capture_piece_at_position(to_pos)

            # 执行主要移动
            success = self.pick_and_place(from_pos, to_pos)

            if success:
                self.current_state = ArmState.IDLE
                self.publish_move_status('completed', f"移动完成，耗时: {time.time() - start_time:.2f}s")
                self.get_logger().info("移动执行成功")
            else:
                self.current_state = ArmState.ERROR
                self.publish_move_status('failed', "移动执行失败")
                self.get_logger().error("移动执行失败")

        except Exception as e:
            self.current_state = ArmState.ERROR
            error_msg = f"移动执行异常: {e}"
            self.publish_move_status('failed', error_msg)
            self.get_logger().error(error_msg)

    def pick_and_place(self, from_pos: Point, to_pos: Point) -> bool:
        """抓取和放置棋子"""
        try:
            # 1. 移动到起始位置上方
            pickup_pose = self.create_pose_above_position(from_pos, self.safe_height)
            if not self.move_to_pose(pickup_pose):
                return False

            # 2. 下降到棋子位置
            pickup_pose.position.z = from_pos.z + self.piece_height / 2
            if not self.move_to_pose(pickup_pose):
                return False

            # 3. 夹取棋子
            if not self.close_gripper():
                return False

            self.current_state = ArmState.HOLDING

            # 4. 提升到安全高度
            pickup_pose.position.z = from_pos.z + self.safe_height
            if not self.move_to_pose(pickup_pose):
                return False

            # 5. 移动到目标位置上方
            place_pose = self.create_pose_above_position(to_pos, self.safe_height)
            if not self.move_to_pose(place_pose):
                return False

            # 6. 下降到放置位置
            place_pose.position.z = to_pos.z + self.piece_height / 2
            if not self.move_to_pose(place_pose):
                return False

            # 7. 松开棋子
            if not self.open_gripper():
                return False

            self.current_state = ArmState.IDLE

            # 8. 提升到安全高度
            place_pose.position.z = to_pos.z + self.safe_height
            if not self.move_to_pose(place_pose):
                return False

            return True

        except Exception as e:
            self.get_logger().error(f"抓取和放置失败: {e}")
            return False

    def capture_piece_at_position(self, pos: Point):
        """在指定位置吃子"""
        try:
            self.get_logger().info(f"吃子操作: ({pos.x:.3f}, {pos.y:.3f})")

            # 移动到目标棋子上方
            capture_pose = self.create_pose_above_position(pos, self.safe_height)
            if not self.move_to_pose(capture_pose):
                return False

            # 下降抓取
            capture_pose.position.z = pos.z + self.piece_height / 2
            if not self.move_to_pose(capture_pose):
                return False

            # 夹取
            if not self.close_gripper():
                return False

            # 提升
            capture_pose.position.z = pos.z + self.safe_height
            if not self.move_to_pose(capture_pose):
                return False

            # 移动到吃子区域
            capture_zone_pose = self.create_pose_above_position(
                self.capture_zone, self.safe_height)
            if not self.move_to_pose(capture_zone_pose):
                return False

            # 放下
            capture_zone_pose.position.z = self.capture_zone.z
            if not self.move_to_pose(capture_zone_pose):
                return False

            # 松开
            return self.open_gripper()

        except Exception as e:
            self.get_logger().error(f"吃子操作失败: {e}")
            return False

    def square_index_to_position(self, square_index: int) -> Point:
        """将棋盘索引转换为3D位置"""
        row = square_index // 8
        col = square_index % 8

        pos = Point()
        pos.x = self.board_origin.x + col * self.square_size
        pos.y = self.board_origin.y + row * self.square_size
        pos.z = self.board_origin.z

        return pos

    def create_pose_above_position(self, pos: Point, height: float) -> Pose:
        """在指定位置上方创建位姿"""
        pose = Pose()
        pose.position.x = pos.x
        pose.position.y = pos.y
        pose.position.z = pos.z + height

        # 末端执行器朝下的方向
        pose.orientation.x = 1.0
        pose.orientation.y = 0.0
        pose.orientation.z = 0.0
        pose.orientation.w = 0.0

        return pose

    def move_to_pose(self, target_pose: Pose) -> bool:
        """移动到目标位姿"""
        try:
            # 逆运动学计算
            target_joints = self.inverse_kinematics(target_pose)
            if target_joints is None:
                self.get_logger().error("逆运动学求解失败")
                return False

            # 执行关节轨迹
            return self.execute_joint_trajectory(target_joints)

        except Exception as e:
            self.get_logger().error(f"移动到位姿失败: {e}")
            return False

    def inverse_kinematics(self, target_pose: Pose) -> Optional[List[float]]:
        """逆运动学求解"""
        # 简化的逆运动学实现
        # 实际项目中应该使用MoveIt或其他专业库
        try:
            # 这里实现DOFBOT Pro的逆运动学
            # 为简化起见，返回一个示例解
            x, y, z = target_pose.position.x, target_pose.position.y, target_pose.position.z

            # 简单的几何解法（需要根据实际机械臂参数调整）
            joint1 = np.arctan2(y, x)
            r = np.sqrt(x*x + y*y)
            joint2 = np.arctan2(z, r)
            joint3 = 0.0
            joint4 = 0.0
            joint5 = 0.0
            joint6 = 0.0

            return [joint1, joint2, joint3, joint4, joint5, joint6]

        except Exception as e:
            self.get_logger().error(f"逆运动学计算失败: {e}")
            return None

    def execute_joint_trajectory(self, target_joints: List[float]) -> bool:
        """执行关节轨迹"""
        try:
            if not self.current_joint_states:
                self.get_logger().error("无当前关节状态")
                return False

            # 创建轨迹消息
            trajectory = JointTrajectory()
            trajectory.header = Header()
            trajectory.header.stamp = self.get_clock().now().to_msg()
            trajectory.joint_names = [
                'joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6'
            ]

            # 起始点
            start_point = JointTrajectoryPoint()
            start_point.positions = list(self.current_joint_states.position)
            start_point.time_from_start.sec = 0

            # 目标点
            end_point = JointTrajectoryPoint()
            end_point.positions = target_joints
            end_point.time_from_start.sec = 3  # 3秒内完成运动

            trajectory.points = [start_point, end_point]

            # 发布轨迹
            self.joint_trajectory_pub.publish(trajectory)
            self.get_logger().info(f"发布关节轨迹: {target_joints}")

            # 等待执行完成
            time.sleep(3.5)
            return True

        except Exception as e:
            self.get_logger().error(f"执行关节轨迹失败: {e}")
            return False

    def close_gripper(self) -> bool:
        """关闭夹爪"""
        try:
            self.get_logger().info("关闭夹爪")
            # 实现夹爪控制逻辑
            self.is_gripper_open = False
            time.sleep(1.0)  # 模拟夹爪动作时间
            return True
        except Exception as e:
            self.get_logger().error(f"关闭夹爪失败: {e}")
            return False

    def open_gripper(self) -> bool:
        """打开夹爪"""
        try:
            self.get_logger().info("打开夹爪")
            # 实现夹爪控制逻辑
            self.is_gripper_open = True
            time.sleep(1.0)  # 模拟夹爪动作时间
            return True
        except Exception as e:
            self.get_logger().error(f"打开夹爪失败: {e}")
            return False

    def move_to_home_position(self):
        """移动到初始位置"""
        try:
            self.get_logger().info("移动到初始位置")
            return self.execute_joint_trajectory(self.home_position)
        except Exception as e:
            self.get_logger().error(f"移动到初始位置失败: {e}")
            return False

    def update_current_pose(self):
        """更新当前末端位姿"""
        if not self.current_joint_states:
            return

        try:
            # 正运动学计算
            self.current_pose = self.forward_kinematics(
                list(self.current_joint_states.position))
        except Exception as e:
            self.get_logger().error(f"更新当前位姿失败: {e}")

    def forward_kinematics(self, joint_positions: List[float]) -> Pose:
        """正运动学计算"""
        # 简化的正运动学实现
        pose = Pose()
        pose.position.x = 0.5
        pose.position.y = 0.0
        pose.position.z = 0.3
        pose.orientation.w = 1.0
        return pose

    def plan_move_callback(self, request, response):
        """规划移动服务回调"""
        try:
            # 实现移动规划逻辑
            response.success = True
            response.message = "规划成功"
            response.estimated_time = 10.0

        except Exception as e:
            response.success = False
            response.message = f"规划失败: {e}"

        return response

    def execute_move_callback(self, request, response):
        """执行移动服务回调"""
        try:
            # 实现移动执行逻辑
            response.success = True
            response.message = "执行成功"

        except Exception as e:
            response.success = False
            response.message = f"执行失败: {e}"

        return response

    def publish_move_status(self, status: str, message: str = ""):
        """发布移动状态"""
        status_data = {
            'status': status,
            'message': message,
            'timestamp': time.time()
        }

        msg = String()
        msg.data = json.dumps(status_data)
        self.status_pub.publish(msg)

    def publish_status(self):
        """发布机械臂状态"""
        status = {
            'node_status': 'running',
            'arm_mode': self.arm_mode.value,
            'arm_state': self.current_state.value,
            'gripper_open': self.is_gripper_open,
            'current_pose': self.pose_to_dict(self.current_pose) if self.current_pose else None,
            'joint_positions': list(self.current_joint_states.position) if self.current_joint_states else None,
            'timestamp': time.time()
        }

        msg = String()
        msg.data = json.dumps(status)
        self.arm_status_pub.publish(msg)

    def pose_to_dict(self, pose: Pose) -> Dict:
        """位姿转换为字典"""
        return {
            'position': {
                'x': pose.position.x,
                'y': pose.position.y,
                'z': pose.position.z
            },
            'orientation': {
                'x': pose.orientation.x,
                'y': pose.orientation.y,
                'z': pose.orientation.z,
                'w': pose.orientation.w
            }
        }

    def emergency_stop(self):
        """紧急停止"""
        with self.state_lock:
            self.current_state = ArmState.IDLE
            self.get_logger().warning("执行紧急停止")
            # 停止所有运动
            # 实现紧急停止逻辑


def main(args=None):
    rclpy.init(args=args)
    node = ArmController()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()