#!/usr/bin/env python3
"""
基于MoveIt2的DOFBOT Pro机械臂控制器
使用专业的运动规划、碰撞检测和轨迹执行
"""

import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup

# MoveIt2 imports
import moveit_commander
from moveit_msgs.msg import CollisionObject, AttachedCollisionObject, PlanningScene
from moveit_msgs.action import MoveGroup
from moveit_msgs.srv import GetStateValidityRequest, GetStateValidityResponse

# Geometry and trajectory imports
from geometry_msgs.msg import Pose, Point, Quaternion, PoseStamped
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from sensor_msgs.msg import JointState
from std_msgs.msg import String, Header
from shape_msgs.msg import SolidPrimitive, Mesh

# Chess interfaces
from chess_interfaces.msg import ChessMove, BoardState
from chess_interfaces.srv import PlanMove, ExecuteMove

# Standard imports
import json
import time
import threading
import numpy as np
from typing import List, Dict, Optional, Tuple
from enum import Enum
import tf2_ros
import tf2_geometry_msgs
from scipy.spatial.transform import Rotation as R


class MoveItArmController(Node):
    """基于MoveIt2的机械臂控制器"""

    def __init__(self):
        super().__init__('moveit_arm_controller')

        # 参数声明
        self.declare_parameter('planning_group', 'arm')
        self.declare_parameter('gripper_group', 'gripper')
        self.declare_parameter('end_effector_link', 'gripper_center')
        self.declare_parameter('reference_frame', 'base_link')
        self.declare_parameter('planning_time', 10.0)
        self.declare_parameter('velocity_scaling', 0.3)
        self.declare_parameter('acceleration_scaling', 0.3)
        self.declare_parameter('max_cartesian_speed', 0.1)
        self.declare_parameter('jump_threshold', 0.0)
        self.declare_parameter('eef_step', 0.01)

        # 获取参数
        self.planning_group = self.get_parameter('planning_group').value
        self.gripper_group = self.get_parameter('gripper_group').value
        self.end_effector_link = self.get_parameter('end_effector_link').value
        self.reference_frame = self.get_parameter('reference_frame').value
        self.planning_time = self.get_parameter('planning_time').value
        self.velocity_scaling = self.get_parameter('velocity_scaling').value
        self.acceleration_scaling = self.get_parameter('acceleration_scaling').value
        self.max_cartesian_speed = self.get_parameter('max_cartesian_speed').value
        self.jump_threshold = self.get_parameter('jump_threshold').value
        self.eef_step = self.get_parameter('eef_step').value

        # 回调组
        self.callback_group = ReentrantCallbackGroup()

        # 状态管理
        self.current_state = 'idle'
        self.current_joint_states = None
        self.is_gripper_open = True
        self.state_lock = threading.Lock()

        # 象棋环境参数
        self.board_height = 0.02
        self.piece_height = 0.03
        self.square_size = 0.057
        self.board_origin = Point(x=0.4, y=-0.22, z=self.board_height)
        self.safe_height = 0.12
        self.capture_zone = Point(x=0.6, y=0.3, z=0.05)

        # 预定义位置（关节空间）
        self.named_positions = {
            'home': [0.0, -1.57, 0.0, 0.0, 1.57, 0.0],
            'ready': [0.0, -0.5, 0.5, 0.0, 1.0, 0.0],
            'observe': [0.0, -0.3, 0.8, 0.0, 0.5, 0.0]
        }

        # TF buffer and listener
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # 初始化MoveIt Commander
        self.init_moveit()

        # ROS2接口
        self.setup_ros_interfaces()

        # 碰撞环境
        self.setup_collision_environment()

        self.get_logger().info("MoveIt2机械臂控制器初始化完成")

    def init_moveit(self):
        """初始化MoveIt Commander"""
        try:
            # 初始化moveit_commander
            moveit_commander.roscpp_initialize([])

            # 创建RobotCommander对象
            self.robot = moveit_commander.RobotCommander()

            # 创建PlanningSceneInterface对象
            self.scene = moveit_commander.PlanningSceneInterface()

            # 创建MoveGroupCommander对象
            self.arm_group = moveit_commander.MoveGroupCommander(self.planning_group)
            self.gripper_group = moveit_commander.MoveGroupCommander(self.gripper_group)

            # 设置规划参数
            self.arm_group.set_planning_time(self.planning_time)
            self.arm_group.set_max_velocity_scaling_factor(self.velocity_scaling)
            self.arm_group.set_max_acceleration_scaling_factor(self.acceleration_scaling)
            self.arm_group.set_end_effector_link(self.end_effector_link)
            self.arm_group.set_pose_reference_frame(self.reference_frame)

            # 设置规划器
            self.arm_group.set_planner_id("RRTConnectkConfigDefault")

            # 获取机器人信息
            self.planning_frame = self.arm_group.get_planning_frame()
            self.eef_link = self.arm_group.get_end_effector_link()
            self.group_names = self.robot.get_group_names()

            self.get_logger().info(f"规划坐标系: {self.planning_frame}")
            self.get_logger().info(f"末端执行器: {self.eef_link}")
            self.get_logger().info(f"机器人组: {self.group_names}")

        except Exception as e:
            self.get_logger().error(f"MoveIt初始化失败: {e}")
            raise

    def setup_ros_interfaces(self):
        """设置ROS2接口"""
        # 订阅者
        self.move_request_sub = self.create_subscription(
            ChessMove, '/chess/move_request', self.move_request_callback, 10,
            callback_group=self.callback_group)

        self.joint_states_sub = self.create_subscription(
            JointState, '/joint_states', self.joint_states_callback, 10,
            callback_group=self.callback_group)

        self.board_state_sub = self.create_subscription(
            BoardState, '/chess/board_state', self.board_state_callback, 10,
            callback_group=self.callback_group)

        # 发布者
        self.planning_scene_pub = self.create_publisher(
            PlanningScene, '/planning_scene', 10)

        self.status_pub = self.create_publisher(
            String, '/chess/arm_status', 10)

        # 服务
        self.plan_move_service = self.create_service(
            PlanMove, '/chess/plan_move', self.plan_move_callback,
            callback_group=self.callback_group)

        self.execute_move_service = self.create_service(
            ExecuteMove, '/chess/execute_move', self.execute_move_callback,
            callback_group=self.callback_group)

        # 定时器
        self.status_timer = self.create_timer(1.0, self.publish_status)

    def setup_collision_environment(self):
        """设置碰撞环境"""
        try:
            # 清空现有碰撞对象
            self.scene.clear()

            # 等待场景更新
            time.sleep(1.0)

            # 添加棋盘
            self.add_chessboard_collision()

            # 添加桌子
            self.add_table_collision()

            # 添加工作空间边界
            self.add_workspace_boundaries()

            self.get_logger().info("碰撞环境设置完成")

        except Exception as e:
            self.get_logger().error(f"设置碰撞环境失败: {e}")

    def add_chessboard_collision(self):
        """添加棋盘碰撞对象"""
        # 棋盘平面
        board_pose = PoseStamped()
        board_pose.header.frame_id = self.reference_frame
        board_pose.pose.position.x = self.board_origin.x + 4 * self.square_size
        board_pose.pose.position.y = self.board_origin.y + 4 * self.square_size
        board_pose.pose.position.z = self.board_origin.z - 0.01
        board_pose.pose.orientation.w = 1.0

        self.scene.add_box("chessboard", board_pose, (0.46, 0.46, 0.02))

        # 棋盘边框
        frame_poses = []
        frame_sizes = []

        # 四个边框
        for i, (dx, dy) in enumerate([(0, 4), (8, 4), (4, 0), (4, 8)]):
            pose = PoseStamped()
            pose.header.frame_id = self.reference_frame
            pose.pose.position.x = self.board_origin.x + dx * self.square_size
            pose.pose.position.y = self.board_origin.y + dy * self.square_size
            pose.pose.position.z = self.board_origin.z + 0.02
            pose.pose.orientation.w = 1.0

            if i < 2:  # 上下边框
                self.scene.add_box(f"frame_{i}", pose, (0.5, 0.02, 0.04))
            else:  # 左右边框
                self.scene.add_box(f"frame_{i}", pose, (0.02, 0.5, 0.04))

    def add_table_collision(self):
        """添加桌子碰撞对象"""
        table_pose = PoseStamped()
        table_pose.header.frame_id = self.reference_frame
        table_pose.pose.position.x = 0.4
        table_pose.pose.position.y = 0.0
        table_pose.pose.position.z = -0.05
        table_pose.pose.orientation.w = 1.0

        self.scene.add_box("table", table_pose, (1.0, 1.0, 0.1))

    def add_workspace_boundaries(self):
        """添加工作空间边界"""
        # 后墙
        back_wall = PoseStamped()
        back_wall.header.frame_id = self.reference_frame
        back_wall.pose.position.x = -0.1
        back_wall.pose.position.y = 0.0
        back_wall.pose.position.z = 0.5
        back_wall.pose.orientation.w = 1.0

        self.scene.add_box("back_wall", back_wall, (0.02, 2.0, 1.0))

    def move_request_callback(self, msg):
        """处理移动请求"""
        with self.state_lock:
            if self.current_state != 'idle':
                self.get_logger().warning(f"机械臂忙碌，状态: {self.current_state}")
                return

            self.get_logger().info(f"收到移动请求: {msg.move_notation}")
            self.execute_chess_move_async(msg)

    def joint_states_callback(self, msg):
        """关节状态回调"""
        self.current_joint_states = msg

    def board_state_callback(self, msg):
        """棋盘状态回调"""
        # 根据棋盘状态更新碰撞环境中的棋子
        self.update_piece_collision_objects(msg)

    def execute_chess_move_async(self, move: ChessMove):
        """异步执行象棋移动"""
        move_thread = threading.Thread(
            target=self.execute_chess_move,
            args=(move,),
            daemon=True
        )
        move_thread.start()

    def execute_chess_move(self, move: ChessMove):
        """执行象棋移动"""
        try:
            self.current_state = 'moving'
            start_time = time.time()

            # 计算起始和目标位置
            from_pos = self.square_index_to_position(move.from_square)
            to_pos = self.square_index_to_position(move.to_square)

            self.get_logger().info(f"执行移动: {move.from_square} → {move.to_square}")

            # 如果是吃子，先移除目标棋子
            if move.is_capture:
                if not self.capture_piece_at_position(to_pos):
                    raise Exception("吃子失败")

            # 执行主要移动
            success = self.pick_and_place_with_planning(from_pos, to_pos)

            if success:
                self.current_state = 'idle'
                elapsed_time = time.time() - start_time
                self.get_logger().info(f"移动完成，耗时: {elapsed_time:.2f}s")
            else:
                raise Exception("移动执行失败")

        except Exception as e:
            self.current_state = 'error'
            self.get_logger().error(f"移动执行异常: {e}")

    def pick_and_place_with_planning(self, from_pos: Point, to_pos: Point) -> bool:
        """使用MoveIt2规划的抓取和放置"""
        try:
            # 1. 移动到抓取准备位置
            pickup_approach = self.create_pose_above_position(from_pos, self.safe_height)
            if not self.plan_and_execute_cartesian_path([pickup_approach]):
                return False

            # 2. 下降到棋子位置
            pickup_pose = self.create_pose_above_position(from_pos, self.piece_height / 2)
            if not self.plan_and_execute_cartesian_path([pickup_pose]):
                return False

            # 3. 夹取棋子
            if not self.control_gripper(close=True):
                return False

            # 4. 提升到安全高度（带棋子）
            pickup_retreat = self.create_pose_above_position(from_pos, self.safe_height)
            if not self.plan_and_execute_cartesian_path([pickup_retreat]):
                return False

            # 5. 移动到放置位置上方
            place_approach = self.create_pose_above_position(to_pos, self.safe_height)
            if not self.plan_and_execute_cartesian_path([place_approach]):
                return False

            # 6. 下降到放置位置
            place_pose = self.create_pose_above_position(to_pos, self.piece_height / 2)
            if not self.plan_and_execute_cartesian_path([place_pose]):
                return False

            # 7. 释放棋子
            if not self.control_gripper(close=False):
                return False

            # 8. 提升到安全高度
            place_retreat = self.create_pose_above_position(to_pos, self.safe_height)
            if not self.plan_and_execute_cartesian_path([place_retreat]):
                return False

            return True

        except Exception as e:
            self.get_logger().error(f"抓取放置失败: {e}")
            return False

    def capture_piece_at_position(self, pos: Point) -> bool:
        """吃子操作"""
        try:
            self.get_logger().info(f"执行吃子: ({pos.x:.3f}, {pos.y:.3f})")

            # 移动到目标棋子上方
            approach_pose = self.create_pose_above_position(pos, self.safe_height)
            if not self.plan_and_execute_cartesian_path([approach_pose]):
                return False

            # 下降抓取
            capture_pose = self.create_pose_above_position(pos, self.piece_height / 2)
            if not self.plan_and_execute_cartesian_path([capture_pose]):
                return False

            # 夹取
            if not self.control_gripper(close=True):
                return False

            # 提升
            retreat_pose = self.create_pose_above_position(pos, self.safe_height)
            if not self.plan_and_execute_cartesian_path([retreat_pose]):
                return False

            # 移动到吃子区域
            capture_zone_approach = self.create_pose_above_position(
                self.capture_zone, self.safe_height)
            if not self.plan_and_execute_cartesian_path([capture_zone_approach]):
                return False

            # 放下
            capture_zone_place = self.create_pose_above_position(
                self.capture_zone, 0.01)
            if not self.plan_and_execute_cartesian_path([capture_zone_place]):
                return False

            # 松开
            return self.control_gripper(close=False)

        except Exception as e:
            self.get_logger().error(f"吃子操作失败: {e}")
            return False

    def plan_and_execute_cartesian_path(self, waypoints: List[Pose]) -> bool:
        """规划并执行笛卡尔路径"""
        try:
            # 使用MoveIt2进行笛卡尔路径规划
            (plan, fraction) = self.arm_group.compute_cartesian_path(
                waypoints,
                self.eef_step,
                self.jump_threshold
            )

            if fraction < 0.9:
                self.get_logger().warning(f"笛卡尔路径规划不完整: {fraction * 100:.1f}%")
                # 尝试关节空间规划作为后备
                return self.plan_and_execute_joint_space(waypoints[-1])

            # 执行路径
            success = self.arm_group.execute(plan, wait=True)
            self.arm_group.stop()
            self.arm_group.clear_pose_targets()

            return success

        except Exception as e:
            self.get_logger().error(f"笛卡尔路径执行失败: {e}")
            return False

    def plan_and_execute_joint_space(self, target_pose: Pose) -> bool:
        """关节空间规划并执行"""
        try:
            # 设置目标位姿
            self.arm_group.set_pose_target(target_pose)

            # 规划
            plan = self.arm_group.plan()[1]

            # 执行
            success = self.arm_group.execute(plan, wait=True)
            self.arm_group.stop()
            self.arm_group.clear_pose_targets()

            return success

        except Exception as e:
            self.get_logger().error(f"关节空间规划失败: {e}")
            return False

    def control_gripper(self, close: bool) -> bool:
        """控制夹爪"""
        try:
            if close:
                target_state = "gripper_closed"
                self.get_logger().info("关闭夹爪")
            else:
                target_state = "gripper_open"
                self.get_logger().info("打开夹爪")

            # 使用MoveIt控制夹爪
            self.gripper_group.set_named_target(target_state)
            success = self.gripper_group.go(wait=True)
            self.gripper_group.stop()

            self.is_gripper_open = not close
            return success

        except Exception as e:
            self.get_logger().error(f"夹爪控制失败: {e}")
            return False

    def move_to_named_position(self, position_name: str) -> bool:
        """移动到预定义位置"""
        try:
            self.get_logger().info(f"移动到位置: {position_name}")
            self.arm_group.set_named_target(position_name)
            success = self.arm_group.go(wait=True)
            self.arm_group.stop()
            return success

        except Exception as e:
            self.get_logger().error(f"移动到预定义位置失败: {e}")
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

        # 末端执行器垂直向下的姿态
        # 使用四元数表示180度绕X轴旋转
        pose.orientation.x = 1.0
        pose.orientation.y = 0.0
        pose.orientation.z = 0.0
        pose.orientation.w = 0.0

        return pose

    def update_piece_collision_objects(self, board_state: BoardState):
        """根据棋盘状态更新棋子碰撞对象"""
        try:
            # 清除现有棋子碰撞对象
            piece_objects = [f"piece_{i}" for i in range(64)]
            for obj in piece_objects:
                self.scene.remove_world_object(obj)

            # 添加当前棋子位置
            for i, piece_type in enumerate(board_state.board_squares):
                if piece_type != 0:  # 0表示空格
                    pos = board_state.square_positions_3d[i]

                    piece_pose = PoseStamped()
                    piece_pose.header.frame_id = self.reference_frame
                    piece_pose.pose.position = pos
                    piece_pose.pose.orientation.w = 1.0

                    # 棋子碰撞对象（圆柱体）
                    self.scene.add_cylinder(f"piece_{i}", piece_pose,
                                          height=self.piece_height,
                                          radius=0.015)

        except Exception as e:
            self.get_logger().error(f"更新棋子碰撞对象失败: {e}")

    def plan_move_callback(self, request, response):
        """运动规划服务回调"""
        try:
            from_pos = self.square_index_to_position(request.from_square)
            to_pos = self.square_index_to_position(request.to_square)

            # 创建测试轨迹
            waypoints = [
                self.create_pose_above_position(from_pos, self.safe_height),
                self.create_pose_above_position(from_pos, self.piece_height / 2),
                self.create_pose_above_position(to_pos, self.safe_height),
                self.create_pose_above_position(to_pos, self.piece_height / 2)
            ]

            # 验证路径可行性
            (plan, fraction) = self.arm_group.compute_cartesian_path(
                waypoints, self.eef_step, self.jump_threshold)

            if fraction > 0.8:
                response.success = True
                response.message = f"规划成功，完整度: {fraction * 100:.1f}%"
                response.estimated_time = len(plan.joint_trajectory.points) * 0.1
            else:
                response.success = False
                response.message = f"规划失败，完整度不足: {fraction * 100:.1f}%"

        except Exception as e:
            response.success = False
            response.message = f"规划失败: {e}"

        return response

    def execute_move_callback(self, request, response):
        """执行移动服务回调"""
        try:
            # 创建ChessMove消息
            move = ChessMove()
            move.from_square = request.from_square
            move.to_square = request.to_square
            move.is_capture = request.is_capture

            # 执行移动
            success = self.execute_chess_move(move)

            response.success = success
            response.message = "执行成功" if success else "执行失败"

        except Exception as e:
            response.success = False
            response.message = f"执行失败: {e}"

        return response

    def publish_status(self):
        """发布状态信息"""
        try:
            current_pose = self.arm_group.get_current_pose().pose
            current_joints = self.arm_group.get_current_joint_values()

            status = {
                'node_status': 'running',
                'arm_state': self.current_state,
                'gripper_open': self.is_gripper_open,
                'current_pose': {
                    'position': {
                        'x': current_pose.position.x,
                        'y': current_pose.position.y,
                        'z': current_pose.position.z
                    },
                    'orientation': {
                        'x': current_pose.orientation.x,
                        'y': current_pose.orientation.y,
                        'z': current_pose.orientation.z,
                        'w': current_pose.orientation.w
                    }
                },
                'joint_positions': current_joints,
                'planning_frame': self.planning_frame,
                'end_effector_link': self.eef_link,
                'timestamp': time.time()
            }

            msg = String()
            msg.data = json.dumps(status)
            self.status_pub.publish(msg)

        except Exception as e:
            self.get_logger().error(f"发布状态失败: {e}")

    def emergency_stop(self):
        """紧急停止"""
        with self.state_lock:
            self.arm_group.stop()
            self.gripper_group.stop()
            self.current_state = 'emergency_stopped'
            self.get_logger().warning("执行紧急停止")

    def shutdown(self):
        """关闭节点"""
        try:
            self.arm_group.stop()
            self.gripper_group.stop()
            moveit_commander.roscpp_shutdown()
            self.get_logger().info("MoveIt2控制器已关闭")
        except Exception as e:
            self.get_logger().error(f"关闭失败: {e}")


def main(args=None):
    rclpy.init(args=args)

    node = MoveItArmController()

    # 使用多线程执行器
    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(node)

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()