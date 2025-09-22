#!/usr/bin/env python3
"""
Gazebo Web查看器服务器 - 修复版
通过Web界面实时查看Gazebo仿真环境
"""

import os
import sys
import time
import threading
import subprocess
import signal
from io import BytesIO
import traceback

import cv2
import numpy as np
from flask import Flask, render_template, Response, jsonify, request
from flask_socketio import SocketIO, emit
from PIL import Image, ImageDraw, ImageFont

# 尝试导入ROS2，如果失败则使用模拟模式
ROS_AVAILABLE = True
try:
    import rclpy
    from rclpy.node import Node
    from sensor_msgs.msg import Image as ROSImage
    from cv_bridge import CvBridge
    print("✅ ROS2模块导入成功")
except ImportError as e:
    print(f"⚠️ ROS2模块导入失败: {e}")
    print("🔄 将使用模拟模式运行")
    ROS_AVAILABLE = False

class GazeboWebViewer:
    def __init__(self):
        self.current_frame = None
        self.frame_lock = threading.Lock()
        self.fps_counter = 0
        self.fps_start_time = time.time()
        self.current_fps = 0
        self.ros_node = None
        self.ros_connected = False

        if ROS_AVAILABLE:
            self.init_ros()
        else:
            self.init_simulation_mode()

    def init_ros(self):
        """初始化ROS2节点"""
        try:
            if not rclpy.ok():
                rclpy.init()

            from rclpy.node import Node
            from sensor_msgs.msg import Image as ROSImage
            from cv_bridge import CvBridge

            class WebViewerNode(Node):
                def __init__(self, parent):
                    super().__init__('gazebo_web_viewer')
                    self.parent = parent
                    self.bridge = CvBridge()

                    # 订阅仿真相机话题
                    self.rgb_subscription = self.create_subscription(
                        ROSImage,
                        '/camera/rgb/image_raw',
                        self.rgb_callback,
                        10
                    )

                    self.get_logger().info('Gazebo Web查看器节点已启动')

                def rgb_callback(self, msg):
                    """接收ROS图像消息"""
                    try:
                        cv_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
                        cv_image = self.parent.add_info_overlay(cv_image)

                        with self.parent.frame_lock:
                            self.parent.current_frame = cv_image

                        self.parent.update_fps()

                    except Exception as e:
                        self.get_logger().error(f'图像处理错误: {e}')

            self.ros_node = WebViewerNode(self)
            self.ros_connected = True
            print("✅ ROS2节点初始化成功")

        except Exception as e:
            print(f"❌ ROS2初始化失败: {e}")
            self.init_simulation_mode()

    def init_simulation_mode(self):
        """初始化仿真模式"""
        print("🎥 启动仿真模式...")
        self.ros_connected = False

        def generate_frames():
            while True:
                frame = self.generate_test_frame()
                with self.frame_lock:
                    self.current_frame = frame
                self.update_fps()
                time.sleep(1/30)  # 30 FPS

        frame_thread = threading.Thread(target=generate_frames, daemon=True)
        frame_thread.start()

    def generate_test_frame(self):
        """生成测试图像"""
        # 创建640x480的测试图像
        image = np.zeros((480, 640, 3), dtype=np.uint8)

        # 添加渐变背景
        for y in range(480):
            for x in range(640):
                image[y, x] = [x//3, y//2, (x+y)//4]

        # 添加移动的图案
        center_x = int(320 + 200 * np.sin(time.time()))
        center_y = int(240 + 100 * np.cos(time.time()))
        cv2.circle(image, (center_x, center_y), 30, (0, 255, 0), -1)

        # 添加网格
        for i in range(0, 640, 50):
            cv2.line(image, (i, 0), (i, 480), (64, 64, 64), 1)
        for i in range(0, 480, 50):
            cv2.line(image, (0, i), (640, i), (64, 64, 64), 1)

        return self.add_info_overlay(image)

    def add_info_overlay(self, image):
        """在图像上添加信息覆盖层"""
        try:
            h, w = image.shape[:2]

            # 添加时间戳
            timestamp = time.strftime("%Y-%m-%d %H:%M:%S")
            cv2.putText(image, f'Time: {timestamp}', (10, 30),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)

            # 添加FPS信息
            cv2.putText(image, f'FPS: {self.current_fps:.1f}', (10, 60),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)

            # 添加分辨率信息
            cv2.putText(image, f'Resolution: {w}x{h}', (10, 90),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)

            # 添加模式信息
            mode = "ROS2 Mode" if self.ros_connected else "Simulation Mode"
            cv2.putText(image, f'Mode: {mode}', (10, 120),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 0), 2)

            return image

        except Exception as e:
            print(f'覆盖层添加错误: {e}')
            return image

    def update_fps(self):
        """更新FPS统计"""
        self.fps_counter += 1
        current_time = time.time()
        if current_time - self.fps_start_time >= 1.0:
            self.current_fps = self.fps_counter / (current_time - self.fps_start_time)
            self.fps_counter = 0
            self.fps_start_time = current_time

    def get_current_frame(self):
        """获取当前帧"""
        with self.frame_lock:
            if self.current_frame is not None:
                return self.current_frame.copy()
            else:
                # 创建默认图像
                image = np.zeros((480, 640, 3), dtype=np.uint8)
                cv2.putText(image, 'Waiting for video...', (200, 240),
                           cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)
                return image

    def spin_ros(self):
        """ROS2节点循环"""
        if self.ros_node and ROS_AVAILABLE:
            try:
                rclpy.spin(self.ros_node)
            except Exception as e:
                print(f"ROS2循环错误: {e}")

# Flask应用设置
app = Flask(__name__)
app.config['SECRET_KEY'] = 'chess_robot_web_viewer_2024'
socketio = SocketIO(app, cors_allowed_origins="*", async_mode='threading')

# 全局变量
gazebo_viewer = None
gazebo_process = None

def create_viewer_thread():
    """创建查看器线程"""
    global gazebo_viewer

    def viewer_worker():
        gazebo_viewer = GazeboWebViewer()
        if gazebo_viewer.ros_node and ROS_AVAILABLE:
            gazebo_viewer.spin_ros()

    viewer_thread = threading.Thread(target=viewer_worker, daemon=True)
    viewer_thread.start()
    time.sleep(1)  # 等待初始化
    return viewer_thread

def generate_video_stream():
    """生成视频流"""
    global gazebo_viewer

    while True:
        if gazebo_viewer is not None:
            frame = gazebo_viewer.get_current_frame()

            # 编码为JPEG
            ret, buffer = cv2.imencode('.jpg', frame,
                                     [cv2.IMWRITE_JPEG_QUALITY, 80])

            if ret:
                frame_bytes = buffer.tobytes()
                yield (b'--frame\r\n'
                       b'Content-Type: image/jpeg\r\n\r\n' + frame_bytes + b'\r\n')

        time.sleep(1/30)  # 30 FPS

@app.route('/')
def index():
    """主页"""
    return render_template('index.html')

@app.route('/video_feed')
def video_feed():
    """视频流路由"""
    return Response(generate_video_stream(),
                   mimetype='multipart/x-mixed-replace; boundary=frame')

@app.route('/api/status')
def api_status():
    """获取系统状态"""
    global gazebo_viewer, gazebo_process

    status = {
        'gazebo_running': gazebo_process is not None and gazebo_process.poll() is None,
        'ros_connected': gazebo_viewer.ros_connected if gazebo_viewer else False,
        'fps': gazebo_viewer.current_fps if gazebo_viewer else 0,
        'timestamp': time.strftime("%Y-%m-%d %H:%M:%S"),
        'mode': 'ROS2' if ROS_AVAILABLE and (gazebo_viewer.ros_connected if gazebo_viewer else False) else 'Simulation'
    }
    return jsonify(status)

@app.route('/api/control', methods=['POST'])
def api_control():
    """控制API"""
    try:
        data = request.json
        command = data.get('command', '')

        if command == 'start_gazebo':
            return start_gazebo_simulation()
        elif command == 'stop_gazebo':
            return stop_gazebo_simulation()
        elif command == 'reset_simulation':
            return reset_simulation()
        else:
            return jsonify({'success': False, 'message': f'未知命令: {command}'})

    except Exception as e:
        return jsonify({'success': False, 'message': str(e)})

def start_gazebo_simulation():
    """启动Gazebo仿真"""
    global gazebo_process

    try:
        if gazebo_process is None or gazebo_process.poll() is not None:
            # 启动Gazebo仿真（无头模式）
            gazebo_cmd = [
                'bash', '-c',
                'source install/setup.bash && ros2 launch chess_simulation gazebo_world.launch.py gui:=false'
            ]

            gazebo_process = subprocess.Popen(
                gazebo_cmd,
                cwd='/home/jetson/aichess',
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE
            )

            return jsonify({
                'success': True,
                'message': 'Gazebo仿真启动中...',
                'pid': gazebo_process.pid
            })
        else:
            return jsonify({
                'success': False,
                'message': 'Gazebo已在运行'
            })

    except Exception as e:
        return jsonify({
            'success': False,
            'message': f'启动Gazebo失败: {str(e)}'
        })

def stop_gazebo_simulation():
    """停止Gazebo仿真"""
    global gazebo_process

    try:
        if gazebo_process and gazebo_process.poll() is None:
            gazebo_process.terminate()
            gazebo_process.wait(timeout=10)
            gazebo_process = None

            return jsonify({
                'success': True,
                'message': 'Gazebo仿真已停止'
            })
        else:
            return jsonify({
                'success': False,
                'message': 'Gazebo未在运行'
            })

    except Exception as e:
        return jsonify({
            'success': False,
            'message': f'停止Gazebo失败: {str(e)}'
        })

def reset_simulation():
    """重置仿真"""
    try:
        # 发送重置命令到Gazebo
        reset_cmd = ['gz', 'service', '-s', '/world/chess_world/control',
                    '--reqtype', 'gz.msgs.WorldControl',
                    '--reptype', 'gz.msgs.Boolean',
                    '--timeout', '3000',
                    '--req', 'reset: {all: true}']

        result = subprocess.run(reset_cmd, capture_output=True, text=True, timeout=5)

        if result.returncode == 0:
            return jsonify({
                'success': True,
                'message': '仿真已重置'
            })
        else:
            return jsonify({
                'success': False,
                'message': f'重置失败: {result.stderr}'
            })

    except Exception as e:
        return jsonify({
            'success': False,
            'message': f'重置仿真失败: {str(e)}'
        })

@socketio.on('connect')
def handle_connect():
    """WebSocket连接处理"""
    print(f'客户端已连接: {request.sid}')
    emit('status', {'message': 'WebSocket连接成功'})

@socketio.on('disconnect')
def handle_disconnect():
    """WebSocket断开处理"""
    print(f'客户端已断开: {request.sid}')

def cleanup():
    """清理资源"""
    global gazebo_process

    print('正在清理资源...')

    if gazebo_process and gazebo_process.poll() is None:
        print('停止Gazebo进程...')
        gazebo_process.terminate()
        try:
            gazebo_process.wait(timeout=5)
        except subprocess.TimeoutExpired:
            gazebo_process.kill()

    if ROS_AVAILABLE and rclpy.ok():
        rclpy.shutdown()

def signal_handler(sig, frame):
    """信号处理器"""
    print('收到中断信号，正在退出...')
    cleanup()
    sys.exit(0)

def main():
    """主函数"""
    global gazebo_viewer

    # 设置信号处理
    signal.signal(signal.SIGINT, signal_handler)
    signal.signal(signal.SIGTERM, signal_handler)

    print('🚀 启动Gazebo Web查看器...')

    try:
        # 启动查看器线程
        gazebo_viewer = GazeboWebViewer()

        # 获取本机IP地址
        import socket
        hostname = socket.gethostname()
        local_ip = socket.gethostbyname(hostname)

        print(f'🌐 Web服务器启动在:')
        print(f'  本地访问: http://localhost:8080')
        print(f'  局域网访问: http://{local_ip}:8080')
        print(f'  或访问: http://192.168.8.88:8080')
        print()
        print('📱 功能说明:')
        print('  - 实时视频流显示')
        print('  - 仿真控制面板')
        print('  - 系统状态监控')
        print('  - 支持移动端访问')
        print()
        print('🛑 按 Ctrl+C 退出')

        # 启动Web服务器
        socketio.run(
            app,
            host='0.0.0.0',  # 监听所有接口
            port=8080,
            debug=False,
            use_reloader=False,
            allow_unsafe_werkzeug=True
        )

    except KeyboardInterrupt:
        print('\n👋 用户中断')
    except Exception as e:
        print(f'❌ 启动失败: {e}')
        traceback.print_exc()
    finally:
        cleanup()

if __name__ == '__main__':
    main()