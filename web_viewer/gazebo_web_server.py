#!/usr/bin/env python3
"""
Gazebo Web查看器服务器
通过Web界面实时查看Gazebo仿真环境
"""

import os
import sys
import time
import threading
import subprocess
import signal
from io import BytesIO
import base64

import cv2
import numpy as np
from flask import Flask, render_template, Response, jsonify, request
from flask_socketio import SocketIO, emit
from PIL import Image, ImageDraw, ImageFont
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image as ROSImage
from cv_bridge import CvBridge

class GazeboWebViewer(Node):
    def __init__(self):
        super().__init__('gazebo_web_viewer')

        # ROS2设置
        self.bridge = CvBridge()
        self.current_frame = None
        self.frame_lock = threading.Lock()

        # 订阅仿真相机话题
        self.rgb_subscription = self.create_subscription(
            ROSImage,
            '/camera/rgb/image_raw',
            self.rgb_callback,
            10
        )

        # 如果没有ROS图像，使用屏幕截图
        self.gazebo_process = None
        self.screenshot_enabled = False

        # 性能统计
        self.fps_counter = 0
        self.fps_start_time = time.time()
        self.current_fps = 0

        self.get_logger().info('Gazebo Web查看器节点已启动')

    def rgb_callback(self, msg):
        """接收ROS图像消息"""
        try:
            # 转换ROS图像为OpenCV格式
            cv_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')

            # 添加信息覆盖层
            cv_image = self.add_info_overlay(cv_image)

            with self.frame_lock:
                self.current_frame = cv_image

            # 更新FPS统计
            self.fps_counter += 1
            current_time = time.time()
            if current_time - self.fps_start_time >= 1.0:
                self.current_fps = self.fps_counter / (current_time - self.fps_start_time)
                self.fps_counter = 0
                self.fps_start_time = current_time

        except Exception as e:
            self.get_logger().error(f'图像处理错误: {e}')

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

            return image

        except Exception as e:
            self.get_logger().error(f'覆盖层添加错误: {e}')
            return image

    def capture_gazebo_screenshot(self):
        """通过屏幕截图方式获取Gazebo图像"""
        try:
            # 这里可以使用xvfb-run或者其他方式截图
            # 作为备选方案，如果ROS话题没有图像数据
            cmd = ['gnome-screenshot', '-w', '-f', '/tmp/gazebo_screenshot.png']
            subprocess.run(cmd, timeout=1)

            image = cv2.imread('/tmp/gazebo_screenshot.png')
            if image is not None:
                with self.frame_lock:
                    self.current_frame = image

        except Exception as e:
            self.get_logger().error(f'截图错误: {e}')

    def get_current_frame(self):
        """获取当前帧"""
        with self.frame_lock:
            if self.current_frame is not None:
                return self.current_frame.copy()
            else:
                # 创建默认图像
                image = np.zeros((480, 640, 3), dtype=np.uint8)
                cv2.putText(image, 'Waiting for Gazebo...', (200, 240),
                           cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)
                return image

# Flask应用设置
app = Flask(__name__)
app.config['SECRET_KEY'] = 'chess_robot_web_viewer_2024'
socketio = SocketIO(app, cors_allowed_origins="*", async_mode='threading')

# 全局变量
gazebo_viewer = None
gazebo_process = None

def create_ros_thread():
    """创建ROS2线程"""
    def ros_spin():
        global gazebo_viewer
        rclpy.init()
        gazebo_viewer = GazeboWebViewer()
        try:
            rclpy.spin(gazebo_viewer)
        except KeyboardInterrupt:
            pass
        finally:
            if gazebo_viewer:
                gazebo_viewer.destroy_node()
            rclpy.shutdown()

    ros_thread = threading.Thread(target=ros_spin, daemon=True)
    ros_thread.start()
    return ros_thread

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
    status = {
        'gazebo_running': gazebo_process is not None and gazebo_process.poll() is None,
        'ros_connected': gazebo_viewer is not None,
        'fps': gazebo_viewer.current_fps if gazebo_viewer else 0,
        'timestamp': time.strftime("%Y-%m-%d %H:%M:%S")
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
            # 启动Gazebo仿真
            gazebo_cmd = [
                'ros2', 'launch', 'chess_simulation', 'gazebo_world.launch.py',
                'gui:=true'
            ]

            gazebo_process = subprocess.Popen(
                gazebo_cmd,
                cwd='/home/jetson/aichess',
                env=dict(os.environ, ROS_DOMAIN_ID='0'),
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

def signal_handler(sig, frame):
    """信号处理器"""
    print('收到中断信号，正在退出...')
    cleanup()
    sys.exit(0)

def main():
    """主函数"""
    # 设置信号处理
    signal.signal(signal.SIGINT, signal_handler)
    signal.signal(signal.SIGTERM, signal_handler)

    print('启动Gazebo Web查看器...')

    try:
        # 启动ROS2线程
        ros_thread = create_ros_thread()

        # 等待ROS2节点启动
        time.sleep(2)

        # 获取本机IP地址
        import socket
        hostname = socket.gethostname()
        local_ip = socket.gethostbyname(hostname)

        print(f'Web服务器启动在:')
        print(f'  本地访问: http://localhost:8080')
        print(f'  局域网访问: http://{local_ip}:8080')
        print(f'  或访问: http://192.168.8.88:8080')
        print()
        print('按 Ctrl+C 退出')

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
        print('用户中断')
    except Exception as e:
        print(f'启动失败: {e}')
    finally:
        cleanup()

if __name__ == '__main__':
    main()