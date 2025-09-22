#!/usr/bin/env python3
"""
简化的Web服务器测试脚本
用于排查问题
"""

import os
import time
import cv2
import numpy as np
from flask import Flask, render_template, Response
from flask_socketio import SocketIO

# Flask应用设置
app = Flask(__name__)
app.config['SECRET_KEY'] = 'test_key'
socketio = SocketIO(app, cors_allowed_origins="*")

def generate_test_frame():
    """生成测试图像"""
    # 创建640x480的测试图像
    image = np.zeros((480, 640, 3), dtype=np.uint8)

    # 添加渐变背景
    for y in range(480):
        for x in range(640):
            image[y, x] = [x//3, y//2, (x+y)//4]

    # 添加时间戳
    timestamp = time.strftime("%H:%M:%S")
    cv2.putText(image, f'Test Stream - {timestamp}', (50, 50),
               cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)

    # 添加移动的圆圈
    center_x = int(320 + 200 * np.sin(time.time()))
    center_y = int(240 + 100 * np.cos(time.time()))
    cv2.circle(image, (center_x, center_y), 30, (0, 255, 0), -1)

    return image

def generate_video_stream():
    """生成视频流"""
    while True:
        frame = generate_test_frame()

        # 编码为JPEG
        ret, buffer = cv2.imencode('.jpg', frame, [cv2.IMWRITE_JPEG_QUALITY, 80])

        if ret:
            frame_bytes = buffer.tobytes()
            yield (b'--frame\r\n'
                   b'Content-Type: image/jpeg\r\n\r\n' + frame_bytes + b'\r\n')

        time.sleep(1/30)  # 30 FPS

@app.route('/')
def index():
    """主页"""
    return '''
    <!DOCTYPE html>
    <html>
    <head>
        <title>Gazebo Web Viewer Test</title>
        <style>
            body { font-family: Arial; background: #1a1a1a; color: white; text-align: center; }
            img { max-width: 100%; border: 2px solid #333; }
            .container { max-width: 800px; margin: 0 auto; padding: 20px; }
        </style>
    </head>
    <body>
        <div class="container">
            <h1>🎥 Gazebo Web查看器测试</h1>
            <p>测试视频流是否正常工作</p>
            <img src="/video_feed" alt="Test Video Stream">
            <p><strong>状态:</strong> <span id="status">正在加载...</span></p>
            <p><strong>访问地址:</strong> http://192.168.8.88:8080</p>
        </div>
        <script>
            setInterval(function() {
                document.getElementById('status').textContent = '运行正常 - ' + new Date().toLocaleTimeString();
            }, 1000);
        </script>
    </body>
    </html>
    '''

@app.route('/video_feed')
def video_feed():
    """视频流路由"""
    return Response(generate_video_stream(),
                   mimetype='multipart/x-mixed-replace; boundary=frame')

@app.route('/api/test')
def api_test():
    """测试API"""
    return {
        'status': 'OK',
        'message': 'Web服务器运行正常',
        'timestamp': time.strftime("%Y-%m-%d %H:%M:%S"),
        'test': True
    }

if __name__ == '__main__':
    print('🚀 启动简化Web测试服务器...')
    print('📱 访问地址:')
    print('  http://localhost:8080')
    print('  http://192.168.8.88:8080')
    print('  http://10.0.0.119:8080')
    print('🛑 按 Ctrl+C 停止')

    try:
        socketio.run(
            app,
            host='0.0.0.0',
            port=8080,
            debug=False,
            use_reloader=False
        )
    except KeyboardInterrupt:
        print('\n👋 服务器已停止')
    except Exception as e:
        print(f'❌ 错误: {e}')