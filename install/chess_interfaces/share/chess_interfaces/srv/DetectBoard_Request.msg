# 棋盘检测服务
# 输入RGB和深度图像，返回棋盘状态

# 请求
sensor_msgs/Image rgb_image      # RGB图像
sensor_msgs/Image depth_image    # 深度图像
bool force_update               # 强制重新检测

