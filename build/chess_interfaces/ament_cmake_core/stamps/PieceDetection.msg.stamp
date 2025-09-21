# 棋子检测结果消息
# 用于表示单个棋子的检测结果

std_msgs/Header header

# 棋子位置信息
geometry_msgs/Point position_3d     # 3D世界坐标
geometry_msgs/Point position_2d     # 2D图像坐标

# 棋子属性
int8 piece_type         # 棋子类型 (1=王, 2=后, 3=车, 4=象, 5=马, 6=兵)
int8 piece_color        # 棋子颜色 (1=白, -1=黑, 0=未知)
float32 confidence      # 检测置信度 (0.0-1.0)

# 图像数据
sensor_msgs/Image roi_image    # 棋子区域图像