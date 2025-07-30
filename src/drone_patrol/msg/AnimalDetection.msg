# 动物检测消息
Header header
int32 animal_id          # 动物类型ID (1-鹿, 2-兔子, 3-狐狸, 等)
string animal_type       # 动物类型名称
float32 confidence       # 检测置信度 (0.0-1.0)
geometry_msgs/Point position_2d  # 图像中的2D坐标 (x, y, z=0)
geometry_msgs/Point position_3d  # 世界坐标中的3D位置 (如果可用)
sensor_msgs/Image image  # 检测时的图像
bool tracking_active     # 是否正在跟踪
float32 distance         # 估计距离(米)
