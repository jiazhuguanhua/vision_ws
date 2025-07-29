# AprilTag检测结果消息
# AprilTag detection result message

# Tag ID
int32 tag_id

# 检测置信度 (0-1)
float32 confidence

# Tag在相机坐标系下的位姿
geometry_msgs/PoseStamped pose

# Tag在图像中的角点像素坐标
geometry_msgs/Point32[] corners

# Tag在图像中的中心点像素坐标
geometry_msgs/Point32 center

# 检测到的tag数量
int32 tag_count

# 是否检测成功
bool detected
