# 动物检测结果消息
string[] animal_types      # 检测到的动物类型数组
uint32[] counts           # 对应每种动物的数量
uint32 total_count        # 总检测数量
geometry_msgs/Point[] positions  # 每个目标相对于无人机的位置
float64[] confidences     # 检测置信度
string[] object_types     # 每个检测目标对应的动物类型
std_msgs/Header header    # 时间戳和坐标系
