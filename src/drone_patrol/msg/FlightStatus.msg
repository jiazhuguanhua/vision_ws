# 飞行状态消息
Header header
string flight_mode       # 当前飞行模式 (MANUAL, AUTO, PATROL, LANDING, etc.)
geometry_msgs/Pose current_pose    # 当前位置和姿态
geometry_msgs/Twist velocity       # 当前速度
float32 altitude         # 海拔高度
float32 relative_altitude # 相对起飞点高度
bool armed              # 解锁状态
bool gps_fix            # GPS定位状态
int32 satellites        # 卫星数量
float32 battery_voltage # 电池电压
bool mission_active     # 任务执行状态
string current_waypoint # 当前目标航点
