# 降落命令消息
Header header
string command           # 降落命令 (START_LANDING, ABORT_LANDING, EMERGENCY_LAND)
geometry_msgs/Point landing_position  # 降落目标位置
float32 descent_angle    # 下降角度 (度)
float32 descent_speed    # 下降速度 (m/s)
bool led_enable         # 是否启用LED指示
int32 led_pattern       # LED闪烁模式
bool precision_landing  # 精确降落模式
string landing_reason   # 降落原因
