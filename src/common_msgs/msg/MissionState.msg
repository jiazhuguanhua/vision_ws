# 任务状态消息
# Mission state message

# 任务状态枚举
uint8 INIT = 0
uint8 TAKEOFF = 1
uint8 GOTO_MISSION = 2
uint8 SCAN_TAG = 3
uint8 LASER_FIRE = 4
uint8 GOTO_LANDING = 5
uint8 PRECISION_LAND = 6
uint8 LAND = 7
uint8 COMPLETE = 8
uint8 ERROR = 9

# 当前状态
uint8 current_state

# 状态描述
string state_description

# 任务进度 (0-100)
float32 progress

# 错误信息 (仅在ERROR状态时有效)
string error_message

# 时间戳
time timestamp
