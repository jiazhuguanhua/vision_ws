# Laser Control Node v3.0 升级说明

## 新增功能

### 常亮模式 (Continuous Mode)
- **话题**: `/laser_continuous` (std_msgs/Bool)
- **功能**: 控制激光器常亮模式的开关

### 优先级逻辑
1. **常亮模式优先级最高**
   - 当 `/laser_continuous` 为 `True` 时，激光器保持常亮
   - 此时忽略所有 `/laser_fire` 命令

2. **定时发射模式**
   - 当 `/laser_continuous` 为 `False` 时，激光器进入定时发射模式
   - `/laser_fire` 命令生效，可以进行定时发射

## 话题接口

### 订阅话题
- `/laser_fire` (std_msgs/Bool): 定时发射控制
- `/laser_continuous` (std_msgs/Bool): 常亮模式控制

### 发布话题
- `/laser_status` (std_msgs/Bool): 激光器当前状态

## 使用示例

### 1. 常亮模式
```bash
# 启用常亮模式
rostopic pub /laser_continuous std_msgs/Bool "data: true"

# 关闭常亮模式
rostopic pub /laser_continuous std_msgs/Bool "data: false"
```

### 2. 定时发射模式
```bash
# 发射激光（定时关闭）
rostopic pub /laser_fire std_msgs/Bool "data: true"

# 手动停止激光
rostopic pub /laser_fire std_msgs/Bool "data: false"
```

### 3. 监控状态
```bash
# 监控激光状态
rostopic echo /laser_status
```

## 测试脚本
运行测试脚本验证所有功能：
```bash
rosrun laser_control test_laser_control.py
```

## 安全特性
- 程序退出时自动关闭激光器
- GPIO资源清理
- 异常处理和错误恢复
- 详细的日志记录

## 配置参数
- `gpio_pin`: GPIO引脚号 (默认: 18)
- `active_high`: 高电平有效 (默认: true)
- `fire_duration`: 发射持续时间 (默认: 3.0秒)
