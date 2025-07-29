# Laser Control 与 AprilTag 协调工作说明

## 系统架构图

```
┌─────────────────┐    ┌─────────────────┐    ┌─────────────────┐
│   USB Camera    │    │  AprilTag       │    │  Mission        │
│     Node        │    │  Detector       │    │  Controller     │
│                 │───▶│                 │───▶│                 │
│ /usb_cam/       │    │ /apriltag/      │    │ 决策逻辑         │
│ image_raw       │    │ detection       │    │                 │
│ camera_info     │    │ pose            │    │                 │
└─────────────────┘    └─────────────────┘    └─────────────────┘
                                                        │
                                                        ▼ 发送激光命令
                                               ┌─────────────────┐
                                               │  Laser Control  │
                                               │     Node        │
                                               │                 │
                                               │ GPIO Pin 18     │───▶ 激光器
                                               │ /laser_fire     │
                                               │ /laser_status   │
                                               └─────────────────┘
```

## 工作流程详解

### 1. **图像采集阶段**
```
USB摄像头 → 图像流 → AprilTag检测器
```
- USB摄像头节点发布图像到 `/usb_cam/image_raw`
- AprilTag检测器订阅图像流进行实时检测

### 2. **AprilTag检测阶段**
```
原始图像 → AprilTag算法 → 检测结果
```
- 检测到的Tag信息包括：
  - Tag ID (识别具体的标记)
  - 3D位姿 (位置和姿态)
  - 检测置信度
  - 像素坐标

### 3. **任务决策阶段**
```
检测结果 → 任务逻辑 → 激光控制命令
```
任务控制器根据检测结果进行决策：
- **检测到任务区Tag (ID=0)**: 发送激光发射命令
- **检测到降落区Tag (ID=1)**: 准备精确降落
- **未检测到Tag**: 继续搜索或执行其他任务

### 4. **激光控制阶段**
```
/laser_fire 命令 → GPIO控制 → 激光器动作
```

## ROS话题通信详解

### 主要话题：

1. **图像数据流**
   ```
   /usb_cam/image_raw        (sensor_msgs/Image)
   /usb_cam/camera_info      (sensor_msgs/CameraInfo)
   ```

2. **AprilTag检测结果**
   ```
   /apriltag/detection       (common_msgs/AprilTagDetection)
   /apriltag/pose           (geometry_msgs/PoseStamped)
   /apriltag/debug_image    (sensor_msgs/Image)
   ```

3. **激光控制**
   ```
   /laser_fire              (std_msgs/Bool)        # 激光发射命令
   /laser_status            (std_msgs/Bool)        # 激光状态反馈
   ```

## 关键配置参数

### AprilTag检测器配置
```yaml
apriltag_detector:
  tags:
    mission_tag_id: 0        # 任务区标记ID
    landing_tag_id: 1        # 降落区标记ID
    tag_size: 0.1           # 标记实际尺寸(米)
```

### 激光控制配置
```yaml
laser_control:
  gpio:
    pin: 18                 # GPIO引脚号
    active_high: true       # 高电平有效
  laser:
    fire_duration: 3.0      # 发射持续时间(秒)
    safety_timeout: 10.0    # 安全超时时间(秒)
```

## GPIO控制详解

### GPIO引脚控制原理
```python
# GPIO初始化
GPIO.setmode(GPIO.BCM)           # 使用BCM引脚编号
GPIO.setup(18, GPIO.OUT)         # 设置引脚18为输出

# 激光开启
GPIO.output(18, GPIO.HIGH)       # 高电平 → 激光开启

# 激光关闭  
GPIO.output(18, GPIO.LOW)        # 低电平 → 激光关闭
```

### 安全机制
1. **定时关闭**: 发射3秒后自动关闭
2. **安全超时**: 10秒强制关闭，防止意外
3. **紧急停止**: 任何异常情况立即关闭
4. **资源清理**: 程序退出时确保GPIO清理

## 实际应用场景

### 无人机自主打击任务
1. **起飞**: 无人机起飞到指定高度
2. **搜索**: 飞行到任务区域搜索AprilTag
3. **识别**: AprilTag检测器识别到目标标记(ID=0)
4. **瞄准**: 根据Tag位姿调整无人机位置
5. **发射**: 任务控制器发送 `/laser_fire true` 命令
6. **执行**: 激光控制器控制GPIO引脚，激光器发射3秒
7. **确认**: 通过 `/laser_status` 确认激光状态
8. **返航**: 飞行到降落区寻找降落标记(ID=1)

### 协调工作的关键点

1. **时序协调**: AprilTag检测 → 位姿确认 → 激光发射
2. **状态反馈**: 所有节点都有状态反馈，确保系统可监控
3. **安全机制**: 多重安全保护，防止激光器异常工作
4. **容错设计**: 检测失败、硬件故障都有对应处理机制

## 启动命令

```bash
# 启动完整系统
roslaunch apriltag_detector usb_cam_apriltag.launch

# 启动激光控制器
rosrun laser_control laser_control_node.py

# 手动测试激光器
rostopic pub /laser_fire std_msgs/Bool "data: true"
```
