# 无人机野生动物巡逻系统

这是一个完整的ROS包，用于实现无人机野生动物巡逻和监控系统。

## 功能特性

### 1. 通信链路节点 (link_comm.py)
- 串口和BLE通信支持
- 地面站命令接收和转发
- 实时状态数据上传
- JSON格式命令解析

### 2. 飞行控制节点 (flight_control.py)
- 集成MAVROS和T265定位
- NED坐标系支持
- 自动起飞和降落
- 航点导航和巡逻模式
- 安全监控和紧急处理

### 3. 动物检测节点 (animal_detect.py)
- YOLO模型动物识别
- USB摄像头实时图像处理
- AprilTag检测支持
- 自动激光跟踪
- 检测结果实时发布

### 4. 降落模式节点 (landing_mode.py)
- 45度角精确降落
- LED状态指示
- 安全检查和避障
- 紧急降落处理
- 降落轨迹规划

## 系统架构

```
地面站 <--> link_comm <--> flight_control <--> MAVROS <--> PX4
                |              |
                |              v
                |         waypoint_planner
                |              |
                v              v
         animal_detect <--> laser_controller
                |
                v
         landing_mode
```

## 安装和使用

### 1. 编译包
```bash
cd /home/micoair/vision_ws
catkin_make
source devel/setup.bash
```

### 2. 安装依赖
```bash
# Python依赖
pip3 install opencv-python numpy pyserial

# YOLO模型文件 (需要下载)
mkdir -p /home/micoair/yolo
# 下载 yolov4.weights, yolov4.cfg, coco.names 到该目录

# USB摄像头驱动
sudo apt install ros-noetic-usb-cam

# AprilTag库 (可选)
sudo apt install ros-noetic-apriltag-ros
pip3 install apriltag
```

### 3. 启动系统
```bash
# 启动完整系统
roslaunch drone_patrol drone_patrol.launch

# 启动时显示图像
roslaunch drone_patrol drone_patrol.launch image_view:=true

# 启动时显示RViz
roslaunch drone_patrol drone_patrol.launch rviz:=true
```

## 消息类型

### AnimalDetection.msg
- 动物检测结果
- 包含类型、置信度、位置信息

### LinkStatus.msg  
- 通信链路状态
- BLE/UART连接状态和信号强度

### FlightStatus.msg
- 飞行状态信息
- 位置、速度、电池、GPS等

### LandingCommand.msg
- 降落命令和参数
- 支持多种降落模式

## 配置参数

主要配置文件: `config/drone_patrol_params.yaml`

### 通信配置
- 串口设备和波特率
- BLE设备参数
- 状态发布频率

### 飞行参数
- 起飞高度和巡航速度
- 位置容差和安全限制
- 最大/最小飞行高度

### 检测参数
- YOLO模型路径和阈值
- 摄像头分辨率和帧率
- AprilTag和激光控制

### 降落参数
- 下降角度和速度
- LED控制和安全半径
- 最低安全高度

## 坐标系统

### NED坐标系
- 北-东-下坐标系
- 起飞点为原点 (0,0,0)
- X轴指向北，Y轴指向东，Z轴向下

### 网格坐标
- 9x9网格，每格5米
- A1-A9, B1-B9 到 I1-I9
- 支持禁飞区设置

## 使用说明

### 1. 基本操作
```bash
# 起飞
rostopic pub /drone_patrol/takeoff_command std_msgs/Bool "data: true"

# 前往指定位置 (NED坐标)
rostopic pub /drone_patrol/waypoint_command geometry_msgs/Point "x: 10.0, y: 5.0, z: 1.2"

# 开始巡逻
rostopic pub /drone_patrol/ground_command std_msgs/String "data: 'START_PATROL'"

# 返回起飞点
rostopic pub /drone_patrol/ground_command std_msgs/String "data: 'RETURN_HOME'"
```

### 2. 降落操作
```bash
# 普通降落
rostopic pub /drone_patrol/landing_command drone_patrol/LandingCommand "
command: 'START_LANDING'
descent_angle: 45.0
descent_speed: 0.5
led_enable: true
landing_reason: 'Manual command'"

# 紧急降落
rostopic pub /drone_patrol/landing_command drone_patrol/LandingCommand "
command: 'EMERGENCY_LAND'"
```

### 3. 动物检测控制
```bash
# 启用/禁用检测
rostopic pub /drone_patrol/detection_enable std_msgs/Bool "data: true"

# 手动跟踪目标
rostopic pub /drone_patrol/track_target geometry_msgs/Point "x: 1.0, y: 2.0, z: 0.0"
```

## 监控话题

### 状态监控
```bash
# 飞行状态
rostopic echo /drone_patrol/flight_status

# 通信状态  
rostopic echo /drone_patrol/link_status

# 动物检测结果
rostopic echo /drone_patrol/animal_detection
```

### 图像监控
```bash
# 检测图像 (带标注)
rostopic echo /drone_patrol/detection_image

# 原始摄像头图像
rostopic echo /usb_cam/image_raw
```

## 故障排除

### 1. 摄像头问题
```bash
# 检查USB摄像头
ls /dev/video*
v4l2-ctl --list-devices

# 测试摄像头
rosrun usb_cam usb_cam_node
```

### 2. 串口通信问题
```bash
# 检查串口设备
ls /dev/ttyUSB*
sudo chmod 666 /dev/ttyUSB0

# 测试串口通信
sudo minicom -D /dev/ttyUSB0 -b 115200
```

### 3. MAVROS连接问题
```bash
# 检查MAVROS状态
rostopic echo /mavros/state

# 检查PX4连接
rostopic echo /mavros/version
```

## 扩展功能

### 1. 自定义动物模型
- 替换YOLO权重文件
- 修改类别映射
- 调整检测阈值

### 2. 高级航点规划
- 集成现有waypoint_planner
- 添加动态避障
- 优化巡逻路径

### 3. 数据记录
- 飞行轨迹记录
- 检测结果保存
- 图像数据存储

## 安全注意事项

1. 确保在安全区域内测试
2. 检查电池电量和信号强度
3. 设置合适的禁飞区
4. 保持手动控制准备
5. 遵守当地无人机法规

## 支持和维护

- 定期更新YOLO模型
- 校准相机参数
- 检查硬件连接
- 监控系统日志

---

开发者: 无人机巡逻系统团队
版本: 1.0.0
日期: 2024年
