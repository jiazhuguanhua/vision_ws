# 无人机自主任务系统使用说明

## 系统概述

本系统实现了一个完整的无人机自主任务解决方案，包括：
- 自主起飞和航点导航
- 基于AprilTag的目标识别
- 激光发射控制
- 精确视觉降落

## 快速开始

### 1. 安装依赖

```bash
cd ~/vision_ws
./install_dependencies.sh
```

### 2. 编译工作空间

```bash
cd ~/vision_ws
catkin_make
source devel/setup.bash
```

### 3. 配置参数

编辑配置文件 `src/flight_control/config/mission_config.yaml`:

```yaml
# 重要参数配置
flight_control:
  waypoints:
    mission_area: [5.0, 5.0, 2.0]    # 任务区坐标 [x, y, z]
    landing_area: [10.0, 0.0, 2.0]   # 降落区坐标 [x, y, z]
  
apriltag_detector:
  camera:
    fx: 615.0  # 根据实际相机标定结果修改
    fy: 615.0
    cx: 320.0
    cy: 240.0
```

### 4. 启动系统

**完整系统启动:**
```bash
roslaunch flight_control full_mission.launch
```

**分步启动(调试用):**
```bash
# 终端1: MAVROS和PX4
roslaunch mavros px4.launch

# 终端2: 相机
roslaunch realsense2_camera rs_camera.launch

# 终端3: AprilTag检测
rosrun apriltag_detector apriltag_detector_node.py

# 终端4: 激光控制
rosrun laser_control laser_control_node.py

# 终端5: 精确降落
rosrun precision_landing precision_landing_node.py

# 终端6: 主控制器
rosrun flight_control flight_control_node.py
```

## 监控和调试

### 查看任务状态
```bash
rostopic echo /mission/state
```

### 查看AprilTag检测
```bash
rostopic echo /apriltag/detection
```

### 查看激光状态
```bash
rostopic echo /laser_status
```

### 可视化调试
```bash
# 启动RViz
rviz -d src/flight_control/config/mission_visualization.rviz

# 查看AprilTag调试图像
rosrun image_view image_view image:=/apriltag/debug_image
```

## 任务流程

1. **INIT**: 系统初始化，等待飞控连接
2. **TAKEOFF**: 自动起飞到指定高度
3. **GOTO_MISSION**: 飞往任务区航点
4. **SCAN_TAG**: 搜索并识别AprilTag (ID=0)
5. **LASER_FIRE**: 对准目标发射激光
6. **GOTO_LANDING**: 飞往降落区航点  
7. **PRECISION_LAND**: 基于AprilTag (ID=1) 精确降落
8. **LAND**: 最终着陆并上锁
9. **COMPLETE**: 任务完成

## 安全注意事项

### 飞行前检查
- [ ] 确保飞行区域安全，无人员和障碍物
- [ ] 检查无人机电池电量充足
- [ ] 验证GPS定位良好(户外飞行)
- [ ] 确认相机工作正常
- [ ] 测试激光器安全性

### 紧急操作
- **手动接管**: 使用遥控器切换到MANUAL模式
- **紧急降落**: 使用遥控器的KILL SWITCH
- **程序终止**: Ctrl+C 停止所有节点

### 故障恢复
- 系统会自动检测超时和异常情况
- 发生错误时会自动切换到AUTO.LAND模式
- 所有状态转换都有超时保护

## 硬件配置

### 必需硬件
- PX4飞控 (Pixhawk 4/5等)
- Intel RealSense T265 (定位)
- Intel RealSense D435i 或单目相机 (视觉)
- 激光发射器模块
- 树莓派或机载计算机

### 硬件连接
```
┌─────────────┐    ┌─────────────┐    ┌─────────────┐
│     PX4     │◄──►│  机载计算机   │◄──►│   RealSense │
│    飞控     │    │(树莓派/NUC)  │    │    相机     │
└─────────────┘    └─────────────┘    └─────────────┘
                           │
                           ▼
                   ┌─────────────┐
                   │   激光器    │
                   │   模块      │
                   └─────────────┘
```

### 激光器连接
**GPIO方式 (树莓派):**
- 激光器正极 → GPIO 18 (可配置)
- 激光器负极 → GND

**串口方式:**
- USB转串口模块连接激光控制器
- 配置波特率和控制指令

## 配置参数详解

### 飞行控制参数
```yaml
flight_control:
  takeoff:
    height: 2.0      # 起飞高度(米)
    timeout: 10.0    # 起飞超时(秒)
  
  waypoints:
    mission_area: [5.0, 5.0, 2.0]   # 任务区坐标
    landing_area: [10.0, 0.0, 2.0]  # 降落区坐标
  
  flight:
    max_velocity: 2.0          # 最大飞行速度
    position_tolerance: 0.3    # 航点到达容差
    hover_time: 2.0           # 航点悬停时间
```

### AprilTag检测参数
```yaml
apriltag_detector:
  detection:
    family: "tag36h11"    # Tag家族
    quad_decimate: 2.0    # 检测精度与速度平衡
  
  tags:
    mission_tag_id: 0     # 任务区Tag ID
    landing_tag_id: 1     # 降落区Tag ID
    tag_size: 0.1         # Tag实际尺寸(米)
```

### 精确降落参数
```yaml
precision_landing:
  pid:
    x_axis: {kp: 0.5, ki: 0.0, kd: 0.1}  # X轴PID参数
    y_axis: {kp: 0.5, ki: 0.0, kd: 0.1}  # Y轴PID参数
    z_axis: {kp: 0.3, ki: 0.0, kd: 0.05} # Z轴PID参数
  
  landing:
    target_height: 0.3    # 目标降落高度
    descent_rate: 0.2     # 下降速度
    position_tolerance: 0.1 # 位置容差
```

## 故障排除

### 常见问题

**1. 无人机不起飞**
- 检查MAVROS连接: `rostopic echo /mavros/state`
- 确认飞控模式: 应显示 `connected: true`
- 检查遥控器是否在正确模式

**2. AprilTag检测不到**
- 检查相机话题: `rostopic echo /camera/infra1/image_rect_raw`
- 验证Tag打印质量和光照条件
- 调整相机曝光和增益参数

**3. 激光不工作**
- 检查GPIO连接和权限
- 验证激光器控制电压
- 查看激光控制日志

**4. 精确降落不稳定**
- 调整PID参数
- 检查AprilTag检测稳定性
- 确认相机标定准确性

### 日志分析
```bash
# 查看系统日志
journalctl -f

# 查看ROS节点日志
rosnode list
rosnode info /flight_control

# 录制调试数据
rosbag record /mission/state /apriltag/detection /mavros/local_position/pose
```

## 扩展开发

### 添加新状态
1. 在 `MissionStates` 枚举中添加新状态
2. 在 `state_machine()` 中添加处理函数
3. 实现状态转换逻辑
4. 更新配置文件超时参数

### 自定义激光控制
1. 继承 `LaserController` 类
2. 重写 `set_laser_state()` 方法
3. 添加新的硬件接口支持

### 改进视觉算法
1. 替换或增强AprilTag检测
2. 添加其他视觉特征检测
3. 实现多目标跟踪

## 技术支持

### 联系方式
- 项目地址: [GitHub链接]
- 技术文档: [文档链接]
- 问题反馈: [Issue链接]

### 贡献代码
欢迎提交Pull Request来改进系统:
1. Fork项目
2. 创建特性分支
3. 提交代码
4. 发起Pull Request
