# 简单无人机系统

这是一个重新组织的简单无人机系统，将功能分离为4个独立的ROS包。

## 📦 包结构

### 1. **link_comm** - 通信链路包
- 简单串口通信
- JSON命令解析
- 地面站指令转发

### 2. **flight_control** - 飞行控制包  
- 基础MAVROS控制
- 起飞/降落/航点导航
- 简单巡逻模式

### 3. **animal_detect** - 动物检测包
- USB摄像头接入
- 基础检测框架
- 激光目标发布

### 4. **landing_mode** - 降落模式包
- 简单降落控制
- 45度下降轨迹
- 紧急降落

## 🚀 快速启动

### 编译系统
```bash
cd /home/micoair/vision_ws
catkin_make
source devel/setup.bash
```

### 启动基础系统
```bash
# 启动4个核心节点
roslaunch simple_drone_system simple_drone.launch

# 包含现有waypoint_planner和laser_controller
roslaunch simple_drone_system simple_drone.launch waypoint_planner:=true laser_controller:=true

# 包含MAVROS和摄像头
roslaunch simple_drone_system simple_drone.launch mavros:=true camera:=true
```

### 单独启动节点
```bash
# 单独启动通信链路
rosrun link_comm link_comm_node.py

# 单独启动飞行控制
rosrun flight_control flight_control_node.py

# 单独启动动物检测
rosrun animal_detect animal_detect_node.py

# 单独启动降落模式
rosrun landing_mode landing_mode_node.py
```

## 📡 话题接口

### 输入话题
- `/takeoff_command` (std_msgs/Bool) - 起飞命令
- `/waypoint_command` (geometry_msgs/Point) - 航点命令
- `/ground_command` (std_msgs/String) - 地面站命令
- `/landing_command` (std_msgs/String) - 降落命令

### 输出话题
- `/animal_detection` (std_msgs/String) - 动物检测结果
- `/detection_image` (sensor_msgs/Image) - 检测图像
- `/laser_target` (geometry_msgs/Point) - 激光目标
- `/landing_status` (std_msgs/String) - 降落状态

## 💡 使用示例

### 基本飞行操作
```bash
# 起飞
rostopic pub /takeoff_command std_msgs/Bool "data: true"

# 前往指定位置
rostopic pub /waypoint_command geometry_msgs/Point "x: 10.0, y: 5.0, z: 1.2"

# 开始巡逻
rostopic pub /ground_command std_msgs/String "data: 'START_PATROL'"

# 返回起飞点
rostopic pub /ground_command std_msgs/String "data: 'RETURN_HOME'"
```

### 降落操作
```bash
# 开始降落
rostopic pub /landing_command std_msgs/String "data: 'START_LANDING'"

# 停止降落
rostopic pub /landing_command std_msgs/String "data: 'STOP_LANDING'"

# 紧急降落
rostopic pub /landing_command std_msgs/String "data: 'EMERGENCY_LAND'"
```

### 串口命令 (JSON格式)
```json
{"type": "takeoff"}
{"type": "goto", "x": 10, "y": 5, "z": 1.2}
{"type": "patrol"}
{"type": "return"}
```

## 🔧 系统特点

### 简化设计
- 移除复杂的安全检查
- 简化状态管理
- 基础功能实现
- 清晰的模块分离

### 易于扩展
- 每个包独立开发
- 标准ROS话题接口
- 简单参数配置
- 模块化架构

### 兼容现有系统
- 保持与waypoint_planner兼容
- 保持与laser_controller兼容
- 标准MAVROS接口
- USB摄像头支持

## 📋 开发说明

每个包都可以独立开发和测试：

1. **link_comm**: 专注串口通信和命令解析
2. **flight_control**: 专注MAVROS飞行控制
3. **animal_detect**: 专注图像处理和检测
4. **landing_mode**: 专注降落逻辑

这样的设计让每个功能模块都很简单，易于理解和维护。

---

版本: 1.0.0 (简化版)
开发者: 无人机系统团队
