# 无人机自主任务系统状态机逻辑

## 总体架构

```
┌─────────────────┐    ┌─────────────────┐    ┌─────────────────┐    ┌─────────────────┐
│  flight_control │    │apriltag_detector│    │  laser_control  │    │precision_landing│
│   (主控制器)     │    │   (视觉检测)     │    │   (激光控制)     │    │   (精确降落)     │
└─────────────────┘    └─────────────────┘    └─────────────────┘    └─────────────────┘
         │                        │                        │                        │
         │                        │                        │                        │
         └─────────────┬──────────┴──────────┬─────────────┴────────────────┬───────┘
                       │                     │                             │
                    MAVROS                 ROS Topics               Mission State
                  (PX4飞控)              (数据交换)                   (状态同步)
```

## 状态机流程图

```
                    [开始]
                      │
                      ▼
              ┌─────────────┐
              │    INIT     │ ◄── 初始化系统，等待飞控连接
              │ (初始化状态)  │
              └─────────────┘
                      │ 飞控连接成功
                      ▼
              ┌─────────────┐
              │   TAKEOFF   │ ◄── 切换OFFBOARD模式，解锁并起飞
              │  (起飞状态)   │
              └─────────────┘
                      │ 到达起飞高度
                      ▼
              ┌─────────────┐
              │GOTO_MISSION │ ◄── 飞往任务区航点
              │ (前往任务区)  │
              └─────────────┘
                      │ 到达任务区
                      ▼
              ┌─────────────┐
              │  SCAN_TAG   │ ◄── 悬停搜索AprilTag (ID=0)
              │ (扫描二维码)  │
              └─────────────┘
                      │ 检测到目标tag
                      ▼
              ┌─────────────┐
              │ LASER_FIRE  │ ◄── 对准并发射激光
              │ (激光发射)    │
              └─────────────┘
                      │ 激光发射完成
                      ▼
              ┌─────────────┐
              │GOTO_LANDING │ ◄── 飞往降落区航点
              │ (前往降落区)  │
              └─────────────┘
                      │ 到达降落区
                      ▼
              ┌─────────────┐
              │PRECISION_LAND│ ◄── 基于AprilTag精确降落 (ID=1)
              │ (精确降落)    │
              └─────────────┘
                      │ 接近地面
                      ▼
              ┌─────────────┐
              │    LAND     │ ◄── 切换AUTO.LAND模式
              │  (着陆状态)   │
              └─────────────┘
                      │ 成功着陆
                      ▼
              ┌─────────────┐
              │  COMPLETE   │ ◄── 任务完成，上锁
              │ (任务完成)    │
              └─────────────┘
                      │
                      ▼
                   [结束]

                    ┌─────────────┐
          任何状态 ──►│    ERROR    │ ◄── 超时或异常错误
           超时/异常   │  (错误状态)   │
                    └─────────────┘
                            │
                            ▼
                    紧急AUTO.LAND降落
```

## 各节点职责详细说明

### 1. flight_control (主控制器)
**主要功能：**
- 任务状态机调度
- 起飞控制
- 航点导航
- 模式切换
- 异常处理

**状态转换条件：**
- INIT → TAKEOFF: 飞控连接且初始化完成
- TAKEOFF → GOTO_MISSION: 到达起飞高度(±0.2m)
- GOTO_MISSION → SCAN_TAG: 到达任务区(误差<0.3m)
- SCAN_TAG → LASER_FIRE: 检测到AprilTag(ID=0)
- LASER_FIRE → GOTO_LANDING: 激光发射时间结束
- GOTO_LANDING → PRECISION_LAND: 到达降落区(误差<0.3m)
- PRECISION_LAND → LAND: 高度<0.5m
- LAND → COMPLETE: 检测到着陆状态

**安全机制：**
- 每个状态都有超时保护
- 失去飞控连接自动进入ERROR状态
- ERROR状态下强制AUTO.LAND降落

### 2. apriltag_detector (视觉检测)
**主要功能：**
- 实时AprilTag检测
- 3D位姿估计
- 发布检测结果
- 调试图像生成

**检测策略：**
- 支持多个Tag ID区分用途
- ID=0: 任务区目标标识
- ID=1: 降落区精确降落标识
- 提供检测置信度和位姿精度

**输出数据：**
- `/apriltag/detection`: 检测结果消息
- `/apriltag/pose`: 3D位姿(PoseStamped)
- `/apriltag/debug_image`: 调试可视化图像

### 3. laser_control (激光控制)
**主要功能：**
- GPIO/串口激光器控制
- 安全超时保护
- 状态监控发布

**控制逻辑：**
- 接收`/laser_fire`布尔命令
- 自动定时关闭(防止过热)
- 安全超时强制关闭
- 支持GPIO和串口两种控制方式

**安全特性：**
- 最大发射时间限制(默认3秒)
- 安全超时保护(默认10秒)
- 程序退出时自动关闭激光

### 4. precision_landing (精确降落)
**主要功能：**
- 基于AprilTag的精确位置控制
- PID控制器实现稳定降落
- 安全条件监控

**控制算法：**
- X/Y轴: PID控制保持tag居中
- Z轴: 恒定速率下降
- 姿态安全监控(防止过度倾斜)

**安全机制：**
- Tag丢失超时保护
- 姿态角度限制
- 最低安全高度限制
- 异常时自动切换AUTO.LAND

## 话题通信架构

```
flight_control           apriltag_detector          laser_control         precision_landing
      │                        │                         │                       │
      ├─► /mavros/setpoint... ─┤                         │                       │
      ├─► /laser_fire ─────────┼────────────────────────►│                       │
      ├─► /mission/state ──────┼─────────────────────────┼──────────────────────►│
      │                        │                         │                       │
      │◄── /apriltag/detection ┤                         │                       │
      │                        ├─► /apriltag/pose ──────┼──────────────────────►│
      │                        ├─► /apriltag/debug_image │                       │
      │                        │                         │                       │
      │◄── /mavros/state ──────┤                         │                       │
      │◄── /mavros/local_pos...┤                         │                       │
      │                        │                         │                       │
      │                        │                         ├─► /laser_status       │
      │                        │                         │                       │
      │                        │                         │                       ├─► /mavros/setpoint...
```

## 配置参数结构

**flight_control:**
- takeoff: 起飞高度、超时时间
- waypoints: 任务区和降落区坐标
- flight: 飞行速度、位置容差、悬停时间
- control_rate: 控制循环频率

**apriltag_detector:**
- detection: AprilTag检测参数
- camera: 相机内参和畸变参数
- tags: Tag尺寸和ID配置
- publish: 发布话题配置

**laser_control:**
- gpio: GPIO引脚配置
- laser: 发射时长和安全超时
- serial: 串口通信参数

**precision_landing:**
- pid: 三轴PID控制器参数
- landing: 降落高度和速度参数
- safety: 安全限制和超时配置

## 调试和监控

**日志系统:**
- 所有节点使用统一的rospy.loginfo/warn/error
- 关键状态转换记录详细日志
- 异常情况记录错误堆栈

**调试工具:**
- AprilTag检测可视化图像
- 任务状态实时发布
- 激光状态监控
- MAVROS状态监控

**故障诊断:**
- 状态机超时检测
- 通信中断检测
- 硬件故障检测
- 自动故障恢复或安全降落

## 部署和使用

1. **编译工作空间:**
   ```bash
   cd ~/vision_ws
   catkin_make
   source devel/setup.bash
   ```

2. **启动完整系统:**
   ```bash
   roslaunch flight_control full_mission.launch
   ```

3. **单独调试节点:**
   ```bash
   # 仅AprilTag检测
   rosrun apriltag_detector apriltag_detector_node.py
   
   # 仅激光控制
   rosrun laser_control laser_control_node.py
   
   # 仅精确降落
   rosrun precision_landing precision_landing_node.py
   ```

4. **监控任务状态:**
   ```bash
   rostopic echo /mission/state
   rostopic echo /apriltag/detection
   rostopic echo /laser_status
   ```
