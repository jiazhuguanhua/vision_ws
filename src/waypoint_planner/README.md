# 野生动物巡查航点规划系统

## 📋 功能概述

这个ROS包实现了无人机野生动物巡查任务的自动航点规划功能，支持：

- 🗺️ 9×9网格地图坐标系统（A1B1 到 A9B9）
- 🚫 禁飞区自动避障
- 🔄 全覆盖路径规划，遍历所有可通行点
- 🏠 自动返回起点形成闭环
- 🎯 A*算法优化路径
- 📡 ROS话题发布航点序列
- 🎨 RViz可视化显示
- ✈️ 2D路径规划，固定飞行高度

## 📦 包结构

```
waypoint_planner/
├── CMakeLists.txt              # CMake构建文件
├── package.xml                 # 包依赖配置
├── launch/
│   └── plan.launch            # 启动文件
├── src/
│   ├── planner_node.py        # 主规划节点
│   └── test_waypoint_planner.py # 测试脚本
├── msg/
│   └── PointArray.msg         # 自定义消息类型
└── config/
    ├── map.yaml               # 地图配置
    └── waypoint_planner.rviz  # RViz配置
```

## 🚀 快速开始

### 1. 编译包

```bash
cd ~/vision_ws
catkin_make
source devel/setup.bash
```

### 2. 基本使用

```bash
# 使用默认参数启动
roslaunch waypoint_planner plan.launch

# 自定义起点和禁飞区
roslaunch waypoint_planner plan.launch start_point:="A9B1" no_fly_zones:="A8B2 A7B3 A2B3"

# 不启动RViz
roslaunch waypoint_planner plan.launch rviz:=false
```

### 3. 查看航点结果

```bash
# 查看航点序列
rostopic echo /waypoints

# 查看话题信息
rostopic info /waypoints
```

## 📐 坐标系统

### 网格坐标格式
- **格式**: `A{行号}B{列号}`
- **范围**: A1B1 到 A9B9（9×9网格）
- **示例**: A9B1 表示第9行第1列

### 世界坐标转换
- 网格坐标(1,1) → 世界坐标(0,0)
- 网格坐标(9,1) → 世界坐标(0,8)
- Z坐标为固定飞行高度（默认1.2米，不变化）
- **注意**：这是2D规划问题，所有航点保持相同高度

## 🛠️ 参数配置

### 启动参数

| 参数 | 类型 | 默认值 | 说明 |
|------|------|--------|------|
| `start_point` | string | "A1B1" | 起飞点坐标 |
| `no_fly_zones` | string | "A3B3 A4B3 A5B3" | 禁飞区列表（空格分隔） |
| `altitude` | float | 1.2 | 固定飞行高度（米，2D规划） |
| `config_file` | string | map.yaml路径 | 配置文件路径 |

### 配置文件参数

```yaml
# config/map.yaml
map:
  size:
    rows: 9    # 地图行数 (A1-A9)
    cols: 9    # 地图列数 (B1-B9)
  cell_size: 1.0  # 网格单元大小（米）

mission:
  start_point: "A1B1"
  no_fly_zones: ["A3B3", "A4B3", "A5B3"]
  altitude: 1.2  # 固定飞行高度（2D规划）
  algorithm: "astar"  # 路径规划算法
```

## 📡 ROS接口

### 发布话题

| 话题名 | 消息类型 | 说明 |
|--------|----------|------|
| `/waypoints` | `waypoint_planner/PointArray` | 航点序列 |
| `/waypoint_viz` | `visualization_msgs/MarkerArray` | RViz可视化标记 |

### 自定义消息

```
# waypoint_planner/PointArray.msg
geometry_msgs/Point[] points
std_msgs/Header header
```

## 🎨 可视化

### RViz显示内容
- 🟢 **绿色线条**: 飞行路径
- 🔵 **蓝色球体**: 起点
- 🔴 **红色方块**: 禁飞区
- ⚪ **白色数字**: 航点编号（每5个显示一个）
- ⬜ **网格背景**: 地图网格

### 启动RViz
```bash
# 启动RViz（需要时手动启动）
roslaunch waypoint_planner plan.launch rviz:=true

# 或手动启动
rviz -d $(rospack find waypoint_planner)/config/waypoint_planner.rviz
```

## 🧪 测试

### 运行测试脚本
```bash
# 在另一个终端运行测试
rosrun waypoint_planner test_waypoint_planner.py
```

### 手动测试命令
```bash
# 测试不同起点
roslaunch waypoint_planner plan.launch start_point:="A1B1"
roslaunch waypoint_planner plan.launch start_point:="A9B9"

# 测试不同禁飞区
roslaunch waypoint_planner plan.launch no_fly_zones:="A5B5 A6B5 A7B5"

# 测试复杂禁飞区
roslaunch waypoint_planner plan.launch no_fly_zones:="A2B2 A2B3 A3B2 A3B3 A7B7 A8B7 A7B8 A8B8"
```

## 🔧 算法说明

### 路径规划算法
1. **A*算法**: 用于计算两点间最短路径
2. **贪心策略**: 全覆盖路径规划，每次选择距离最近的未访问点
3. **8连通**: 支持8个方向移动（包括对角线）
4. **避障**: 自动避开禁飞区域

### 算法特点
- ✅ 保证遍历所有可通行点
- ✅ 自动返回起点形成闭环
- ✅ 路径长度优化
- ✅ 实时计算，适合仿真使用
- ✅ 2D路径规划，固定飞行高度
- ✅ 纯平面运动，无高度变化

## 🔍 故障排除

### 常见问题

1. **编译错误**
   ```bash
   # 确保安装所有依赖
   rosdep install --from-paths src --ignore-src -r -y
   ```

2. **找不到消息类型**
   ```bash
   # 重新编译并source
   catkin_make
   source devel/setup.bash
   ```

3. **RViz不显示**
   - 检查Fixed Frame是否设为"map"
   - 确保MarkerArray话题为"/waypoint_viz"

4. **航点规划失败**
   - 检查起点是否在禁飞区内
   - 确保坐标格式正确（如"A9B1"）

### 调试命令
```bash
# 查看节点状态
rosnode list
rosnode info /waypoint_planner

# 查看话题
rostopic list
rostopic info /waypoints

# 查看日志
rosrun rqt_console rqt_console
```

## 📈 性能指标

- **地图大小**: 9×9网格
- **最大航点数**: ~81个（取决于禁飞区配置）
- **规划时间**: < 1秒
- **内存使用**: < 50MB
- **支持实时更新**: 是
- **规划类型**: 2D路径规划，固定高度

## 🔮 扩展功能

### 可扩展特性
- 📏 支持自定义地图大小
- 🧮 可更换路径规划算法
- 📊 支持路径优化指标调整
- 🎯 可添加优先访问区域
- 🔄 支持动态禁飞区更新

## 📞 技术支持

如遇到问题，请检查：
1. ROS Noetic是否正确安装
2. 包依赖是否满足
3. Python3环境是否配置正确
4. 参数格式是否符合要求

---

**祝您的野生动物巡查任务顺利！** 🦅🚁
