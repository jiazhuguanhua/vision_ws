# 🔒 严格航点规划系统

## ⚠️ 重要变更: 严格遵守规划数据

系统现在**严格要求**必须有有效的航点规划数据才能执行飞行任务。

### 🚫 拒绝策略

#### 以下情况下命令将被**拒绝**:
- ❌ 没有启动waypoint_planner
- ❌ waypoint_planner未发布任何航点数据
- ❌ /waypoints话题为空或无数据

#### 被拒绝的命令:
- `START_PATROL` - 需要规划航点
- `START_PLANNED_MISSION` - 需要规划航点

#### 仍然可用的命令:
- ✅ `RETURN_HOME` - 返回起飞点 (不需要规划数据)
- ✅ `CHECK_WAYPOINTS` - 检查航点状态
- ✅ 单独的 `/waypoint_command` - 手动航点命令

## 🔄 正确的使用流程

### 1. 启动顺序 (必须按顺序)
```bash
# 第一步: 启动waypoint_planner
roslaunch waypoint_planner plan.launch start_point:="A1B1" no_fly_zones:="A3B3 A4B3"

# 第二步: 启动flight_control (会等待航点数据)
rosrun flight_control flight_control_node.py

# 第三步: 其他节点...
```

### 2. 检查系统状态
```bash
# 检查航点状态
rostopic pub /ground_command std_msgs/String "data: 'CHECK_WAYPOINTS'"

# 查看航点数据
rostopic echo /waypoints -n 1
```

### 3. 执行任务 (只有在有航点数据后)
```bash
# 执行规划任务
rostopic pub /ground_command std_msgs/String "data: 'START_PLANNED_MISSION'"

# 开始巡逻
rostopic pub /ground_command std_msgs/String "data: 'START_PATROL'"
```

## 📊 系统状态指示

### 🔴 等待状态 (无航点数据)
```
⚠️  等待waypoint_planner提供航点数据...
请确保waypoint_planner正在运行并发布/waypoints话题
在接收到航点数据之前，巡逻和规划任务命令将被拒绝
```

### 🟢 就绪状态 (有航点数据)
```
==================================================
✅ 成功接收规划航点: 98个
航点规划完成，系统准备就绪
航点1: (0.0, 0.0, 1.2)
航点2: (1.0, 0.0, 1.2)
...
现在可以发送 START_PATROL 或 START_PLANNED_MISSION 命令
==================================================
```

### ❌ 命令被拒绝
```
ERROR: 没有规划航点数据！
请先启动waypoint_planner或等待航点规划完成
巡逻命令被拒绝 - 必须先有有效的航点规划
```

## 🎯 测试验证

### 运行测试脚本
```bash
./test_strict_waypoints.sh
```

### 手动测试步骤

1. **测试无航点时的行为**:
```bash
# 只启动flight_control (不启动waypoint_planner)
rosrun flight_control flight_control_node.py

# 尝试巡逻 (应该被拒绝)
rostopic pub /ground_command std_msgs/String "data: 'START_PATROL'"
```

2. **测试有航点时的行为**:
```bash
# 启动waypoint_planner
roslaunch waypoint_planner plan.launch

# 等待几秒后，再次尝试巡逻 (应该成功)
rostopic pub /ground_command std_msgs/String "data: 'START_PATROL'"
```

## 📡 串口JSON命令

### 新增检查命令
```json
{"type": "check"}      // 检查航点状态
{"type": "patrol"}     // 开始巡逻 (需要航点数据)
{"type": "return"}     // 返回起飞点 (始终可用)
```

## 💡 设计原则

### ✅ 安全优先
- 不允许使用默认/硬编码路径
- 必须基于实际规划数据飞行
- 明确的错误提示和状态反馈

### ✅ 数据驱动
- 严格遵守waypoint_planner的规划结果
- 实时监控航点数据的可用性
- 透明的状态显示

### ✅ 用户友好
- 清晰的错误消息和指导
- 状态检查命令
- 详细的日志输出

---

## 🔧 故障排除

### 问题: 命令一直被拒绝
**解决方案**: 
1. 检查waypoint_planner是否运行: `rosnode list | grep waypoint`
2. 检查航点话题: `rostopic echo /waypoints -n 1`
3. 检查航点状态: `rostopic pub /ground_command std_msgs/String "data: 'CHECK_WAYPOINTS'"`

### 问题: waypoint_planner运行但flight_control没收到数据
**解决方案**:
1. 检查话题连接: `rostopic info /waypoints`
2. 重启flight_control节点
3. 检查ROS网络连接

现在系统真正做到了**严格遵守规划数据**，没有规划就不飞行！🛡️
