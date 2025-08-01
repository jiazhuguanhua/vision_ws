# 双激光头控制系统优化说明

## 📋 更新内容

### 1. 激光控制节点升级 (v4.0)
- 从单激光头升级为**双激光头支持** (GPIO18 和 GPIO19)
- 新增支持 **common_msgs/LaserControl** 消息格式
- 保持与旧版 Bool 消息的兼容性
- 独立的激光头状态管理和定时器控制

### 2. 消息格式优化
- **LaserControl.msg** 移至 `common_msgs` 包
- 消息格式：
  ```
  uint8 laser_id      # 激光头ID (1或2)  
  bool laser_state    # 激光开关状态 (true=开启, false=关闭)
  ```

### 3. 动物检测节点测试脚本
- 创建了完整的测试脚本 `test_animal_detect.py`
- 支持综合测试和简单测试两种模式
- 包含图像模拟、检测结果验证等功能

## 🚀 使用方法

### 启动双激光控制节点
```bash
# 设置环境
source /home/micoair/vision_ws/devel/setup.bash

# 启动激光控制节点
rosrun laser_control laser_control_node.py
```

### 使用新的LaserControl消息
```python
from common_msgs.msg import LaserControl

# 发布者
laser_pub = rospy.Publisher('/laser_control', LaserControl, queue_size=10)

# 控制激光1
laser_msg = LaserControl()
laser_msg.laser_id = 1
laser_msg.laser_state = True  # 开启
laser_pub.publish(laser_msg)

# 控制激光2  
laser_msg.laser_id = 2
laser_msg.laser_state = False  # 关闭
laser_pub.publish(laser_msg)
```

### 测试激光控制系统
```bash
# 完整测试
rosrun laser_control test_laser_control.py

# 简单测试
rosrun laser_control test_laser_control.py _test_type:=simple
```

### 测试动物检测节点
```bash
# 先启动动物检测节点
rosrun animal_detect animal_detect_node.py

# 然后运行测试
rosrun animal_detect test_animal_detect.py

# 或使用启动脚本
cd /home/micoair/vision_ws
./test_animal_detection.sh
```

## 🔧 硬件配置

### GPIO引脚分配
- **激光1**: GPIO 18
- **激光2**: GPIO 19
- **电平逻辑**: 高电平有效 (可配置)
- **自动关闭时间**: 3.0秒 (可配置)

### 参数配置
```yaml
# launch文件或yaml配置
laser_control:
  gpio:
    active_high: true    # 高电平有效
  laser:
    fire_duration: 3.0   # 发射持续时间(秒)
```

## 📡 ROS话题

### 发布的话题
- `/laser_status` (std_msgs/Bool) - 激光状态（任一激光开启为true）

### 订阅的话题
- `/laser_control` (common_msgs/LaserControl) - **新的激光控制消息**
- `/laser_fire` (std_msgs/Bool) - 兼容性支持，控制激光1
- `/laser_continuous` (std_msgs/Bool) - 兼容性支持，激光1常亮模式

## 🧪 测试功能

### 激光控制测试
1. **LaserControl消息测试** - 分别控制两个激光头
2. **同时控制测试** - 双激光同时开启/关闭
3. **兼容性测试** - Bool消息支持
4. **错误处理测试** - 无效激光ID处理
5. **快速切换测试** - 快速开关切换

### 动物检测测试
1. **图像模拟** - 生成带动物标记的测试图像
2. **检测结果验证** - 验证检测消息格式
3. **激光联动测试** - 检测与激光控制的集成
4. **性能测试** - 检测频率和响应时间

## ⚠️ 注意事项

1. **安全警告**: 激光设备具有潜在危险，使用时务必注意安全
2. **GPIO权限**: 确保用户有GPIO访问权限
3. **硬件连接**: 确认GPIO18和GPIO19正确连接激光设备
4. **电平匹配**: 确保激光设备与Raspberry Pi电平匹配
5. **散热考虑**: 长时间使用时注意激光设备散热

## 🔄 兼容性

- **向后兼容**: 保持对旧版Bool消息的支持
- **消息迁移**: 建议逐步迁移到新的LaserControl消息
- **参数兼容**: 支持通过ROS参数和配置文件设置

## 📝 更新日志

### v4.0.0 (2025-08-01)
- ✅ 升级为双激光头支持
- ✅ 新增LaserControl消息支持  
- ✅ 保持兼容性
- ✅ 完善测试脚本
- ✅ 优化错误处理
- ✅ 更新文档

### v3.0.0 (2025-07-30)
- 单激光头GPIO控制
- Bool消息支持
- 常亮模式支持
