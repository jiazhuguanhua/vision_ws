# Animal Detection Package

改进的动物检测包，支持YOLOv5模型和激光控制。

## 功能特性

- ✅ 支持YOLOv5自定义模型 (best.pt)
- ✅ 实时动物检测和分类
- ✅ 多目标位置计算
- ✅ 激光控制接口
- ✅ 可视化检测结果
- ✅ 灵活的参数配置

## 节点信息

### animal_detect_node.py

**发布话题:**
- `/animal_detection` (String) - 简单检测结果字符串
- `/animal_detection_result` (AnimalDetection) - 详细检测结果
- `/detection_image` (Image) - 带检测框的图像
- `/detect_start` (Empty) - 检测开始信号

**订阅话题:**
- `/camera/image_raw` (Image) - 摄像头图像
- `/laser_control` (LaserControl) - 激光控制指令
- `/start_detection` (Empty) - 开始检测命令
- `/stop_detection` (Empty) - 停止检测命令

**参数:**
- `camera_index`: 摄像头索引 (默认: 0)
- `camera_frame_id`: 摄像头坐标系 (默认: "camera")
- `model_path`: YOLOv5模型文件路径
- `confidence_threshold`: 检测置信度阈值 (默认: 0.5)
- `detection_rate`: 检测频率 (默认: 2.0 Hz)

## 消息类型

### LaserControl.msg
```
uint8 laser_id      # 激光头ID
bool laser_state    # 激光开关状态
```

### AnimalDetection.msg
```
string animal_type         # 动物类型
uint32 count              # 检测到的数量
geometry_msgs/Point[] positions  # 相对位置
float64[] confidences     # 检测置信度
std_msgs/Header header    # 时间戳和坐标系
```

## 使用方法

### 1. 编译包
```bash
cd /path/to/vision_ws
catkin_make
source devel/setup.bash
```

### 2. 启动检测节点
```bash
# 使用USB摄像头
roslaunch animal_detect animal_detect.launch

# 使用外部摄像头源
roslaunch animal_detect animal_detect.launch use_external_camera:=true
```

### 3. 测试节点
```bash
# 运行测试脚本
rosrun animal_detect test_animal_detect.py

# 手动控制
rostopic pub /start_detection std_msgs/Empty "{}"
rostopic pub /stop_detection std_msgs/Empty "{}"
```

### 4. 查看结果
```bash
# 查看检测结果
rostopic echo /animal_detection_result

# 查看图像 (需要rqt)
rosrun rqt_image_view rqt_image_view /detection_image
```

## 模型要求

- 支持YOLOv5格式的.pt文件
- 模型应训练用于动物检测
- 模型文件放在包根目录下命名为`best.pt`

## 坐标系统

检测结果中的位置信息相对于无人机坐标系:
- X轴: 左右方向 (-1 到 1)
- Y轴: 前后方向 (-1 到 1) 
- Z轴: 高度方向 (通常为0，假设目标在地面)

## 依赖包

- rospy
- cv_bridge
- sensor_msgs
- geometry_msgs
- std_msgs
- torch (PyTorch)
- ultralytics (YOLOv5)
- opencv-python

## 安装依赖

```bash
pip3 install torch torchvision ultralytics opencv-python
```

## 故障排除

1. **模型加载失败**: 检查best.pt文件是否存在且格式正确
2. **摄像头无图像**: 检查摄像头设备和权限
3. **检测结果为空**: 调整confidence_threshold参数
4. **性能问题**: 考虑降低detection_rate或使用GPU加速

## 开发说明

如需修改检测类别或坐标转换逻辑，请编辑:
- `get_animal_type()` 函数: 类别映射
- `pixel_to_relative_position()` 函数: 坐标转换
- `config/animal_detect_config.yaml`: 参数配置
