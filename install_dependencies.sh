#!/bin/bash
# 安装无人机自主任务系统所需的Python依赖

echo "正在安装Python依赖..."

# 更新pip
pip3 install --upgrade pip

# 安装AprilTag检测库
pip3 install apriltag

# 安装OpenCV (如果还没有安装)
pip3 install opencv-python opencv-contrib-python

# 安装NumPy和SciPy
pip3 install numpy scipy

# 安装PyYAML (用于配置文件解析)
pip3 install PyYAML

# 安装串口通信库
pip3 install pyserial

# 安装tf变换库
pip3 install transforms3d

# 树莓派GPIO库 (仅在树莓派上安装)
if grep -q "Raspberry Pi" /proc/cpuinfo 2>/dev/null; then
    echo "检测到树莓派，安装GPIO库..."
    pip3 install RPi.GPIO
fi

# 额外的ROS Python库
pip3 install rospkg catkin_pkg

echo "Python依赖安装完成！"

# 显示安装的库版本
echo "已安装的关键库版本:"
python3 -c "
import cv2; print(f'OpenCV: {cv2.__version__}')
import numpy; print(f'NumPy: {numpy.__version__}')
import yaml; print(f'PyYAML: {yaml.__version__}')
try:
    import apriltag; print('AprilTag: 已安装')
except ImportError:
    print('AprilTag: 未安装')
try:
    import serial; print('PySerial: 已安装')
except ImportError:
    print('PySerial: 未安装')
"
