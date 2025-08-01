#!/bin/bash

echo "🚁 无人机自主任务系统 - 安装检查"
echo "=================================="

# 颜色定义
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

# 检查函数
check_ok() {
    echo -e "${GREEN}✓${NC} $1"
}

check_warn() {
    echo -e "${YELLOW}⚠${NC} $1"
}

check_error() {
    echo -e "${RED}✗${NC} $1"
}

echo ""
echo "📋 系统基础检查"
echo "--------------"

# 检查操作系统
if grep -q "Ubuntu 20.04" /etc/os-release 2>/dev/null; then
    check_ok "Ubuntu 20.04 LTS"
else
    check_warn "当前系统不是Ubuntu 20.04 LTS"
fi

# 检查ROS安装
if [ -f "/opt/ros/noetic/setup.bash" ]; then
    check_ok "ROS Noetic 已安装"
else
    check_error "ROS Noetic 未安装"
fi

# 检查Python版本
PYTHON_VERSION=$(python3 --version 2>&1 | grep -o "3\.[0-9]*")
if [[ "$PYTHON_VERSION" > "3.7" ]]; then
    check_ok "Python $PYTHON_VERSION"
else
    check_warn "Python版本可能过低: $PYTHON_VERSION"
fi

echo ""
echo "📦 工作空间检查"
echo "--------------"

# 检查工作空间结构
WORKSPACE_DIR="/home/micoair/vision_ws"
if [ -d "$WORKSPACE_DIR" ]; then
    check_ok "工作空间目录存在"
else
    check_error "工作空间目录不存在"
    exit 1
fi

# 检查包结构
PACKAGES=("common_msgs" "flight_control" "apriltag_detector" "laser_control" "precision_landing")
for pkg in "${PACKAGES[@]}"; do
    if [ -d "$WORKSPACE_DIR/src/$pkg" ]; then
        check_ok "包 $pkg 存在"
    else
        check_error "包 $pkg 不存在"
    fi
done

# 检查编译结果
if [ -d "$WORKSPACE_DIR/devel/lib" ]; then
    check_ok "工作空间已编译"
else
    check_warn "工作空间未编译，请运行: cd ~/vision_ws && catkin_make"
fi

echo ""
echo "🐍 Python依赖检查"
echo "-----------------"

# 检查Python库
check_python_lib() {
    if python3 -c "import $1" 2>/dev/null; then
        check_ok "Python库: $1"
    else
        check_error "Python库: $1 (请运行: pip3 install $2)"
    fi
}

check_python_lib "cv2" "opencv-python"
check_python_lib "numpy" "numpy"
check_python_lib "yaml" "PyYAML"
check_python_lib "serial" "pyserial"

# 检查AprilTag库
if python3 -c "import apriltag" 2>/dev/null; then
    check_ok "Python库: apriltag"
else
    check_warn "AprilTag库未安装 (可选: pip3 install apriltag)"
fi

# 检查GPIO库 (仅树莓派)
if grep -q "Raspberry Pi" /proc/cpuinfo 2>/dev/null; then
    if python3 -c "import RPi.GPIO" 2>/dev/null; then
        check_ok "Python库: RPi.GPIO"
    else
        check_warn "RPi.GPIO库未安装 (树莓派需要: pip3 install RPi.GPIO)"
    fi
fi

echo ""
echo "🔧 ROS包检查"
echo "------------"

# 设置ROS环境
source /opt/ros/noetic/setup.bash 2>/dev/null

# 检查关键ROS包
check_ros_pkg() {
    if rospack find $1 >/dev/null 2>&1; then
        check_ok "ROS包: $1"
    else
        check_error "ROS包: $1 (请安装: sudo apt install ros-noetic-$1)"
    fi
}

check_ros_pkg "mavros"
check_ros_pkg "mavros-extras"
check_ros_pkg "cv-bridge"
check_ros_pkg "image-transport"

# 检查RealSense
if rospack find realsense2_camera >/dev/null 2>&1; then
    check_ok "ROS包: realsense2_camera"
else
    check_warn "RealSense相机包未安装 (如需要请按官方文档安装)"
fi

echo ""
echo "📁 配置文件检查"
echo "--------------"

# 检查配置文件
CONFIG_FILE="$WORKSPACE_DIR/src/flight_control/config/mission_config.yaml"
if [ -f "$CONFIG_FILE" ]; then
    check_ok "任务配置文件存在"
else
    check_error "任务配置文件不存在"
fi

# 检查启动文件
LAUNCH_FILE="$WORKSPACE_DIR/src/flight_control/launch/full_mission.launch"
if [ -f "$LAUNCH_FILE" ]; then
    check_ok "启动文件存在"
else
    check_error "启动文件不存在"
fi

echo ""
echo "🔐 权限检查"
echo "----------"

# 检查脚本执行权限
SCRIPTS_DIR="$WORKSPACE_DIR/src/*/scripts/*.py"
for script in $SCRIPTS_DIR; do
    if [ -x "$script" ]; then
        filename=$(basename "$script")
        check_ok "脚本可执行: $filename"
    else
        filename=$(basename "$script")
        check_warn "脚本无执行权限: $filename"
    fi
done

echo ""
echo "📋 总结"
echo "------"

echo "系统检查完成！"
echo ""
echo "🚀 快速启动命令:"
echo "   cd ~/vision_ws"
echo "   source devel/setup.bash"
echo "   roslaunch flight_control full_mission.launch"
echo ""
echo "🧪 系统测试命令:"
echo "   rosrun flight_control system_test.py"
echo ""
echo "📖 详细文档位置:"
echo "   - 用户手册: ~/vision_ws/README.md"
echo "   - 技术文档: ~/vision_ws/src/flight_control/README.md"
echo "   - 项目总结: ~/vision_ws/PROJECT_SUMMARY.md"
echo ""
echo "⚠️  使用前请务必:"
echo "   1. 阅读安全注意事项"
echo "   2. 在安全环境下测试"
echo "   3. 确保遥控器随时可接管"
echo ""
echo "🎉 祝您使用愉快！"
