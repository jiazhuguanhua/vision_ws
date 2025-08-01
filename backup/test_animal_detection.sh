#!/bin/bash
# -*- coding: utf-8 -*-
"""
动物检测节点测试启动脚本
使用方法: ./test_animal_detection.sh [test_type]
test_type: simple | comprehensive (默认: comprehensive)
"""

echo "🧪 动物检测节点测试脚本"
echo "=========================="

# 设置ROS环境
source /opt/ros/noetic/setup.bash
source /home/micoair/vision_ws/devel/setup.bash

# 检查参数
TEST_TYPE=${1:-comprehensive}
echo "🎯 测试类型: $TEST_TYPE"

# 检查roscore是否运行
if ! pgrep -x "roscore" > /dev/null; then
    echo "⚠️  roscore未运行，正在启动..."
    roscore &
    ROSCORE_PID=$!
    sleep 3
    echo "✅ roscore已启动 (PID: $ROSCORE_PID)"
else
    echo "✅ roscore已运行"
    ROSCORE_PID=""
fi

# 清理函数
cleanup() {
    echo ""
    echo "🧹 正在清理进程..."
    
    # 杀死所有相关的ROS节点
    pkill -f "animal_detect_node"
    pkill -f "test_animal_detect"
    
    # 如果我们启动了roscore，也要杀死它
    if [ ! -z "$ROSCORE_PID" ]; then
        kill $ROSCORE_PID 2>/dev/null
        echo "🛑 roscore已停止"
    fi
    
    echo "✅ 清理完成"
    exit 0
}

# 设置信号处理
trap cleanup SIGINT SIGTERM

echo ""
echo "🚀 启动动物检测节点..."

# 启动动物检测节点
rosrun animal_detect animal_detect_node.py &
DETECT_NODE_PID=$!
sleep 2

echo "✅ 动物检测节点已启动 (PID: $DETECT_NODE_PID)"
echo ""

echo "🧪 启动测试节点..."

# 启动测试节点
if [ "$TEST_TYPE" = "simple" ]; then
    rosrun animal_detect test_animal_detect.py _test_type:=simple
else
    rosrun animal_detect test_animal_detect.py _test_type:=comprehensive
fi

echo ""
echo "🏁 测试完成"

# 等待用户输入
echo ""
echo "按 Ctrl+C 退出或等待5秒自动退出..."
sleep 5

# 清理
cleanup
