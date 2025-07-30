#!/bin/bash
# Laser Control Quick Test Commands

echo "🔫 Laser Control v3.0 Quick Test Commands"
echo "=========================================="
echo ""

echo "1. 启动激光控制节点:"
echo "   rosrun laser_control laser_control_node.py"
echo ""

echo "2. 常亮模式测试:"
echo "   # 开启常亮"
echo "   rostopic pub /laser_continuous std_msgs/Bool \"data: true\""
echo "   # 关闭常亮"
echo "   rostopic pub /laser_continuous std_msgs/Bool \"data: false\""
echo ""

echo "3. 定时发射测试:"
echo "   # 发射激光 (定时关闭)"
echo "   rostopic pub /laser_fire std_msgs/Bool \"data: true\""
echo "   # 手动停止"
echo "   rostopic pub /laser_fire std_msgs/Bool \"data: false\""
echo ""

echo "4. 监控状态:"
echo "   rostopic echo /laser_status"
echo ""

echo "5. 运行完整测试:"
echo "   rosrun laser_control test_laser_control.py"
echo ""

echo "6. 查看日志:"
echo "   rosnode info /laser_control_node"
echo ""

echo "⚠️  注意事项:"
echo "   - 常亮模式优先级最高"
echo "   - 常亮开启时会忽略 /laser_fire 命令"
echo "   - 程序退出时自动关闭激光器"
