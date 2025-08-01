#!/bin/bash
# 简单无人机系统测试脚本

echo "=== 简单无人机系统测试 ==="

# 启动系统
echo "1. 启动简单无人机系统..."
gnome-terminal -- bash -c "cd /home/micoair/vision_ws && source devel/setup.bash && roslaunch simple_drone_system simple_drone_integrated.launch; exec bash"

sleep 5

echo "2. 等待系统启动..."
sleep 3

echo "3. 查看已发布的话题:"
rostopic list | grep -E "(waypoint|flight|animal|landing|link)"

echo ""
echo "4. 查看waypoints话题数据:"
rostopic echo /waypoints -n 1

echo ""
echo "5. 基本命令测试:"
echo "   - 起飞: rostopic pub /takeoff_command std_msgs/Bool \"data: true\""
echo "   - 开始巡逻: rostopic pub /ground_command std_msgs/String \"data: 'START_PATROL'\""
echo "   - 执行规划任务: rostopic pub /ground_command std_msgs/String \"data: 'START_PLANNED_MISSION'\""
echo "   - 返回起飞点: rostopic pub /ground_command std_msgs/String \"data: 'RETURN_HOME'\""
echo "   - 降落: rostopic pub /landing_command std_msgs/String \"data: 'START_LANDING'\""

echo ""
echo "6. 串口命令测试 (JSON格式):"
echo "   {\"type\": \"takeoff\"}"
echo "   {\"type\": \"patrol\"}"
echo "   {\"type\": \"goto\", \"x\": 10, \"y\": 5, \"z\": 1.2}"

echo ""
echo "=== 系统已启动，请在新终端中测试命令 ==="
