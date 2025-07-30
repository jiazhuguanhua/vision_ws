#!/bin/bash
# 严格航点检查测试脚本

echo "=== 严格航点规划系统测试 ==="

echo "1. 启动flight_control节点 (无航点数据)"
echo "   应该显示等待航点数据的警告..."

# 在后台启动flight_control
cd /home/micoair/vision_ws
source devel/setup.bash
gnome-terminal -- bash -c "rosrun flight_control flight_control_node.py; exec bash" &

sleep 3

echo ""
echo "2. 测试无航点时的命令 (应该被拒绝)"
echo "   尝试启动巡逻..."
rostopic pub -1 /ground_command std_msgs/String "data: 'START_PATROL'"

sleep 1

echo "   尝试启动规划任务..."
rostopic pub -1 /ground_command std_msgs/String "data: 'START_PLANNED_MISSION'"

sleep 1

echo "   检查航点状态..."
rostopic pub -1 /ground_command std_msgs/String "data: 'CHECK_WAYPOINTS'"

echo ""
echo "3. 启动waypoint_planner提供航点数据..."
gnome-terminal -- bash -c "cd /home/micoair/vision_ws && source devel/setup.bash && roslaunch waypoint_planner plan.launch start_point:='A1B1' no_fly_zones:='A3B3 A4B3'; exec bash" &

sleep 5

echo ""
echo "4. 现在航点数据可用，重新测试命令 (应该成功)"
echo "   检查航点状态..."
rostopic pub -1 /ground_command std_msgs/String "data: 'CHECK_WAYPOINTS'"

sleep 1

echo "   尝试启动规划任务..."
rostopic pub -1 /ground_command std_msgs/String "data: 'START_PLANNED_MISSION'"

echo ""
echo "=== 测试完成 ==="
echo ""
echo "✅ 验证要点:"
echo "   1. 没有航点时，巡逻/规划任务命令被拒绝"
echo "   2. 有航点后，命令正常执行"
echo "   3. CHECK_WAYPOINTS命令显示当前状态"
echo ""
echo "📋 可用命令:"
echo "   rostopic pub /ground_command std_msgs/String \"data: 'CHECK_WAYPOINTS'\""
echo "   rostopic pub /ground_command std_msgs/String \"data: 'START_PLANNED_MISSION'\""
echo "   rostopic pub /ground_command std_msgs/String \"data: 'START_PATROL'\""
echo ""
echo "📡 串口JSON命令:"
echo "   {\"type\": \"check\"}      - 检查航点状态"
echo "   {\"type\": \"patrol\"}    - 开始巡逻 (需要航点)"
echo "   {\"type\": \"return\"}    - 返回起飞点"
