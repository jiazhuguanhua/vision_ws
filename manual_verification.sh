#!/bin/bash

echo "=== 手动验证严格航点系统 ==="
echo ""

echo "1️⃣ 验证flight_control节点是否严格检查航点:"
echo "   启动flight_control节点，观察是否显示等待航点的消息"
echo "   命令: rosrun flight_control flight_control_node.py"
echo ""

echo "2️⃣ 测试无航点时的拒绝行为:"
echo "   在另一个终端发送以下命令（应该被拒绝）:"
echo "   rostopic pub /ground_command std_msgs/String \"data: 'START_PATROL'\""
echo "   rostopic pub /ground_command std_msgs/String \"data: 'START_PLANNED_MISSION'\""
echo ""

echo "3️⃣ 检查航点状态:"
echo "   rostopic pub /ground_command std_msgs/String \"data: 'CHECK_WAYPOINTS'\""
echo ""

echo "4️⃣ 启动waypoint_planner提供数据:"
echo "   在新终端中:"
echo "   cd /home/micoair/catkin_ws"
echo "   source devel/setup.bash"
echo "   roslaunch waypoint_planner plan.launch start_point:=\"A1B1\" no_fly_zones:=\"A3B3 A4B3\""
echo ""

echo "5️⃣ 验证有航点后的正常工作:"
echo "   等待flight_control显示收到航点的消息后，重新测试:"
echo "   rostopic pub /ground_command std_msgs/String \"data: 'CHECK_WAYPOINTS'\""
echo "   rostopic pub /ground_command std_msgs/String \"data: 'START_PLANNED_MISSION'\""
echo ""

echo "✅ 期望行为:"
echo "   - 无航点时: 显示错误消息，拒绝执行巡逻/规划任务"
echo "   - 有航点时: 显示航点信息，正常执行任务"
echo "   - RETURN_HOME命令始终可用（不需要航点数据）"
echo ""

echo "📊 关键状态消息:"
echo "   🔴 等待状态: '⚠️  等待waypoint_planner提供航点数据...'"
echo "   🟢 就绪状态: '✅ 成功接收规划航点: XX个'"
echo "   ❌ 拒绝消息: 'ERROR: 没有规划航点数据！'"
echo ""

echo "🎯 快速测试命令队列:"
echo "rosrun flight_control flight_control_node.py &"
echo "sleep 2"
echo "rostopic pub /ground_command std_msgs/String \"data: 'CHECK_WAYPOINTS'\" &"
echo "rostopic pub /ground_command std_msgs/String \"data: 'START_PATROL'\" &"
echo ""
