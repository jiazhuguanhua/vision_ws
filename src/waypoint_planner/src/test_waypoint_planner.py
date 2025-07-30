#!/usr/bin/env python3
"""
Waypoint Planner Test Script
测试野生动物巡查航点规划功能
"""

import rospy
import time
from waypoint_planner.msg import PointArray
from geometry_msgs.msg import Point


class WaypointTester:
    """航点规划测试器"""
    
    def __init__(self):
        rospy.init_node('waypoint_test_node', anonymous=True)
        
        self.received_waypoints = False
        self.waypoint_count = 0
        
        # 订阅航点话题
        self.waypoint_sub = rospy.Subscriber('/waypoints', PointArray, self.waypoint_callback)
        
        rospy.loginfo("🧪 Waypoint Planner Tester Started")
    
    def waypoint_callback(self, msg):
        """航点消息回调"""
        self.received_waypoints = True
        self.waypoint_count = len(msg.points)
        
        rospy.loginfo(f"📍 Received {self.waypoint_count} waypoints")
        
        # 显示前10个航点
        rospy.loginfo("🗺️ First 10 waypoints:")
        for i, point in enumerate(msg.points[:10]):
            # 转换回网格坐标显示
            row = int(point.y) + 1
            col = int(point.x) + 1
            rospy.loginfo(f"  {i+1}: A{row}B{col} -> ({point.x:.1f}, {point.y:.1f}, {point.z:.1f})")
        
        if self.waypoint_count > 10:
            rospy.loginfo(f"  ... and {self.waypoint_count - 10} more waypoints")
        
        # 验证起点和终点
        if self.waypoint_count > 0:
            start_point = msg.points[0]
            end_point = msg.points[-1]
            
            start_row, start_col = int(start_point.y) + 1, int(start_point.x) + 1
            end_row, end_col = int(end_point.y) + 1, int(end_point.x) + 1
            
            rospy.loginfo(f"🛫 Start: A{start_row}B{start_col}")
            rospy.loginfo(f"🛬 End: A{end_row}B{end_col}")
            
            if start_row == end_row and start_col == end_col:
                rospy.loginfo("✅ Path forms a closed loop!")
            else:
                rospy.logwarn("⚠️ Path does not return to start point!")
    
    def test_waypoint_planning(self):
        """测试航点规划功能"""
        rospy.loginfo("⏳ Waiting for waypoints...")
        
        timeout = 30  # 30秒超时
        start_time = time.time()
        
        while not self.received_waypoints and not rospy.is_shutdown():
            if time.time() - start_time > timeout:
                rospy.logerr("❌ Timeout waiting for waypoints!")
                return False
            
            time.sleep(0.1)
        
        if self.received_waypoints:
            rospy.loginfo("✅ Waypoint planning test completed successfully!")
            rospy.loginfo(f"📊 Total waypoints: {self.waypoint_count}")
            return True
        else:
            rospy.logerr("❌ Failed to receive waypoints!")
            return False


def run_test():
    """运行测试"""
    try:
        tester = WaypointTester()
        success = tester.test_waypoint_planning()
        
        if success:
            rospy.loginfo("🎉 All tests passed!")
        else:
            rospy.logerr("💥 Tests failed!")
            
    except rospy.ROSInterruptException:
        rospy.loginfo("🛑 Test interrupted")
    except KeyboardInterrupt:
        rospy.loginfo("🛑 Test stopped by user")


if __name__ == "__main__":
    run_test()
