#!/usr/bin/env python3
"""
Waypoint Height Verification Script
验证航点高度一致性和禁飞区边缘安全性
"""

import rospy
import time
from waypoint_planner.msg import PointArray
from geometry_msgs.msg import Point


class HeightVerifier:
    """航点高度验证器"""
    
    def __init__(self):
        rospy.init_node('height_verifier_node', anonymous=True)
        
        self.received_waypoints = False
        self.waypoints = []
        
        # 订阅航点话题
        self.waypoint_sub = rospy.Subscriber('/waypoints', PointArray, self.waypoint_callback)
        
        rospy.loginfo("🔍 Height Verifier Started - Checking altitude consistency")
    
    def waypoint_callback(self, msg):
        """航点消息回调"""
        self.received_waypoints = True
        self.waypoints = msg.points
        
        rospy.loginfo(f"📍 Received {len(self.waypoints)} waypoints")
        
        # 检查高度一致性
        self.check_altitude_consistency()
        
        # 检查路径安全性
        self.check_path_safety()
    
    def check_altitude_consistency(self):
        """检查高度一致性"""
        rospy.loginfo("🔍 Checking altitude consistency...")
        
        if not self.waypoints:
            rospy.logwarn("⚠️ No waypoints to check")
            return
        
        altitudes = [point.z for point in self.waypoints]
        unique_altitudes = set(altitudes)
        
        rospy.loginfo(f"📊 Altitude statistics:")
        rospy.loginfo(f"   Total waypoints: {len(altitudes)}")
        rospy.loginfo(f"   Unique altitudes: {len(unique_altitudes)}")
        rospy.loginfo(f"   Altitude values: {sorted(unique_altitudes)}")
        
        if len(unique_altitudes) == 1:
            rospy.loginfo(f"✅ All waypoints have consistent altitude: {list(unique_altitudes)[0]}m")
        else:
            rospy.logerr(f"❌ Found inconsistent altitudes!")
            altitude_counts = {}
            for alt in altitudes:
                altitude_counts[alt] = altitude_counts.get(alt, 0) + 1
            
            for alt, count in altitude_counts.items():
                rospy.loginfo(f"   {alt}m: {count} waypoints")
            
            # 显示有问题的航点
            expected_alt = 1.2
            for i, point in enumerate(self.waypoints):
                if abs(point.z - expected_alt) > 0.01:  # 允许小的浮点误差
                    rospy.logwarn(f"   ⚠️ Waypoint {i+1}: ({point.x:.1f}, {point.y:.1f}, {point.z:.1f})")
    
    def check_path_safety(self):
        """检查路径安全性（禁飞区边缘）"""
        rospy.loginfo("🛡️ Checking path safety around no-fly zones...")
        
        # 假设的禁飞区（应该从参数获取，这里简化处理）
        no_fly_zones = {(3, 3), (4, 3), (5, 3)}  # A3B3, A4B3, A5B3
        
        unsafe_moves = []
        
        for i in range(len(self.waypoints) - 1):
            current = self.waypoints[i]
            next_point = self.waypoints[i + 1]
            
            # 转换为网格坐标
            current_grid = (int(current.y) + 1, int(current.x) + 1)
            next_grid = (int(next_point.y) + 1, int(next_point.x) + 1)
            
            # 检查是否是对角线移动
            if (abs(current_grid[0] - next_grid[0]) == 1 and 
                abs(current_grid[1] - next_grid[1]) == 1):
                
                # 检查对角线移动的两条边
                edge1 = (current_grid[0], next_grid[1])
                edge2 = (next_grid[0], current_grid[1])
                
                if edge1 in no_fly_zones or edge2 in no_fly_zones:
                    unsafe_moves.append((i+1, current_grid, next_grid))
        
        if not unsafe_moves:
            rospy.loginfo("✅ All diagonal moves are safe from no-fly zones")
        else:
            rospy.logerr(f"❌ Found {len(unsafe_moves)} potentially unsafe diagonal moves:")
            for waypoint_num, from_pos, to_pos in unsafe_moves:
                rospy.loginfo(f"   Waypoint {waypoint_num}: A{from_pos[0]}B{from_pos[1]} -> A{to_pos[0]}B{to_pos[1]}")
    
    def run_verification(self):
        """运行验证"""
        rospy.loginfo("⏳ Waiting for waypoints...")
        
        timeout = 30  # 30秒超时
        start_time = time.time()
        
        while not self.received_waypoints and not rospy.is_shutdown():
            if time.time() - start_time > timeout:
                rospy.logerr("❌ Timeout waiting for waypoints!")
                return False
            
            time.sleep(0.1)
        
        if self.received_waypoints:
            rospy.loginfo("✅ Verification completed!")
            return True
        else:
            rospy.logerr("❌ Failed to receive waypoints!")
            return False


def run_verification():
    """运行验证"""
    try:
        verifier = HeightVerifier()
        success = verifier.run_verification()
        
        if success:
            rospy.loginfo("🎉 Height verification completed!")
        else:
            rospy.logerr("💥 Verification failed!")
            
    except rospy.ROSInterruptException:
        rospy.loginfo("🛑 Verification interrupted")
    except KeyboardInterrupt:
        rospy.loginfo("🛑 Verification stopped by user")


if __name__ == "__main__":
    run_verification()
