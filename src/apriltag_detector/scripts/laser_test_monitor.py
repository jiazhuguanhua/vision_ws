#!/usr/bin/env python3
"""
AprilTag激光控制测试脚本
"""

import rospy
from std_msgs.msg import Bool
from common_msgs.msg import AprilTagDetection

class LaserTestMonitor:
    def __init__(self):
        rospy.init_node('laser_test_monitor', anonymous=True)
        
        # 订阅者
        self.detection_sub = rospy.Subscriber("/apriltag/detection", AprilTagDetection, self.detection_callback)
        self.laser_status_sub = rospy.Subscriber("/laser_status", Bool, self.laser_status_callback)
        
        rospy.loginfo("🔍 Laser Test Monitor Started")
        rospy.loginfo("👀 Monitoring AprilTag detections and laser status...")
    
    def detection_callback(self, msg):
        if msg.detected:
            rospy.loginfo(f"📷 AprilTag detected: ID={msg.tag_id}, confidence={msg.confidence:.1f}")
        
    def laser_status_callback(self, msg):
        if msg.data:
            rospy.logwarn("🔥 LASER ON!")
        else:
            rospy.loginfo("💡 Laser OFF")

if __name__ == "__main__":
    try:
        monitor = LaserTestMonitor()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
