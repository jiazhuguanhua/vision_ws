#!/usr/bin/env python3
"""
简单激光控制测试脚本
Simple Laser Control Test Script
"""

import rospy
from std_msgs.msg import Bool
import time

def test_laser():
    """测试激光控制功能"""
    rospy.init_node('laser_test', anonymous=True)
    
    # 创建发布者
    laser_pub = rospy.Publisher('/laser_fire', Bool, queue_size=10)
    
    # 订阅激光状态
    def status_callback(msg):
        status = "ON" if msg.data else "OFF"
        rospy.loginfo(f"🔥 Laser status: {status}")
    
    status_sub = rospy.Subscriber('/laser_status', Bool, status_callback)
    
    rospy.loginfo("🧪 Laser test script started")
    rospy.loginfo("Press Ctrl+C to stop")
    
    try:
        while not rospy.is_shutdown():
            # 发射激光
            rospy.loginfo("🔥 Sending laser fire command...")
            fire_msg = Bool()
            fire_msg.data = True
            laser_pub.publish(fire_msg)
            
            # 等待5秒
            rospy.loginfo("⏳ Waiting 5 seconds...")
            time.sleep(5)
            
    except KeyboardInterrupt:
        rospy.loginfo("🛑 Test stopped by user")
    except Exception as e:
        rospy.logerr(f"❌ Test error: {e}")

if __name__ == "__main__":
    try:
        test_laser()
    except rospy.ROSInterruptException:
        pass
