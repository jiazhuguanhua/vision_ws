#!/usr/bin/env python3
"""
Laser Control Test Script
测试激光控制功能，包括常亮模式和定时发射模式
"""

import rospy
import time
from std_msgs.msg import Bool


def test_laser_control():
    """测试激光控制功能"""
    rospy.init_node('laser_test_node', anonymous=True)
    
    # 创建发布者
    fire_pub = rospy.Publisher('/laser_fire', Bool, queue_size=10)
    continuous_pub = rospy.Publisher('/laser_continuous', Bool, queue_size=10)
    
    # 订阅状态
    def status_callback(msg):
        status = "ON" if msg.data else "OFF"
        print(f"📊 Laser Status: {status}")
    
    status_sub = rospy.Subscriber('/laser_status', Bool, status_callback)
    
    # 等待连接建立
    time.sleep(1)
    
    print("🧪 Starting Laser Control Tests...")
    print("-" * 50)
    
    # 测试1: 定时发射模式
    print("🔥 Test 1: Fire mode (3 seconds)")
    fire_msg = Bool()
    fire_msg.data = True
    fire_pub.publish(fire_msg)
    time.sleep(5)  # 等待定时关闭
    
    print("\n⏸️ Test 2: Manual stop")
    fire_msg.data = True
    fire_pub.publish(fire_msg)
    time.sleep(6)
    fire_msg.data = False
    fire_pub.publish(fire_msg)
    time.sleep(2)
    
    # 测试3: 常亮模式
    print("\n🔆 Test 3: Continuous mode ON")
    continuous_msg = Bool()
    continuous_msg.data = True
    continuous_pub.publish(continuous_msg)
    time.sleep(3)
    
    # 测试4: 常亮模式下的fire命令（应该被忽略）
    print("\n🔥 Test 4: Fire command during continuous mode (should be ignored)")
    fire_msg.data = True
    fire_pub.publish(fire_msg)
    time.sleep(2)
    
    # 测试5: 关闭常亮模式
    print("\n🌙 Test 5: Continuous mode OFF")
    continuous_msg.data = False
    continuous_pub.publish(continuous_msg)
    time.sleep(2)
    
    # 测试6: 常亮模式关闭后的fire命令
    print("\n🔥 Test 6: Fire after continuous mode disabled")
    fire_msg.data = True
    fire_pub.publish(fire_msg)
    time.sleep(5)
    
    print("\n✅ All tests completed!")


if __name__ == "__main__":
    try:
        test_laser_control()
    except rospy.ROSInterruptException:
        print("🛑 Test interrupted")
    except KeyboardInterrupt:
        print("🛑 Test stopped by user")
