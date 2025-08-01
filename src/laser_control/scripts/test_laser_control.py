#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
双激光头控制测试脚本
Author: ROS Developer
Date: 2025-08-01
Version: 1.0.0
"""

import rospy
import time
from std_msgs.msg import Bool
from common_msgs.msg import LaserControl

def test_dual_laser():
    """测试双激光头控制"""
    rospy.init_node('dual_laser_tester', anonymous=True)
    rospy.loginfo("🧪 Dual Laser Control Tester Started")
    
    # 创建发布者
    laser_control_pub = rospy.Publisher('/laser_control', LaserControl, queue_size=10)
    
    # 兼容性测试发布者
    laser_fire_pub = rospy.Publisher('/laser_fire', Bool, queue_size=1)
    laser_continuous_pub = rospy.Publisher('/laser_continuous', Bool, queue_size=1)
    
    rospy.sleep(2)  # 等待连接建立
    
    rospy.loginfo("🚀 Starting dual laser test sequence...")
    
    try:
        # 测试1: 使用新的LaserControl消息
        rospy.loginfo("📋 Test 1: LaserControl Messages")
        
        # 激光1开启
        rospy.loginfo("🔫 Testing Laser 1 ON")
        laser_msg = LaserControl()
        laser_msg.laser_id = 1
        laser_msg.laser_state = True
        laser_control_pub.publish(laser_msg)
        time.sleep(2)
        
        # 激光1关闭
        rospy.loginfo("� Testing Laser 1 OFF")
        laser_msg.laser_state = False
        laser_control_pub.publish(laser_msg)
        time.sleep(1)
        
        # 激光2开启
        rospy.loginfo("🔫 Testing Laser 2 ON")
        laser_msg.laser_id = 2
        laser_msg.laser_state = True
        laser_control_pub.publish(laser_msg)
        time.sleep(2)
        
        # 激光2关闭
        rospy.loginfo("💡 Testing Laser 2 OFF")
        laser_msg.laser_state = False
        laser_control_pub.publish(laser_msg)
        time.sleep(1)
        
        # 双激光同时开启
        rospy.loginfo("🔥 Testing BOTH Lasers ON")
        laser_msg.laser_id = 1
        laser_msg.laser_state = True
        laser_control_pub.publish(laser_msg)
        time.sleep(0.2)
        
        laser_msg.laser_id = 2
        laser_msg.laser_state = True
        laser_control_pub.publish(laser_msg)
        time.sleep(3)
        
        # 双激光同时关闭
        rospy.loginfo("💡 Testing BOTH Lasers OFF")
        laser_msg.laser_id = 1
        laser_msg.laser_state = False
        laser_control_pub.publish(laser_msg)
        
        laser_msg.laser_id = 2
        laser_msg.laser_state = False
        laser_control_pub.publish(laser_msg)
        time.sleep(1)
        
        # 测试2: 兼容性测试（Bool消息）
        rospy.loginfo("📋 Test 2: Legacy Bool Messages (Laser 1)")
        
        # 常亮模式测试
        rospy.loginfo("🔆 Testing continuous mode")
        continuous_msg = Bool()
        continuous_msg.data = True
        laser_continuous_pub.publish(continuous_msg)
        time.sleep(3)
        
        # 关闭常亮模式
        rospy.loginfo("🌙 Disabling continuous mode")
        continuous_msg.data = False
        laser_continuous_pub.publish(continuous_msg)
        time.sleep(1)
        
        # 定时发射测试
        rospy.loginfo("⏰ Testing timed fire")
        fire_msg = Bool()
        fire_msg.data = True
        laser_fire_pub.publish(fire_msg)
        time.sleep(5)  # 等待自动关闭
        
        # 测试3: 错误处理
        rospy.loginfo("📋 Test 3: Error Handling")
        
        # 测试无效激光ID
        rospy.loginfo("⚠️ Testing invalid laser ID")
        laser_msg.laser_id = 99  # 无效ID
        laser_msg.laser_state = True
        laser_control_pub.publish(laser_msg)
        time.sleep(1)
        
        # 测试4: 快速切换
        rospy.loginfo("📋 Test 4: Rapid Switching")
        for i in range(5):
            rospy.loginfo(f"🔄 Rapid test cycle {i+1}")
            
            # 激光1
            laser_msg.laser_id = 1
            laser_msg.laser_state = True
            laser_control_pub.publish(laser_msg)
            time.sleep(0.5)
            
            # 切换到激光2
            laser_msg.laser_id = 1
            laser_msg.laser_state = False
            laser_control_pub.publish(laser_msg)
            
            laser_msg.laser_id = 2
            laser_msg.laser_state = True
            laser_control_pub.publish(laser_msg)
            time.sleep(0.5)
            
            # 关闭激光2
            laser_msg.laser_state = False
            laser_control_pub.publish(laser_msg)
            time.sleep(0.5)
        
        rospy.loginfo("✅ All tests completed successfully!")
        
    except Exception as e:
        rospy.logerr(f"❌ Test failed: {e}")
    
    finally:
        # 确保所有激光关闭
        rospy.loginfo("🧹 Ensuring all lasers are OFF")
        laser_msg = LaserControl()
        for laser_id in [1, 2]:
            laser_msg.laser_id = laser_id
            laser_msg.laser_state = False
            laser_control_pub.publish(laser_msg)
            time.sleep(0.1)

def test_simple_sequence():
    """简单测试序列"""
    rospy.init_node('simple_laser_tester', anonymous=True)
    rospy.loginfo("🧪 Simple Laser Test Started")
    
    laser_control_pub = rospy.Publisher('/laser_control', LaserControl, queue_size=10)
    rospy.sleep(2)
    
    # 简单测试：激光1和激光2轮流开启
    for cycle in range(3):
        rospy.loginfo(f"� Test cycle {cycle + 1}/3")
        
        # 激光1
        rospy.loginfo("🔫 Laser 1 ON")
        laser_msg = LaserControl()
        laser_msg.laser_id = 1
        laser_msg.laser_state = True
        laser_control_pub.publish(laser_msg)
        time.sleep(1)
        
        laser_msg.laser_state = False
        laser_control_pub.publish(laser_msg)
        rospy.loginfo("💡 Laser 1 OFF")
        time.sleep(0.5)
        
        # 激光2
        rospy.loginfo("🔫 Laser 2 ON")
        laser_msg.laser_id = 2
        laser_msg.laser_state = True
        laser_control_pub.publish(laser_msg)
        time.sleep(1)
        
        laser_msg.laser_state = False
        laser_control_pub.publish(laser_msg)
        rospy.loginfo("💡 Laser 2 OFF")
        time.sleep(0.5)
    
    rospy.loginfo("✅ Simple test completed")

if __name__ == '__main__':
    try:
        # 根据参数选择测试类型
        test_type = rospy.get_param('~test_type', 'full')
        
        if test_type == 'simple':
            test_simple_sequence()
        else:
            test_dual_laser()
            
    except rospy.ROSInterruptException:
        rospy.loginfo("🛑 Test interrupted")
    except Exception as e:
        rospy.logerr(f"❌ Test error: {e}")
