#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
动物检测消息格式演示脚本
"""

import rospy
from geometry_msgs.msg import Point
from std_msgs.msg import Header
from animal_detect.msg import AnimalDetection

def demo_new_message_format():
    """演示新消息格式"""
    rospy.init_node('animal_detection_demo', anonymous=True)
    
    # 创建发布者
    demo_pub = rospy.Publisher('/animal_detection_result', AnimalDetection, queue_size=10)
    
    rospy.sleep(2.0)  # 等待发布者连接
    
    # 创建演示消息
    demo_msg = AnimalDetection()
    
    # 填充消息头
    demo_msg.header = Header()
    demo_msg.header.stamp = rospy.Time.now()
    demo_msg.header.frame_id = "camera"
    
    # 假设检测到: 2只老虎, 1只大象, 3只孔雀
    demo_msg.animal_types = ["tiger", "elephant", "peacock"]
    demo_msg.counts = [2, 1, 3]
    demo_msg.total_count = 6
    
    # 创建6个目标的位置信息
    positions = [
        Point(x=0.1, y=0.2, z=0.0),   # 老虎1
        Point(x=-0.3, y=0.1, z=0.0),  # 老虎2
        Point(x=0.5, y=-0.2, z=0.0),  # 大象1
        Point(x=-0.1, y=0.4, z=0.0),  # 孔雀1
        Point(x=0.2, y=-0.3, z=0.0),  # 孔雀2
        Point(x=-0.4, y=-0.1, z=0.0)  # 孔雀3
    ]
    demo_msg.positions = positions
    
    # 置信度信息
    demo_msg.confidences = [0.95, 0.87, 0.92, 0.78, 0.82, 0.89]
    
    # 每个目标对应的动物类型
    demo_msg.object_types = ["tiger", "tiger", "elephant", "peacock", "peacock", "peacock"]
    
    # 发布消息
    rospy.loginfo("🚀 发布演示检测结果...")
    demo_pub.publish(demo_msg)
    
    # 打印演示信息
    rospy.loginfo("=" * 60)
    rospy.loginfo("📝 演示消息内容:")
    rospy.loginfo(f"🦁 动物种类: {demo_msg.animal_types}")
    rospy.loginfo(f"📊 各种数量: {demo_msg.counts}")
    rospy.loginfo(f"📈 总数量: {demo_msg.total_count}")
    rospy.loginfo("📍 目标详情:")
    for i, (pos, conf, obj_type) in enumerate(zip(demo_msg.positions, demo_msg.confidences, demo_msg.object_types)):
        rospy.loginfo(f"  目标{i+1}({obj_type}): x={pos.x:.2f}, y={pos.y:.2f}, 置信度={conf:.2f}")
    rospy.loginfo("=" * 60)
    
    rospy.sleep(1.0)  # 确保消息发送完成

if __name__ == '__main__':
    try:
        demo_new_message_format()
        rospy.loginfo("✅ 演示完成!")
    except rospy.ROSInterruptException:
        rospy.loginfo("❌ 演示被中断")
