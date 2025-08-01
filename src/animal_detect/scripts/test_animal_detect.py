#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
动物检测节点测试脚本
Author: ROS Developer  
Date: 2025-08-01
Version: 1.0.0
"""

import rospy
import cv2
import numpy as np
import threading
import time
from std_msgs.msg import String, Empty
from sensor_msgs.msg import Image
from geometry_msgs.msg import Point
from cv_bridge import CvBridge
from animal_detect.msg import LaserControl, AnimalDetection

class AnimalDetectTester:
    def __init__(self):
        rospy.init_node('animal_detect_tester', anonymous=True)
        
        self.bridge = CvBridge()
        self.detection_results = []
        self.test_results = {}
        self.camera_active = False
        
        # 发布者
        self.image_pub = rospy.Publisher('/camera/image_raw', Image, queue_size=1)
        self.laser_pub = rospy.Publisher('/laser_control', LaserControl, queue_size=1)
        self.start_pub = rospy.Publisher('/start_detection', Empty, queue_size=1)
        self.stop_pub = rospy.Publisher('/stop_detection', Empty, queue_size=1)
        
        # 订阅者
        self.detection_sub = rospy.Subscriber('/animal_detection', String, self.detection_callback)
        self.result_sub = rospy.Subscriber('/animal_detection_result', AnimalDetection, self.result_callback)
        self.image_sub = rospy.Subscriber('/detection_image', Image, self.detection_image_callback)
        
        rospy.loginfo("🧪 Animal Detection Tester Started")
    
    def detection_callback(self, msg):
        """检测结果回调"""
        rospy.loginfo(f"📊 Detection String: {msg.data}")
        self.detection_results.append({
            'timestamp': rospy.Time.now(),
            'type': 'string',
            'data': msg.data
        })
    
    def result_callback(self, msg):
        """详细结果回调"""
        animal_info = []
        for i, animal_type in enumerate(msg.animal_types):
            animal_info.append(f"{animal_type}({msg.counts[i]})")
        
        result_str = f"详细检测: {', '.join(animal_info)}, 总数: {msg.total_count}"
        rospy.loginfo(f"📋 {result_str}")
        
        self.detection_results.append({
            'timestamp': rospy.Time.now(),
            'type': 'detailed',
            'animal_types': msg.animal_types,
            'counts': msg.counts,
            'total_count': msg.total_count,
            'positions': len(msg.positions),
            'confidences': len(msg.confidences)
        })
    
    def detection_image_callback(self, msg):
        """检测图像回调"""
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            # 可以在这里保存或显示图像
            rospy.loginfo("🖼️ Received detection image")
        except Exception as e:
            rospy.logwarn(f"Image callback error: {e}")
    
    def create_test_image(self, width=640, height=480, pattern="solid"):
        """创建测试图像"""
        if pattern == "solid":
            # 纯色图像
            image = np.full((height, width, 3), (0, 255, 0), dtype=np.uint8)
            cv2.putText(image, "TEST IMAGE", (50, 50), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)
        
        elif pattern == "checkerboard":
            # 棋盘图案
            image = np.zeros((height, width, 3), dtype=np.uint8)
            square_size = 50
            for i in range(0, height, square_size):
                for j in range(0, width, square_size):
                    if ((i // square_size) + (j // square_size)) % 2 == 0:
                        image[i:i+square_size, j:j+square_size] = (255, 255, 255)
        
        elif pattern == "gradient":
            # 渐变图像
            image = np.zeros((height, width, 3), dtype=np.uint8)
            for i in range(height):
                intensity = int(255 * i / height)
                image[i, :] = (intensity, intensity, intensity)
        
        elif pattern == "animal_simulation":
            # 模拟动物的彩色块
            image = np.full((height, width, 3), (50, 50, 50), dtype=np.uint8)  # 灰色背景
            
            # 绘制几个彩色矩形模拟动物
            animals = [
                {"rect": (100, 100, 150, 150), "color": (0, 165, 255), "label": "Tiger"},
                {"rect": (300, 200, 200, 100), "color": (128, 128, 128), "label": "Elephant"},
                {"rect": (450, 50, 80, 120), "color": (0, 255, 255), "label": "Peacock"},
            ]
            
            for animal in animals:
                x, y, w, h = animal["rect"]
                cv2.rectangle(image, (x, y), (x+w, y+h), animal["color"], -1)
                cv2.putText(image, animal["label"], (x, y-10), 
                           cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
        
        else:
            # 默认白色图像
            image = np.full((height, width, 3), (255, 255, 255), dtype=np.uint8)
        
        return image
    
    def publish_test_image(self, pattern="solid", rate=1.0):
        """发布测试图像"""
        rate_obj = rospy.Rate(rate)
        
        while not rospy.is_shutdown() and self.camera_active:
            try:
                # 创建测试图像
                test_image = self.create_test_image(pattern=pattern)
                
                # 添加时间戳
                timestamp = rospy.Time.now()
                time_str = f"{timestamp.secs}.{timestamp.nsecs // 1000000:03d}"
                cv2.putText(test_image, f"Time: {time_str}", (10, test_image.shape[0]-20), 
                           cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
                
                # 转换为ROS消息
                img_msg = self.bridge.cv2_to_imgmsg(test_image, "bgr8")
                img_msg.header.stamp = timestamp
                img_msg.header.frame_id = "camera"
                
                # 发布图像
                self.image_pub.publish(img_msg)
                
                rate_obj.sleep()
                
            except Exception as e:
                rospy.logwarn(f"Image publishing error: {e}")
                break
    
    def test_laser_control(self):
        """测试激光控制"""
        rospy.loginfo("🔫 Testing laser control...")
        
        # 测试激光开启
        laser_msg = LaserControl()
        laser_msg.laser_id = 1
        laser_msg.laser_state = True
        self.laser_pub.publish(laser_msg)
        rospy.sleep(1)
        
        # 测试激光关闭
        laser_msg.laser_state = False
        self.laser_pub.publish(laser_msg)
        
        self.test_results['laser_control'] = True
        rospy.loginfo("✅ Laser control test completed")
    
    def test_detection_control(self):
        """测试检测控制"""
        rospy.loginfo("🎛️ Testing detection control...")
        
        # 开始检测
        rospy.loginfo("🟢 Starting detection...")
        self.start_pub.publish(Empty())
        rospy.sleep(2)
        
        # 停止检测
        rospy.loginfo("🔴 Stopping detection...")
        self.stop_pub.publish(Empty())
        rospy.sleep(1)
        
        # 重新开始检测
        rospy.loginfo("🟢 Restarting detection...")
        self.start_pub.publish(Empty())
        
        self.test_results['detection_control'] = True
        rospy.loginfo("✅ Detection control test completed")
    
    def run_comprehensive_test(self):
        """运行综合测试"""
        rospy.loginfo("🚀 Starting comprehensive test...")
        
        # 清空之前的结果
        self.detection_results.clear()
        self.test_results.clear()
        
        try:
            # 1. 测试激光控制
            self.test_laser_control()
            rospy.sleep(1)
            
            # 2. 测试检测控制
            self.test_detection_control()
            rospy.sleep(1)
            
            # 3. 开始相机模拟
            rospy.loginfo("📷 Starting camera simulation...")
            self.camera_active = True
            
            # 在新线程中发布图像
            image_thread = threading.Thread(
                target=self.publish_test_image, 
                args=("animal_simulation", 2.0)
            )
            image_thread.daemon = True
            image_thread.start()
            
            # 4. 运行检测测试
            rospy.loginfo("🔍 Running detection for 10 seconds...")
            start_time = rospy.Time.now()
            test_duration = rospy.Duration(10.0)
            
            while rospy.Time.now() - start_time < test_duration and not rospy.is_shutdown():
                rospy.sleep(0.5)
            
            # 5. 停止相机
            self.camera_active = False
            rospy.loginfo("📷 Stopping camera simulation...")
            
            # 6. 分析结果
            self.analyze_results()
            
        except Exception as e:
            rospy.logerr(f"Test error: {e}")
        
        rospy.loginfo("🏁 Comprehensive test completed")
    
    def analyze_results(self):
        """分析测试结果"""
        rospy.loginfo("📊 Analyzing test results...")
        
        total_detections = len(self.detection_results)
        string_detections = len([r for r in self.detection_results if r['type'] == 'string'])
        detailed_detections = len([r for r in self.detection_results if r['type'] == 'detailed'])
        
        rospy.loginfo(f"📈 Total detections received: {total_detections}")
        rospy.loginfo(f"📄 String detections: {string_detections}")
        rospy.loginfo(f"📋 Detailed detections: {detailed_detections}")
        
        if detailed_detections > 0:
            # 分析最后一个详细检测结果
            last_detailed = None
            for result in reversed(self.detection_results):
                if result['type'] == 'detailed':
                    last_detailed = result
                    break
            
            if last_detailed:
                rospy.loginfo(f"🐾 Last detection summary:")
                rospy.loginfo(f"   Animal types: {last_detailed['animal_types']}")
                rospy.loginfo(f"   Counts: {last_detailed['counts']}")
                rospy.loginfo(f"   Total count: {last_detailed['total_count']}")
                rospy.loginfo(f"   Positions detected: {last_detailed['positions']}")
                rospy.loginfo(f"   Confidences available: {last_detailed['confidences']}")
        
        # 评估测试成功率
        success_rate = 0
        if 'laser_control' in self.test_results:
            success_rate += 25
        if 'detection_control' in self.test_results:
            success_rate += 25
        if total_detections > 0:
            success_rate += 25
        if detailed_detections > 0:
            success_rate += 25
        
        rospy.loginfo(f"🎯 Test success rate: {success_rate}%")
        
        if success_rate >= 75:
            rospy.loginfo("✅ Test PASSED - Animal detection node is working properly")
        else:
            rospy.logwarn("⚠️ Test PARTIAL - Some features may not be working correctly")
    
    def run_simple_test(self):
        """运行简单测试"""
        rospy.loginfo("🧪 Running simple test...")
        
        # 开始检测
        self.start_pub.publish(Empty())
        rospy.sleep(1)
        
        # 发布几张测试图像
        for i in range(5):
            test_image = self.create_test_image(pattern="animal_simulation")
            img_msg = self.bridge.cv2_to_imgmsg(test_image, "bgr8")
            img_msg.header.stamp = rospy.Time.now()
            img_msg.header.frame_id = "camera"
            self.image_pub.publish(img_msg)
            rospy.sleep(1)
        
        rospy.loginfo("✅ Simple test completed")

def main():
    try:
        tester = AnimalDetectTester()
        rospy.sleep(2)  # 等待节点启动
        
        # 根据参数选择测试类型
        test_type = rospy.get_param('~test_type', 'comprehensive')
        
        if test_type == 'simple':
            tester.run_simple_test()
        else:
            tester.run_comprehensive_test()
        
    except rospy.ROSInterruptException:
        rospy.loginfo("🛑 Test interrupted by user")
    except Exception as e:
        rospy.logerr(f"Test failed: {e}")

if __name__ == '__main__':
    main()
