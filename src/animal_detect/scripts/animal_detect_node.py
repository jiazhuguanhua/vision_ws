#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
简单动物检测节点 - 基础摄像头和检测
"""

import rospy
import cv2
import threading
import time
from std_msgs.msg import String
from sensor_msgs.msg import Image
from geometry_msgs.msg import Point
from cv_bridge import CvBridge

class AnimalDetectNode:
    def __init__(self):
        rospy.init_node('animal_detect', anonymous=True)
        
        # 参数
        self.camera_index = rospy.get_param('~camera_index', 0)
        self.detection_rate = rospy.get_param('~detection_rate', 2.0)  # Hz
        
        # OpenCV
        self.bridge = CvBridge()
        self.cap = None
        self.latest_frame = None
        self.frame_lock = threading.Lock()
        
        # 发布者
        self.detection_pub = rospy.Publisher('/animal_detection', String, queue_size=10)
        self.image_pub = rospy.Publisher('/detection_image', Image, queue_size=1)
        self.target_pub = rospy.Publisher('/laser_target', Point, queue_size=10)
        
        # 初始化摄像头
        self.setup_camera()
        
        # 启动线程
        self.start_threads()
        
        rospy.loginfo("动物检测节点启动")
    
    def setup_camera(self):
        """设置摄像头"""
        try:
            self.cap = cv2.VideoCapture(self.camera_index)
            if self.cap.isOpened():
                self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
                self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
                rospy.loginfo(f"摄像头连接成功: {self.camera_index}")
            else:
                rospy.logwarn("摄像头连接失败")
        except Exception as e:
            rospy.logwarn(f"摄像头初始化错误: {e}")
    
    def start_threads(self):
        """启动线程"""
        # 图像采集线程
        capture_thread = threading.Thread(target=self.capture_loop)
        capture_thread.daemon = True
        capture_thread.start()
        
        # 检测定时器
        rospy.Timer(rospy.Duration(1.0/self.detection_rate), self.detection_loop)
    
    def capture_loop(self):
        """图像采集循环"""
        while not rospy.is_shutdown():
            if self.cap and self.cap.isOpened():
                ret, frame = self.cap.read()
                if ret:
                    with self.frame_lock:
                        self.latest_frame = frame.copy()
            time.sleep(0.033)  # ~30 FPS
    
    def detection_loop(self, event):
        """检测循环"""
        with self.frame_lock:
            if self.latest_frame is None:
                return
            frame = self.latest_frame.copy()
        
        # 简单的运动检测 (示例)
        detections = self.simple_motion_detect(frame)
        
        if detections:
            self.process_detections(detections, frame)
        
        # 发布图像
        self.publish_image(frame)
    
    def simple_motion_detect(self, frame):
        """简单运动检测"""
        # 这里可以实现简单的检测算法
        # 或者集成YOLO等模型
        
        # 示例：检测图像中心区域的变化
        h, w = frame.shape[:2]
        center_x, center_y = w//2, h//2
        
        # 绘制中心点
        cv2.circle(frame, (center_x, center_y), 5, (0, 255, 0), -1)
        
        # 模拟检测结果
        detections = []
        # 这里应该是实际的检测逻辑
        
        return detections
    
    def process_detections(self, detections, frame):
        """处理检测结果"""
        for detection in detections:
            # 发布检测结果
            self.detection_pub.publish(String(data=f"检测到动物: {detection}"))
            
            # 发送激光目标
            target = Point()
            target.x = detection.get('x', 0)
            target.y = detection.get('y', 0)
            target.z = 0
            self.target_pub.publish(target)
    
    def publish_image(self, frame):
        """发布图像"""
        try:
            # 添加状态信息
            cv2.putText(frame, "Animal Detection", (10, 30), 
                       cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
            
            img_msg = self.bridge.cv2_to_imgmsg(frame, "bgr8")
            self.image_pub.publish(img_msg)
        except Exception as e:
            rospy.logwarn(f"图像发布错误: {e}")
    
    def shutdown(self):
        """关闭处理"""
        if self.cap:
            self.cap.release()
        cv2.destroyAllWindows()

if __name__ == '__main__':
    try:
        node = AnimalDetectNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
    finally:
        if 'node' in locals():
            node.shutdown()
