#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
改进的动物检测节点 - 支持YOLOv5模型和激光控制
Author: ROS Developer
Date: 2025-08-01
Version: 2.0.0
"""

import rospy
import cv2
import torch
import numpy as np
import os
from pathlib import Path

from std_msgs.msg import String, Empty, Header
from sensor_msgs.msg import Image
from geometry_msgs.msg import Point
from cv_bridge import CvBridge
from animal_detect.msg import LaserControl, AnimalDetection

class AnimalDetectNode:
    def __init__(self):
        rospy.init_node('animal_detect', anonymous=True)
        
        # 参数
        self.camera_index = rospy.get_param('~camera_index', 0)
        self.camera_frame_id = rospy.get_param('~camera_frame_id', 'camera')
        self.model_path = rospy.get_param('~model_path', self.get_default_model_path())
        self.confidence_threshold = rospy.get_param('~confidence_threshold', 0.5)
        self.detection_rate = rospy.get_param('~detection_rate', 2.0)  # Hz
        
        # OpenCV和ROS Bridge
        self.bridge = CvBridge()
        self.latest_frame = None
        self.detection_active = False
        
        # YOLOv5模型
        self.model = None
        self.device = None
        self.load_model()
        
        # 发布者
        self.detection_pub = rospy.Publisher('/animal_detection', String, queue_size=10)
        self.detection_result_pub = rospy.Publisher('/animal_detection_result', AnimalDetection, queue_size=10)
        self.image_pub = rospy.Publisher('/detection_image', Image, queue_size=1)
        self.detect_start_pub = rospy.Publisher('/detect_start', Empty, queue_size=1)
        
        # 订阅者
        rospy.Subscriber('/camera/image_raw', Image, self.image_callback)
        rospy.Subscriber('/laser_control', LaserControl, self.laser_control_callback)
        rospy.Subscriber('/start_detection', Empty, self.start_detection_callback)
        rospy.Subscriber('/stop_detection', Empty, self.stop_detection_callback)
        
        # 检测定时器
        self.detection_timer = rospy.Timer(rospy.Duration(1.0/self.detection_rate), self.detection_loop)
        
        rospy.loginfo("🔍 Animal Detection Node v2.0 Started")
        rospy.loginfo(f"📷 Camera frame ID: {self.camera_frame_id}")
        rospy.loginfo(f"🤖 Model path: {self.model_path}")
        rospy.loginfo(f"⚙️ Confidence threshold: {self.confidence_threshold}")
    
    def get_default_model_path(self):
        """获取默认模型路径"""
        pkg_path = Path(__file__).parent.parent
        model_path = pkg_path / "best.pt"
        return str(model_path)
    
    def load_model(self):
        """加载YOLOv5模型"""
        try:
            if not os.path.exists(self.model_path):
                rospy.logwarn(f"❌ Model file not found: {self.model_path}")
                return
            
            # 检测设备
            self.device = torch.device('cuda' if torch.cuda.is_available() else 'cpu')
            rospy.loginfo(f"🖥️ Using device: {self.device}")
            
            # 加载模型
            self.model = torch.hub.load('ultralytics/yolov5', 'custom', path=self.model_path)
            self.model.to(self.device)
            self.model.conf = self.confidence_threshold
            
            rospy.loginfo("✅ YOLOv5 model loaded successfully")
            
        except Exception as e:
            rospy.logwarn(f"❌ Failed to load model: {e}")
            self.model = None
    
    def image_callback(self, msg):
        """摄像头图像回调"""
        try:
            self.latest_frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except Exception as e:
            rospy.logwarn(f"Image conversion error: {e}")
    
    def laser_control_callback(self, msg):
        """激光控制回调"""
        rospy.loginfo(f"🔫 Laser {msg.laser_id}: {'ON' if msg.laser_state else 'OFF'}")
        # 这里可以根据激光状态调整检测行为
    
    def start_detection_callback(self, msg):
        """开始检测回调"""
        self.detection_active = True
        rospy.loginfo("🟢 Detection started")
        
        # 发布检测开始信号
        self.detect_start_pub.publish(Empty())
    
    def stop_detection_callback(self, msg):
        """停止检测回调"""
        self.detection_active = False
        rospy.loginfo("🔴 Detection stopped")
    
    def detection_loop(self, event):
        """检测主循环"""
        if not self.detection_active or self.latest_frame is None or self.model is None:
            return
        
        try:
            # 复制当前帧
            frame = self.latest_frame.copy()
            
            # YOLOv5推理
            results = self.model(frame)
            
            # 处理检测结果
            detections = self.process_yolo_results(results, frame)
            
            # 发布结果
            if detections:
                self.publish_detections(detections, frame)
            
            # 发布检测图像
            self.publish_detection_image(frame, results)
            
        except Exception as e:
            rospy.logwarn(f"Detection error: {e}")
    
    def process_yolo_results(self, results, frame):
        """处理YOLOv5检测结果"""
        detections = []
        
        # 解析YOLO结果
        for *box, conf, cls in results.xyxy[0].cpu().numpy():
            if conf > self.confidence_threshold:
                x1, y1, x2, y2 = map(int, box)
                
                # 计算中心点和相对位置
                center_x = (x1 + x2) / 2
                center_y = (y1 + y2) / 2
                
                # 转换为相对于无人机的位置 (简化版本)
                # 这里可以添加更复杂的坐标转换
                relative_pos = self.pixel_to_relative_position(center_x, center_y, frame.shape)
                
                detection = {
                    'class': int(cls),
                    'confidence': float(conf),
                    'bbox': [x1, y1, x2, y2],
                    'center': [center_x, center_y],
                    'relative_position': relative_pos
                }
                detections.append(detection)
        
        return detections
    
    def pixel_to_relative_position(self, pixel_x, pixel_y, frame_shape):
        """将像素坐标转换为相对位置"""
        height, width = frame_shape[:2]
        
        # 简化的坐标转换 (假设摄像头向下看)
        # 实际应用中需要考虑摄像头标定和无人机姿态
        relative_x = (pixel_x - width/2) / width * 2.0   # -1 到 1
        relative_y = (pixel_y - height/2) / height * 2.0  # -1 到 1
        relative_z = 0.0  # 假设在地面
        
        return Point(x=relative_x, y=relative_y, z=relative_z)
    
    def publish_detections(self, detections, frame):
        """发布检测结果"""
        if not detections:
            return
        
        # 统计检测结果
        animal_types_dict = {}
        positions = []
        confidences = []
        object_types = []  # 每个目标对应的动物类型
        
        for det in detections:
            # 根据模型类别映射
            class_id = det['class']
            animal_type = self.get_animal_type(class_id)
            
            # 统计每种动物的数量
            if animal_type not in animal_types_dict:
                animal_types_dict[animal_type] = 0
            animal_types_dict[animal_type] += 1
            
            # 收集每个目标的详细信息
            positions.append(det['relative_position'])
            confidences.append(det['confidence'])
            object_types.append(animal_type)
        
        # 发布简单字符串结果
        result_str = f"检测到: "
        for animal_type, count in animal_types_dict.items():
            result_str += f"{animal_type}({count}) "
        
        self.detection_pub.publish(String(data=result_str))
        
        # 发布详细结果 - 使用新的消息格式
        detection_msg = AnimalDetection()
        detection_msg.header = Header()
        detection_msg.header.stamp = rospy.Time.now()
        detection_msg.header.frame_id = self.camera_frame_id
        
        # 填充新的消息字段
        detection_msg.animal_types = list(animal_types_dict.keys())  # 动物类型数组
        detection_msg.counts = list(animal_types_dict.values())      # 对应数量数组
        detection_msg.total_count = len(detections)                  # 总数量
        detection_msg.positions = positions                          # 位置数组
        detection_msg.confidences = confidences                      # 置信度数组
        detection_msg.object_types = object_types                    # 每个目标的类型
        
        self.detection_result_pub.publish(detection_msg)
    
    def get_animal_type(self, class_id):
        """根据类别ID获取动物类型名称"""
        # 改成我的映射了
        class_names = {
            0: "tiger",
            1: "elephant",
            2: "wolf",
            3: "monkey",
            4: "peacock"
        }
        return class_names.get(class_id, f"class_{class_id}")
    
    def publish_detection_image(self, frame, results):
        """发布带检测框的图像"""
        try:
            # 绘制检测结果
            annotated_frame = results.render()[0]
            
            # 添加状态信息
            status_text = "🟢 DETECTING" if self.detection_active else "🔴 STOPPED"
            cv2.putText(annotated_frame, status_text, (10, 30), 
                       cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
            
            # 添加模型信息
            model_info = f"Model: {os.path.basename(self.model_path)}"
            cv2.putText(annotated_frame, model_info, (10, 60), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
            
            # 发布图像
            img_msg = self.bridge.cv2_to_imgmsg(annotated_frame, "bgr8")
            img_msg.header.stamp = rospy.Time.now()
            img_msg.header.frame_id = self.camera_frame_id
            
            self.image_pub.publish(img_msg)
            
        except Exception as e:
            rospy.logwarn(f"Image publishing error: {e}")
    
    def shutdown(self):
        """节点关闭处理"""
        rospy.loginfo("🔄 Shutting down Animal Detection Node")
        self.detection_active = False
        if hasattr(self, 'detection_timer'):
            self.detection_timer.shutdown()

if __name__ == '__main__':
    try:
        node = AnimalDetectNode()
        
        # 自动开始检测
        #rospy.sleep(1.0)
        #node.start_detection_callback(Empty())
        
        rospy.spin()
        
    except rospy.ROSInterruptException:
        pass
    finally:
        if 'node' in locals():
            node.shutdown()
