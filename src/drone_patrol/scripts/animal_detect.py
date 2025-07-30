#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
动物检测节点 - 使用YOLO模型检测动物并控制激光器
集成USB摄像头和AprilTag检测
"""

import rospy
import cv2
import numpy as np
import threading
import time
from std_msgs.msg import Bool, String
from sensor_msgs.msg import Image, CompressedImage
from geometry_msgs.msg import Point
from cv_bridge import CvBridge, CvBridgeError
from drone_patrol.msg import AnimalDetection

# YOLO类别 - 这里包含常见的动物类别
ANIMAL_CLASSES = {
    15: 'bird', 16: 'cat', 17: 'dog', 18: 'horse', 
    19: 'sheep', 20: 'cow', 21: 'elephant', 22: 'bear', 
    23: 'zebra', 24: 'giraffe'
}

class AnimalDetectNode:
    def __init__(self):
        rospy.init_node('animal_detect', anonymous=True)
        
        # 参数配置
        self.camera_index = rospy.get_param('~camera_index', 0)
        self.image_width = rospy.get_param('~image_width', 640)
        self.image_height = rospy.get_param('~image_height', 480)
        self.detection_rate = rospy.get_param('~detection_rate', 5.0)  # Hz
        self.confidence_threshold = rospy.get_param('~confidence_threshold', 0.5)
        self.nms_threshold = rospy.get_param('~nms_threshold', 0.4)
        
        # YOLO模型路径 (需要下载到本地)
        self.yolo_weights = rospy.get_param('~yolo_weights', '/home/micoair/yolo/yolov4.weights')
        self.yolo_config = rospy.get_param('~yolo_config', '/home/micoair/yolo/yolov4.cfg')
        self.yolo_classes = rospy.get_param('~yolo_classes', '/home/micoair/yolo/coco.names')
        
        # AprilTag检测
        self.apriltag_enabled = rospy.get_param('~apriltag_enabled', True)
        
        # 激光控制
        self.laser_enabled = rospy.get_param('~laser_enabled', True)
        self.auto_track = rospy.get_param('~auto_track', True)
        
        # OpenCV Bridge
        self.bridge = CvBridge()
        
        # 摄像头
        self.cap = None
        self.camera_connected = False
        
        # YOLO网络
        self.net = None
        self.output_layers = None
        self.classes = []
        
        # 检测状态
        self.current_detections = []
        self.tracking_target = None
        self.last_detection_time = 0
        
        # 图像处理线程
        self.processing = False
        self.latest_frame = None
        self.frame_lock = threading.Lock()
        
        # 发布者
        self.detection_pub = rospy.Publisher('/drone_patrol/animal_detection', AnimalDetection, queue_size=10)
        self.image_pub = rospy.Publisher('/drone_patrol/detection_image', Image, queue_size=1)
        self.laser_pub = rospy.Publisher('/laser_controller/target', Point, queue_size=10)
        self.laser_enable_pub = rospy.Publisher('/laser_controller/enable', Bool, queue_size=10)
        
        # 订阅者
        rospy.Subscriber('/drone_patrol/detection_enable', Bool, self.detection_enable_callback)
        rospy.Subscriber('/drone_patrol/track_target', Point, self.track_target_callback)
        
        # 初始化
        self.setup_camera()
        self.setup_yolo()
        if self.apriltag_enabled:
            self.setup_apriltag()
        
        # 启动图像采集线程
        self.image_thread = threading.Thread(target=self.image_capture_loop)
        self.image_thread.daemon = True
        self.image_thread.start()
        
        # 启动检测定时器
        rospy.Timer(rospy.Duration(1.0/self.detection_rate), self.detection_loop)
        
        rospy.loginfo("动物检测节点已启动")
    
    def setup_camera(self):
        """设置USB摄像头"""
        try:
            self.cap = cv2.VideoCapture(self.camera_index)
            if self.cap.isOpened():
                self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.image_width)
                self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.image_height)
                self.cap.set(cv2.CAP_PROP_FPS, 30)
                self.camera_connected = True
                rospy.loginfo(f"USB摄像头连接成功: /dev/video{self.camera_index}")
            else:
                rospy.logwarn(f"USB摄像头连接失败: /dev/video{self.camera_index}")
        except Exception as e:
            rospy.logwarn(f"摄像头初始化错误: {e}")
    
    def setup_yolo(self):
        """设置YOLO模型"""
        try:
            # 加载YOLO网络
            self.net = cv2.dnn.readNet(self.yolo_weights, self.yolo_config)
            
            # 获取输出层
            layer_names = self.net.getLayerNames()
            self.output_layers = [layer_names[i[0] - 1] for i in self.net.getUnconnectedOutLayers()]
            
            # 加载类别名称
            with open(self.yolo_classes, 'r') as f:
                self.classes = [line.strip() for line in f.readlines()]
            
            rospy.loginfo(f"YOLO模型加载成功: {len(self.classes)}个类别")
            
        except Exception as e:
            rospy.logwarn(f"YOLO模型加载失败: {e}")
            rospy.loginfo("将使用基础的运动检测作为备用方案")
            self.net = None
    
    def setup_apriltag(self):
        """设置AprilTag检测器"""
        try:
            import apriltag
            self.apriltag_detector = apriltag.Detector()
            rospy.loginfo("AprilTag检测器初始化成功")
        except ImportError:
            rospy.logwarn("AprilTag库未安装，跳过AprilTag检测")
            self.apriltag_enabled = False
    
    def image_capture_loop(self):
        """图像采集循环"""
        while not rospy.is_shutdown():
            if self.camera_connected and self.cap:
                ret, frame = self.cap.read()
                if ret:
                    with self.frame_lock:
                        self.latest_frame = frame.copy()
            time.sleep(0.033)  # ~30 FPS
    
    def detection_loop(self, event):
        """检测循环"""
        if not self.processing:
            self.processing = True
            
            with self.frame_lock:
                if self.latest_frame is not None:
                    frame = self.latest_frame.copy()
                else:
                    self.processing = False
                    return
            
            # 执行检测
            detections = self.detect_animals(frame)
            
            # 处理检测结果
            if detections:
                self.process_detections(detections, frame)
            
            # 发布处理后的图像
            self.publish_detection_image(frame)
            
            self.processing = False
    
    def detect_animals(self, frame):
        """检测动物"""
        detections = []
        
        if self.net is not None:
            # 使用YOLO检测
            detections.extend(self.yolo_detect(frame))
        else:
            # 使用运动检测作为备用
            detections.extend(self.motion_detect(frame))
        
        # AprilTag检测 (用于测试和校准)
        if self.apriltag_enabled:
            detections.extend(self.apriltag_detect(frame))
        
        return detections
    
    def yolo_detect(self, frame):
        """YOLO动物检测"""
        height, width, channels = frame.shape
        
        # 预处理图像
        blob = cv2.dnn.blobFromImage(frame, 0.00392, (416, 416), (0, 0, 0), True, crop=False)
        self.net.setInput(blob)
        outputs = self.net.forward(self.output_layers)
        
        # 解析检测结果
        boxes = []
        confidences = []
        class_ids = []
        
        for output in outputs:
            for detection in output:
                scores = detection[5:]
                class_id = np.argmax(scores)
                confidence = scores[class_id]
                
                # 只检测动物类别
                if confidence > self.confidence_threshold and class_id in ANIMAL_CLASSES:
                    center_x = int(detection[0] * width)
                    center_y = int(detection[1] * height)
                    w = int(detection[2] * width)
                    h = int(detection[3] * height)
                    
                    # 计算边界框
                    x = int(center_x - w / 2)
                    y = int(center_y - h / 2)
                    
                    boxes.append([x, y, w, h])
                    confidences.append(float(confidence))
                    class_ids.append(class_id)
        
        # 非最大抑制
        indexes = cv2.dnn.NMSBoxes(boxes, confidences, self.confidence_threshold, self.nms_threshold)
        
        detections = []
        if len(indexes) > 0:
            for i in indexes.flatten():
                x, y, w, h = boxes[i]
                class_id = class_ids[i]
                confidence = confidences[i]
                animal_type = ANIMAL_CLASSES[class_id]
                
                # 计算中心点
                center_x = x + w // 2
                center_y = y + h // 2
                
                detection = {
                    'type': animal_type,
                    'confidence': confidence,
                    'bbox': (x, y, w, h),
                    'center': (center_x, center_y),
                    'class_id': class_id
                }
                detections.append(detection)
                
                # 绘制检测框
                cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2)
                cv2.putText(frame, f"{animal_type} {confidence:.2f}", 
                           (x, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
        
        return detections
    
    def motion_detect(self, frame):
        """简单的运动检测 (备用方案)"""
        # 这里可以实现基于背景减除的运动检测
        # 作为YOLO不可用时的备用方案
        detections = []
        
        # 简单的示例：检测较大的运动区域
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        
        # 这里需要实现运动检测算法
        # 暂时返回空列表
        
        return detections
    
    def apriltag_detect(self, frame):
        """AprilTag检测 (用于测试)"""
        if not hasattr(self, 'apriltag_detector'):
            return []
        
        detections = []
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        tags = self.apriltag_detector.detect(gray)
        
        for tag in tags:
            # 计算tag中心
            center = tag.center.astype(int)
            
            detection = {
                'type': f'apriltag_{tag.tag_id}',
                'confidence': 1.0,
                'bbox': tuple(tag.corners.flatten().astype(int)),
                'center': tuple(center),
                'class_id': -1  # 特殊标识
            }
            detections.append(detection)
            
            # 绘制AprilTag
            cv2.polylines(frame, [tag.corners.astype(int)], True, (255, 0, 0), 2)
            cv2.putText(frame, f"Tag {tag.tag_id}", 
                       tuple(center), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 2)
        
        return detections
    
    def process_detections(self, detections, frame):
        """处理检测结果"""
        self.current_detections = detections
        self.last_detection_time = time.time()
        
        for detection in detections:
            # 发布检测消息
            self.publish_detection(detection, frame)
            
            # 自动跟踪最高置信度的目标
            if self.auto_track and self.laser_enabled:
                if self.tracking_target is None or detection['confidence'] > self.tracking_target['confidence']:
                    self.tracking_target = detection
                    self.control_laser(detection)
    
    def publish_detection(self, detection, frame):
        """发布动物检测消息"""
        msg = AnimalDetection()
        msg.header.stamp = rospy.Time.now()
        msg.animal_type = detection['type']
        msg.confidence = detection['confidence']
        msg.animal_id = detection['class_id']
        
        # 2D位置 (图像坐标)
        msg.position_2d.x = detection['center'][0]
        msg.position_2d.y = detection['center'][1]
        msg.position_2d.z = 0.0
        
        # 估算3D位置 (简化计算)
        # 这里需要相机标定参数来准确计算
        estimated_distance = self.estimate_distance(detection)
        msg.distance = estimated_distance
        
        # 3D位置 (相对相机的估算位置)
        camera_fov_h = 60.0  # 水平视场角度
        camera_fov_v = 45.0  # 垂直视场角度
        
        # 简化的3D位置计算
        angle_h = (detection['center'][0] - self.image_width/2) / self.image_width * camera_fov_h
        angle_v = (detection['center'][1] - self.image_height/2) / self.image_height * camera_fov_v
        
        msg.position_3d.x = estimated_distance * np.cos(np.radians(angle_h))
        msg.position_3d.y = estimated_distance * np.sin(np.radians(angle_h))
        msg.position_3d.z = -estimated_distance * np.sin(np.radians(angle_v))
        
        msg.tracking_active = (self.tracking_target == detection)
        
        # 转换图像
        try:
            # 裁剪检测区域
            if 'bbox' in detection and len(detection['bbox']) == 4:
                x, y, w, h = detection['bbox']
                cropped = frame[max(0, y):min(frame.shape[0], y+h), 
                               max(0, x):min(frame.shape[1], x+w)]
                msg.image = self.bridge.cv2_to_imgmsg(cropped, "bgr8")
        except Exception as e:
            rospy.logwarn(f"图像转换错误: {e}")
        
        self.detection_pub.publish(msg)
        
        rospy.loginfo(f"检测到 {detection['type']}, 置信度: {detection['confidence']:.2f}, 距离: {estimated_distance:.1f}m")
    
    def estimate_distance(self, detection):
        """估算距离"""
        # 简化的距离估算，基于目标大小
        if 'bbox' in detection and len(detection['bbox']) == 4:
            bbox_area = detection['bbox'][2] * detection['bbox'][3]
            # 假设一个标准大小，根据像素面积反推距离
            reference_area = 10000  # 参考面积
            distance = max(1.0, reference_area / bbox_area)
            return min(distance, 50.0)  # 限制最大距离
        return 10.0  # 默认距离
    
    def control_laser(self, detection):
        """控制激光器指向"""
        if not self.laser_enabled:
            return
        
        # 发送激光器目标位置
        target = Point()
        target.x = detection['position_3d'].x
        target.y = detection['position_3d'].y
        target.z = detection['position_3d'].z
        
        self.laser_pub.publish(target)
        self.laser_enable_pub.publish(Bool(data=True))
        
        rospy.loginfo(f"激光器指向目标: ({target.x:.2f}, {target.y:.2f}, {target.z:.2f})")
    
    def publish_detection_image(self, frame):
        """发布检测图像"""
        try:
            # 添加状态信息
            status_text = f"Detections: {len(self.current_detections)}"
            cv2.putText(frame, status_text, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)
            
            if self.tracking_target:
                tracking_text = f"Tracking: {self.tracking_target['type']}"
                cv2.putText(frame, tracking_text, (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)
            
            # 发布图像
            img_msg = self.bridge.cv2_to_imgmsg(frame, "bgr8")
            self.image_pub.publish(img_msg)
            
        except Exception as e:
            rospy.logwarn(f"图像发布错误: {e}")
    
    def detection_enable_callback(self, msg):
        """检测使能回调"""
        if not msg.data:
            self.current_detections = []
            self.tracking_target = None
            if self.laser_enabled:
                self.laser_enable_pub.publish(Bool(data=False))
        rospy.loginfo(f"动物检测 {'启用' if msg.data else '禁用'}")
    
    def track_target_callback(self, msg):
        """手动跟踪目标回调"""
        # 手动指定跟踪位置
        if self.laser_enabled:
            self.laser_pub.publish(msg)
            rospy.loginfo(f"手动跟踪目标: ({msg.x:.2f}, {msg.y:.2f}, {msg.z:.2f})")
    
    def shutdown(self):
        """节点关闭处理"""
        if self.cap:
            self.cap.release()
        cv2.destroyAllWindows()
        rospy.loginfo("动物检测节点已关闭")

if __name__ == '__main__':
    try:
        node = AnimalDetectNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
    finally:
        if 'node' in locals():
            node.shutdown()
