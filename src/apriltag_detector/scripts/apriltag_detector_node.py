#!/usr/bin/env python3
"""
AprilTag Detector Node for Autonomous Drone Mission System
Author: AI Assistant
Description: 使用AprilTag库检测二维码并发布位姿信息
"""

import rospy
import cv2
import numpy as np
import yaml
import os
import apriltag
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PoseStamped, Point32
from common_msgs.msg import AprilTagDetection
import tf.transformations as tf_trans


class AprilTagDetector:
    """AprilTag检测器"""
    
    def __init__(self):
        """初始化AprilTag检测器"""
        rospy.init_node('apriltag_detector_node', anonymous=True)
        rospy.loginfo("AprilTag Detector Node Started")
        
        # 加载配置参数
        self.load_config()
        
        # 初始化CV Bridge
        self.bridge = CvBridge()
        
        # 相机参数
        self.camera_matrix = None
        self.dist_coeffs = None
        self.camera_info_received = False
        
        # AprilTag检测器
        self.detector = None
        self.init_apriltag_detector()
        
        # 最新图像
        self.latest_image = None
        self.image_lock = False
        
        # 初始化ROS通信
        self.init_ros_communication()
        
        # 调试图像保存路径
        self.debug_image_counter = 0
        
        rospy.loginfo("AprilTag Detector initialized successfully")
    
    def load_config(self):
        """加载配置文件"""
        try:
            config_path = rospy.get_param('~config_file', 
                                        os.path.join(os.path.dirname(__file__), 
                                                   '../config/mission_config.yaml'))
            with open(config_path, 'r') as file:
                self.config = yaml.safe_load(file)
            rospy.loginfo(f"Config loaded from: {config_path}")
        except Exception as e:
            rospy.logerr(f"Failed to load config: {e}")
            self.config = self.get_default_config()
    
    def get_default_config(self):
        """获取默认配置"""
        return {
            'apriltag_detector': {
                'detection': {
                    'family': 'tag36h11',
                    'max_hamming_error': 0,
                    'quad_decimate': 2.0,
                    'quad_sigma': 0.0,
                    'refine_edges': 1,
                    'decode_sharpening': 0.25
                },
                'camera': {
                    'fx': 615.0, 'fy': 615.0,
                    'cx': 320.0, 'cy': 240.0,
                    'k1': 0.0, 'k2': 0.0, 'p1': 0.0, 'p2': 0.0
                },
                'tags': {
                    'mission_tag_id': 0,
                    'landing_tag_id': 1,
                    'tag_size': 0.1
                },
                'publish': {
                    'debug_image': True,
                    'pose_topic': '/apriltag/pose',
                    'debug_image_topic': '/apriltag/debug_image'
                }
            },
            'debug': {
                'save_images': True,
                'image_save_path': '/home/micoair/Desktop/mission_images'
            }
        }
    
    def init_apriltag_detector(self):
        """初始化AprilTag检测器"""
        try:
            detection_config = self.config['apriltag_detector']['detection']
            
            # 创建AprilTag检测器选项
            options = apriltag.DetectorOptions(
                families=detection_config['family'],
                border=1,
                nthreads=4,
                quad_decimate=detection_config['quad_decimate'],
                quad_blur=detection_config['quad_sigma'],
                refine_edges=detection_config['refine_edges'],
                refine_decode=detection_config['decode_sharpening'],
                refine_pose=True
            )
            
            # 创建检测器
            self.detector = apriltag.Detector(options)
            rospy.loginfo(f"AprilTag detector initialized with family: {detection_config['family']}")
            
        except Exception as e:
            rospy.logerr(f"Failed to initialize AprilTag detector: {e}")
            # 使用默认参数
            self.detector = apriltag.Detector()
    
    def init_ros_communication(self):
        """初始化ROS通信"""
        # 订阅者
        self.image_sub = rospy.Subscriber("/camera/infra1/image_rect_raw", 
                                        Image, self.image_callback, queue_size=1)
        self.camera_info_sub = rospy.Subscriber("/camera/infra1/camera_info", 
                                              CameraInfo, self.camera_info_callback)
        
        # 发布者
        self.detection_pub = rospy.Publisher("/apriltag/detection", 
                                           AprilTagDetection, queue_size=10)
        self.pose_pub = rospy.Publisher(
            self.config['apriltag_detector']['publish']['pose_topic'], 
            PoseStamped, queue_size=10)
        
        if self.config['apriltag_detector']['publish']['debug_image']:
            self.debug_image_pub = rospy.Publisher(
                self.config['apriltag_detector']['publish']['debug_image_topic'],
                Image, queue_size=1)
        
        rospy.loginfo("ROS communication initialized")
    
    def camera_info_callback(self, msg):
        """相机信息回调"""
        if not self.camera_info_received:
            # 提取相机内参
            self.camera_matrix = np.array([
                [msg.K[0], 0, msg.K[2]],
                [0, msg.K[4], msg.K[5]],
                [0, 0, 1]
            ], dtype=np.float32)
            
            self.dist_coeffs = np.array(msg.D, dtype=np.float32)
            
            rospy.loginfo("Camera parameters received")
            rospy.loginfo(f"Camera matrix:\n{self.camera_matrix}")
            rospy.loginfo(f"Distortion coeffs: {self.dist_coeffs}")
            
            self.camera_info_received = True
            
            # 如果没有从话题获取到有效参数，使用配置文件中的参数
            if np.allclose(self.camera_matrix, 0) or len(self.dist_coeffs) == 0:
                self.load_camera_params_from_config()
    
    def load_camera_params_from_config(self):
        """从配置文件加载相机参数"""
        cam_config = self.config['apriltag_detector']['camera']
        
        self.camera_matrix = np.array([
            [cam_config['fx'], 0, cam_config['cx']],
            [0, cam_config['fy'], cam_config['cy']],
            [0, 0, 1]
        ], dtype=np.float32)
        
        self.dist_coeffs = np.array([
            cam_config['k1'], cam_config['k2'], 
            cam_config['p1'], cam_config['p2']
        ], dtype=np.float32)
        
        rospy.loginfo("Using camera parameters from config file")
        rospy.loginfo(f"Camera matrix:\n{self.camera_matrix}")
    
    def image_callback(self, msg):
        """图像回调函数"""
        if self.image_lock:
            return
        
        try:
            # 转换ROS图像到OpenCV格式
            if msg.encoding == "mono8":
                cv_image = self.bridge.imgmsg_to_cv2(msg, "mono8")
            else:
                cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
                # 转换为灰度图像（AprilTag检测需要）
                cv_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
            
            self.latest_image = cv_image
            
            # 进行AprilTag检测
            self.detect_apriltags(cv_image, msg.header)
            
        except CvBridgeError as e:
            rospy.logerr(f"CV Bridge error: {e}")
        except Exception as e:
            rospy.logerr(f"Error in image callback: {e}")
    
    def detect_apriltags(self, image, header):
        """检测AprilTag"""
        if self.detector is None:
            return
        
        try:
            # 进行检测
            detections = self.detector.detect(image)
            
            # 创建检测结果消息
            detection_msg = AprilTagDetection()
            detection_msg.detected = len(detections) > 0
            detection_msg.tag_count = len(detections)
            
            if len(detections) > 0:
                # 处理第一个检测到的tag（如果有多个）
                detection = detections[0]
                detection_msg.tag_id = detection.tag_id
                detection_msg.confidence = detection.decision_margin
                
                # 计算tag的中心点
                center_x = np.mean(detection.corners[:, 0])
                center_y = np.mean(detection.corners[:, 1])
                detection_msg.center.x = center_x
                detection_msg.center.y = center_y
                detection_msg.center.z = 0.0
                
                # 保存角点信息
                for corner in detection.corners:
                    point = Point32()
                    point.x = corner[0]
                    point.y = corner[1]
                    point.z = 0.0
                    detection_msg.corners.append(point)
                
                # 计算3D位姿（如果有相机参数）
                if self.camera_matrix is not None:
                    pose_msg = self.calculate_tag_pose(detection, header)
                    if pose_msg:
                        detection_msg.pose = pose_msg
                        self.pose_pub.publish(pose_msg)
                
                rospy.loginfo(f"AprilTag detected: ID={detection.tag_id}, "
                            f"confidence={detection.decision_margin:.3f}")
            
            # 发布检测结果
            self.detection_pub.publish(detection_msg)
            
            # 发布调试图像
            if self.config['apriltag_detector']['publish']['debug_image']:
                debug_image = self.draw_debug_image(image, detections)
                self.publish_debug_image(debug_image, header)
            
            # 保存调试图像（如果启用）
            if self.config['debug']['save_images'] and len(detections) > 0:
                self.save_debug_image(image, detections)
                
        except Exception as e:
            rospy.logerr(f"Error in AprilTag detection: {e}")
    
    def calculate_tag_pose(self, detection, header):
        """计算AprilTag的3D位姿"""
        try:
            tag_size = self.config['apriltag_detector']['tags']['tag_size']
            
            # 3D目标点（tag的四个角点在tag坐标系中的坐标）
            half_size = tag_size / 2.0
            object_points = np.array([
                [-half_size, -half_size, 0],
                [ half_size, -half_size, 0],
                [ half_size,  half_size, 0],
                [-half_size,  half_size, 0]
            ], dtype=np.float32)
            
            # 图像点（检测到的角点）
            image_points = detection.corners.astype(np.float32)
            
            # 使用PnP求解位姿
            success, rvec, tvec = cv2.solvePnP(
                object_points, image_points, 
                self.camera_matrix, self.dist_coeffs
            )
            
            if not success:
                rospy.logwarn("PnP pose estimation failed")
                return None
            
            # 转换为PoseStamped消息
            pose_msg = PoseStamped()
            pose_msg.header = header
            pose_msg.header.frame_id = "camera_link"
            
            # 设置位置
            pose_msg.pose.position.x = tvec[0][0]
            pose_msg.pose.position.y = tvec[1][0]
            pose_msg.pose.position.z = tvec[2][0]
            
            # 将旋转向量转换为四元数
            rotation_matrix, _ = cv2.Rodrigues(rvec)
            quaternion = tf_trans.quaternion_from_matrix(
                np.vstack([
                    np.hstack([rotation_matrix, [[0], [0], [0]]]),
                    [0, 0, 0, 1]
                ])
            )
            
            pose_msg.pose.orientation.x = quaternion[0]
            pose_msg.pose.orientation.y = quaternion[1]
            pose_msg.pose.orientation.z = quaternion[2]
            pose_msg.pose.orientation.w = quaternion[3]
            
            return pose_msg
            
        except Exception as e:
            rospy.logerr(f"Error calculating tag pose: {e}")
            return None
    
    def draw_debug_image(self, image, detections):
        """绘制调试图像"""
        # 转换为彩色图像以便绘制彩色标记
        if len(image.shape) == 2:
            debug_image = cv2.cvtColor(image, cv2.COLOR_GRAY2BGR)
        else:
            debug_image = image.copy()
        
        for detection in detections:
            # 绘制边界框
            corners = detection.corners.astype(int)
            cv2.polylines(debug_image, [corners], True, (0, 255, 0), 2)
            
            # 绘制tag ID
            center = corners.mean(axis=0).astype(int)
            cv2.putText(debug_image, f"ID: {detection.tag_id}", 
                       (center[0] - 20, center[1] - 20),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 0, 0), 2)
            
            # 绘制置信度
            cv2.putText(debug_image, f"Conf: {detection.decision_margin:.2f}", 
                       (center[0] - 30, center[1] + 20),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 1)
            
            # 绘制中心点
            cv2.circle(debug_image, tuple(center), 5, (0, 0, 255), -1)
            
            # 绘制坐标轴（如果有位姿信息）
            if self.camera_matrix is not None:
                self.draw_coordinate_axes(debug_image, detection)
        
        return debug_image
    
    def draw_coordinate_axes(self, image, detection):
        """在图像上绘制坐标轴"""
        try:
            tag_size = self.config['apriltag_detector']['tags']['tag_size']
            
            # 3D坐标轴点
            axes_points = np.array([
                [0, 0, 0],           # 原点
                [tag_size, 0, 0],    # X轴
                [0, tag_size, 0],    # Y轴
                [0, 0, -tag_size]    # Z轴
            ], dtype=np.float32)
            
            # 计算位姿
            half_size = tag_size / 2.0
            object_points = np.array([
                [-half_size, -half_size, 0],
                [ half_size, -half_size, 0],
                [ half_size,  half_size, 0],
                [-half_size,  half_size, 0]
            ], dtype=np.float32)
            
            image_points = detection.corners.astype(np.float32)
            
            success, rvec, tvec = cv2.solvePnP(
                object_points, image_points, 
                self.camera_matrix, self.dist_coeffs
            )
            
            if success:
                # 投影坐标轴到图像
                axes_img_points, _ = cv2.projectPoints(
                    axes_points, rvec, tvec, 
                    self.camera_matrix, self.dist_coeffs
                )
                
                axes_img_points = axes_img_points.reshape(-1, 2).astype(int)
                
                # 绘制坐标轴
                origin = tuple(axes_img_points[0])
                x_axis = tuple(axes_img_points[1])
                y_axis = tuple(axes_img_points[2])
                z_axis = tuple(axes_img_points[3])
                
                cv2.arrowedLine(image, origin, x_axis, (0, 0, 255), 3)  # X轴-红色
                cv2.arrowedLine(image, origin, y_axis, (0, 255, 0), 3)  # Y轴-绿色
                cv2.arrowedLine(image, origin, z_axis, (255, 0, 0), 3)  # Z轴-蓝色
                
        except Exception as e:
            rospy.logwarn(f"Failed to draw coordinate axes: {e}")
    
    def publish_debug_image(self, debug_image, header):
        """发布调试图像"""
        try:
            debug_msg = self.bridge.cv2_to_imgmsg(debug_image, "bgr8")
            debug_msg.header = header
            self.debug_image_pub.publish(debug_msg)
        except Exception as e:
            rospy.logerr(f"Error publishing debug image: {e}")
    
    def save_debug_image(self, image, detections):
        """保存调试图像"""
        try:
            save_path = self.config['debug']['image_save_path']
            if not os.path.exists(save_path):
                os.makedirs(save_path)
            
            debug_image = self.draw_debug_image(image, detections)
            
            filename = f"apriltag_detection_{self.debug_image_counter:04d}.jpg"
            filepath = os.path.join(save_path, filename)
            
            cv2.imwrite(filepath, debug_image)
            self.debug_image_counter += 1
            
            rospy.loginfo(f"Debug image saved: {filepath}")
            
        except Exception as e:
            rospy.logerr(f"Error saving debug image: {e}")
    
    def run(self):
        """主运行循环"""
        rospy.loginfo("AprilTag Detector running...")
        rospy.spin()


if __name__ == "__main__":
    try:
        detector = AprilTagDetector()
        detector.run()
    except rospy.ROSInterruptException:
        rospy.loginfo("AprilTag detector node interrupted")
    except Exception as e:
        rospy.logerr(f"AprilTag detector node failed: {e}")
