#!/usr/bin/env python3
"""
Precision Landing Node for Autonomous Drone Mission System
Author: AI Assistant
Description: 基于AprilTag实现精确降落控制
"""

import rospy
import yaml
import os
import math
import numpy as np
from geometry_msgs.msg import PoseStamped, TwistStamped
from mavros_msgs.msg import State, ExtendedState
from mavros_msgs.srv import SetMode, SetModeRequest
from common_msgs.msg import AprilTagDetection, MissionState
import tf.transformations as tf_trans


class PIDController:
    """PID控制器"""
    
    def __init__(self, kp, ki, kd, max_output=1.0):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.max_output = max_output
        
        self.previous_error = 0.0
        self.integral = 0.0
        self.last_time = None
    
    def update(self, error, current_time):
        """更新PID控制器"""
        if self.last_time is None:
            self.last_time = current_time
            dt = 0.1  # 默认时间间隔
        else:
            dt = (current_time - self.last_time).to_sec()
            self.last_time = current_time
        
        if dt <= 0:
            dt = 0.1
        
        # 积分项
        self.integral += error * dt
        
        # 微分项
        derivative = (error - self.previous_error) / dt
        
        # PID输出
        output = (self.kp * error + 
                 self.ki * self.integral + 
                 self.kd * derivative)
        
        # 输出限幅
        output = max(-self.max_output, min(self.max_output, output))
        
        self.previous_error = error
        
        return output
    
    def reset(self):
        """重置PID控制器"""
        self.previous_error = 0.0
        self.integral = 0.0
        self.last_time = None


class PrecisionLandingController:
    """精确降落控制器"""
    
    def __init__(self):
        """初始化精确降落控制器"""
        rospy.init_node('precision_landing_node', anonymous=True)
        rospy.loginfo("Precision Landing Node Started")
        
        # 加载配置参数
        self.load_config()
        
        # 状态变量
        self.current_state = State()
        self.current_extended_state = ExtendedState()
        self.current_pose = PoseStamped()
        self.mission_state = MissionState()
        self.apriltag_detection = AprilTagDetection()
        
        # 降落控制状态
        self.landing_active = False
        self.tag_lost_time = None
        self.landing_start_time = None
        
        # PID控制器
        self.init_pid_controllers()
        
        # 目标位置
        self.target_pose = PoseStamped()
        self.target_pose.header.frame_id = "map"
        
        # 初始化ROS通信
        self.init_ros_communication()
        
        # 控制频率
        self.control_rate = rospy.Rate(20)  # 20Hz
        
        rospy.loginfo("Precision Landing Controller initialized successfully")
    
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
            'precision_landing': {
                'pid': {
                    'x_axis': {'kp': 0.5, 'ki': 0.0, 'kd': 0.1, 'max_output': 1.0},
                    'y_axis': {'kp': 0.5, 'ki': 0.0, 'kd': 0.1, 'max_output': 1.0},
                    'z_axis': {'kp': 0.3, 'ki': 0.0, 'kd': 0.05, 'max_output': 0.5}
                },
                'landing': {
                    'target_height': 0.3,
                    'descent_rate': 0.2,
                    'position_tolerance': 0.1,
                    'final_height': 0.1
                },
                'safety': {
                    'max_tilt_angle': 15.0,
                    'min_tag_area': 100,
                    'lost_tag_timeout': 3.0
                }
            },
            'apriltag_detector': {
                'tags': {
                    'landing_tag_id': 1
                }
            }
        }
    
    def init_pid_controllers(self):
        """初始化PID控制器"""
        pid_config = self.config['precision_landing']['pid']
        
        # X轴PID控制器
        x_config = pid_config['x_axis']
        self.pid_x = PIDController(
            x_config['kp'], x_config['ki'], x_config['kd'], x_config['max_output']
        )
        
        # Y轴PID控制器
        y_config = pid_config['y_axis']
        self.pid_y = PIDController(
            y_config['kp'], y_config['ki'], y_config['kd'], y_config['max_output']
        )
        
        # Z轴PID控制器
        z_config = pid_config['z_axis']
        self.pid_z = PIDController(
            z_config['kp'], z_config['ki'], z_config['kd'], z_config['max_output']
        )
        
        rospy.loginfo("PID controllers initialized")
    
    def init_ros_communication(self):
        """初始化ROS通信"""
        # 订阅者
        self.state_sub = rospy.Subscriber("/mavros/state", State, self.state_callback)
        self.extended_state_sub = rospy.Subscriber("/mavros/extended_state", 
                                                 ExtendedState, self.extended_state_callback)
        self.pose_sub = rospy.Subscriber("/mavros/local_position/pose", 
                                       PoseStamped, self.pose_callback)
        self.mission_state_sub = rospy.Subscriber("/mission/state", 
                                                MissionState, self.mission_state_callback)
        self.apriltag_sub = rospy.Subscriber("/apriltag/detection", 
                                           AprilTagDetection, self.apriltag_callback)
        
        # 发布者
        self.local_pos_pub = rospy.Publisher("/mavros/setpoint_position/local", 
                                           PoseStamped, queue_size=10)
        self.velocity_pub = rospy.Publisher("/mavros/setpoint_velocity/cmd_vel", 
                                          TwistStamped, queue_size=10)
        
        # 服务客户端
        rospy.wait_for_service("/mavros/set_mode")
        self.set_mode_client = rospy.ServiceProxy("/mavros/set_mode", SetMode)
        
        rospy.loginfo("ROS communication initialized")
    
    def state_callback(self, msg):
        """MAVROS状态回调"""
        self.current_state = msg
    
    def extended_state_callback(self, msg):
        """MAVROS扩展状态回调"""
        self.current_extended_state = msg
    
    def pose_callback(self, msg):
        """位置回调"""
        self.current_pose = msg
    
    def mission_state_callback(self, msg):
        """任务状态回调"""
        self.mission_state = msg
        
        # 检查是否进入精确降落模式
        if (msg.current_state == MissionState.PRECISION_LAND and 
            not self.landing_active):
            self.start_precision_landing()
        elif (msg.current_state != MissionState.PRECISION_LAND and 
              self.landing_active):
            self.stop_precision_landing()
    
    def apriltag_callback(self, msg):
        """AprilTag检测回调"""
        # 只处理降落区的AprilTag
        landing_tag_id = self.config['apriltag_detector']['tags']['landing_tag_id']
        
        if msg.detected and msg.tag_id == landing_tag_id:
            self.apriltag_detection = msg
            self.tag_lost_time = None  # 重置丢失时间
        else:
            # 标记tag丢失
            if self.tag_lost_time is None:
                self.tag_lost_time = rospy.Time.now()
    
    def start_precision_landing(self):
        """开始精确降落"""
        rospy.loginfo("Starting precision landing")
        self.landing_active = True
        self.landing_start_time = rospy.Time.now()
        
        # 重置PID控制器
        self.pid_x.reset()
        self.pid_y.reset()
        self.pid_z.reset()
        
        # 设置初始目标位置
        if self.current_pose.pose.position:
            self.target_pose.header.stamp = rospy.Time.now()
            self.target_pose.pose.position = self.current_pose.pose.position
            self.target_pose.pose.orientation = self.current_pose.pose.orientation
    
    def stop_precision_landing(self):
        """停止精确降落"""
        if self.landing_active:
            rospy.loginfo("Stopping precision landing")
            self.landing_active = False
    
    def calculate_tag_position_error(self):
        """计算相对于AprilTag的位置误差"""
        if not self.apriltag_detection.pose.pose.position:
            return None, None, None
        
        # 获取tag在相机坐标系下的位置
        tag_x = self.apriltag_detection.pose.pose.position.x
        tag_y = self.apriltag_detection.pose.pose.position.y
        tag_z = self.apriltag_detection.pose.pose.position.z
        
        # 相机坐标系到机体坐标系的转换（假设相机朝下安装）
        # 这里需要根据实际相机安装方式调整坐标转换
        # 假设：相机X轴 -> 机体前方(X), 相机Y轴 -> 机体右侧(-Y), 相机Z轴 -> 机体下方(Z)
        
        # 位置误差（期望tag在相机正下方中心）
        error_x = tag_x  # 前后误差
        error_y = -tag_y  # 左右误差（注意符号）
        error_z = tag_z  # 高度误差
        
        return error_x, error_y, error_z
    
    def check_safety_conditions(self):
        """检查安全条件"""
        safety_config = self.config['precision_landing']['safety']
        
        # 检查tag丢失时间
        if self.tag_lost_time is not None:
            lost_duration = (rospy.Time.now() - self.tag_lost_time).to_sec()
            if lost_duration > safety_config['lost_tag_timeout']:
                rospy.logwarn(f"AprilTag lost for {lost_duration:.1f}s, switching to AUTO.LAND")
                return False
        
        # 检查无人机姿态（防止过度倾斜）
        if self.current_pose.pose.orientation:
            # 转换四元数到欧拉角
            orientation = self.current_pose.pose.orientation
            euler = tf_trans.euler_from_quaternion([
                orientation.x, orientation.y, orientation.z, orientation.w
            ])
            
            roll, pitch, yaw = euler
            max_tilt_rad = math.radians(safety_config['max_tilt_angle'])
            
            if abs(roll) > max_tilt_rad or abs(pitch) > max_tilt_rad:
                rospy.logwarn(f"Excessive tilt: roll={math.degrees(roll):.1f}°, "
                            f"pitch={math.degrees(pitch):.1f}°")
                return False
        
        return True
    
    def precision_landing_control(self):
        """精确降落控制逻辑"""
        if not self.landing_active:
            return
        
        current_time = rospy.Time.now()
        landing_config = self.config['precision_landing']['landing']
        
        # 检查安全条件
        if not self.check_safety_conditions():
            self.switch_to_auto_land()
            return
        
        # 如果检测到AprilTag，进行精确控制
        if (self.apriltag_detection.detected and 
            self.tag_lost_time is None):
            
            # 计算位置误差
            error_x, error_y, error_z = self.calculate_tag_position_error()
            
            if error_x is not None:
                # PID控制计算
                control_x = self.pid_x.update(error_x, current_time)
                control_y = self.pid_y.update(error_y, current_time)
                
                # 高度控制（缓慢下降）
                target_height = max(
                    landing_config['final_height'],
                    self.current_pose.pose.position.z - landing_config['descent_rate'] * 0.05
                )
                
                height_error = self.current_pose.pose.position.z - target_height
                control_z = self.pid_z.update(height_error, current_time)
                
                # 设置目标位置
                self.target_pose.header.stamp = current_time
                self.target_pose.pose.position.x = self.current_pose.pose.position.x + control_x
                self.target_pose.pose.position.y = self.current_pose.pose.position.y + control_y
                self.target_pose.pose.position.z = target_height
                
                # 保持当前朝向
                self.target_pose.pose.orientation = self.current_pose.pose.orientation
                
                rospy.loginfo_throttle(1, 
                    f"Precision landing: errors=[{error_x:.3f}, {error_y:.3f}, {error_z:.3f}], "
                    f"controls=[{control_x:.3f}, {control_y:.3f}, {control_z:.3f}], "
                    f"height={self.current_pose.pose.position.z:.3f}m")
                
                # 检查是否到达最终高度
                if self.current_pose.pose.position.z < landing_config['final_height'] + 0.1:
                    rospy.loginfo("Reached final landing height, switching to AUTO.LAND")
                    self.switch_to_auto_land()
                    return
                
        else:
            # 没有检测到AprilTag，保持当前位置或缓慢下降
            rospy.logwarn_throttle(2, "No AprilTag detected, maintaining position")
            
            # 保持当前X,Y位置，缓慢下降
            if self.current_pose.pose.position:
                self.target_pose.header.stamp = current_time
                self.target_pose.pose.position.x = self.current_pose.pose.position.x
                self.target_pose.pose.position.y = self.current_pose.pose.position.y
                self.target_pose.pose.position.z = max(
                    landing_config['final_height'],
                    self.current_pose.pose.position.z - landing_config['descent_rate'] * 0.05
                )
                self.target_pose.pose.orientation = self.current_pose.pose.orientation
        
        # 发布控制指令
        self.local_pos_pub.publish(self.target_pose)
    
    def switch_to_auto_land(self):
        """切换到AUTO.LAND模式"""
        try:
            set_mode_req = SetModeRequest()
            set_mode_req.custom_mode = 'AUTO.LAND'
            
            response = self.set_mode_client.call(set_mode_req)
            if response.mode_sent:
                rospy.loginfo("Switched to AUTO.LAND mode")
                self.stop_precision_landing()
            else:
                rospy.logwarn("Failed to switch to AUTO.LAND mode")
                
        except Exception as e:
            rospy.logerr(f"Error switching to AUTO.LAND: {e}")
    
    def run(self):
        """主运行循环"""
        rospy.loginfo("Precision Landing Controller running...")
        
        while not rospy.is_shutdown():
            try:
                # 执行精确降落控制
                self.precision_landing_control()
                
                # 控制循环频率
                self.control_rate.sleep()
                
            except Exception as e:
                rospy.logerr(f"Error in precision landing control: {e}")
                self.switch_to_auto_land()


if __name__ == "__main__":
    try:
        controller = PrecisionLandingController()
        controller.run()
    except rospy.ROSInterruptException:
        rospy.loginfo("Precision landing node interrupted")
    except Exception as e:
        rospy.logerr(f"Precision landing node failed: {e}")
