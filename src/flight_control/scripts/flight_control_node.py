#!/usr/bin/env python3
"""
Flight Control Node for Autonomous Drone Mission System
Author: AI Assistant
Description: 主飞行控制节点，负责任务状态机、起飞、航点导航等
"""

import rospy
import yaml
import os
import math
from enum import Enum
from geometry_msgs.msg import PoseStamped, Point
from mavros_msgs.msg import State, ExtendedState
from mavros_msgs.srv import CommandBool, CommandBoolRequest, SetMode, SetModeRequest
from std_msgs.msg import Bool
from common_msgs.msg import MissionState, AprilTagDetection


class MissionStates(Enum):
    """任务状态枚举"""
    INIT = 0
    TAKEOFF = 1
    GOTO_MISSION = 2
    SCAN_TAG = 3
    LASER_FIRE = 4
    GOTO_LANDING = 5
    PRECISION_LAND = 6
    LAND = 7
    COMPLETE = 8
    ERROR = 9


class FlightController:
    """飞行控制器主类"""
    
    def __init__(self):
        """初始化飞行控制器"""
        rospy.init_node('flight_control_node', anonymous=True)
        rospy.loginfo("Flight Control Node Started")
        
        # 加载配置参数
        self.load_config()
        
        # 状态变量
        self.current_state = State()
        self.current_extended_state = ExtendedState()
        self.current_pose = PoseStamped()
        self.apriltag_detection = AprilTagDetection()
        
        # 任务状态
        self.mission_state = MissionStates.INIT
        self.mission_start_time = None
        self.state_start_time = None
        
        # 目标位置
        self.target_pose = PoseStamped()
        self.target_pose.header.frame_id = "map"
        
        # 航点索引
        self.current_waypoint_idx = 0
        self.waypoints = [
            self.config['flight_control']['waypoints']['mission_area'],
            self.config['flight_control']['waypoints']['landing_area']
        ]
        
        # 初始化ROS通信
        self.init_ros_communication()
        
        # 控制循环频率
        self.rate = rospy.Rate(self.config['flight_control']['control_rate'])
        
        rospy.loginfo("Flight Controller initialized successfully")
    
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
            # 使用默认配置
            self.config = self.get_default_config()
    
    def get_default_config(self):
        """获取默认配置"""
        return {
            'flight_control': {
                'takeoff': {'height': 2.0, 'timeout': 10.0},
                'waypoints': {
                    'mission_area': [5.0, 5.0, 2.0],
                    'landing_area': [10.0, 0.0, 2.0]
                },
                'flight': {
                    'max_velocity': 2.0,
                    'position_tolerance': 0.3,
                    'hover_time': 2.0
                },
                'control_rate': 20
            },
            'state_machine': {
                'timeouts': {
                    'takeoff': 15.0,
                    'goto_mission': 30.0,
                    'scan_tag': 20.0,
                    'laser_fire': 10.0,
                    'goto_landing': 30.0,
                    'precision_land': 60.0,
                    'land': 15.0
                }
            }
        }
    
    def init_ros_communication(self):
        """初始化ROS通信"""
        # 订阅者
        self.state_sub = rospy.Subscriber("/mavros/state", State, self.state_callback)
        self.extended_state_sub = rospy.Subscriber("/mavros/extended_state", 
                                                 ExtendedState, self.extended_state_callback)
        self.pose_sub = rospy.Subscriber("/mavros/local_position/pose", 
                                       PoseStamped, self.pose_callback)
        self.apriltag_sub = rospy.Subscriber("/apriltag/detection", 
                                           AprilTagDetection, self.apriltag_callback)
        
        # 发布者
        self.local_pos_pub = rospy.Publisher("/mavros/setpoint_position/local", 
                                           PoseStamped, queue_size=10)
        self.mission_state_pub = rospy.Publisher("/mission/state", 
                                               MissionState, queue_size=10)
        self.laser_fire_pub = rospy.Publisher("/laser_fire", Bool, queue_size=10)
        
        # 服务客户端
        rospy.wait_for_service("/mavros/cmd/arming")
        self.arming_client = rospy.ServiceProxy("/mavros/cmd/arming", CommandBool)
        
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
    
    def apriltag_callback(self, msg):
        """AprilTag检测回调"""
        self.apriltag_detection = msg
    
    def distance_to_target(self, target_pos):
        """计算到目标点的距离"""
        if self.current_pose.pose.position is None:
            return float('inf')
        
        dx = self.current_pose.pose.position.x - target_pos[0]
        dy = self.current_pose.pose.position.y - target_pos[1]
        dz = self.current_pose.pose.position.z - target_pos[2]
        
        return math.sqrt(dx*dx + dy*dy + dz*dz)
    
    def set_target_position(self, x, y, z):
        """设置目标位置"""
        self.target_pose.header.stamp = rospy.Time.now()
        self.target_pose.pose.position.x = x
        self.target_pose.pose.position.y = y
        self.target_pose.pose.position.z = z
        # 保持当前朝向
        if self.current_pose.pose.orientation:
            self.target_pose.pose.orientation = self.current_pose.pose.orientation
        else:
            self.target_pose.pose.orientation.w = 1.0
    
    def change_flight_mode(self, mode):
        """改变飞行模式"""
        try:
            set_mode_req = SetModeRequest()
            set_mode_req.custom_mode = mode
            
            response = self.set_mode_client.call(set_mode_req)
            if response.mode_sent:
                rospy.loginfo(f"Flight mode changed to: {mode}")
                return True
            else:
                rospy.logwarn(f"Failed to change flight mode to: {mode}")
                return False
        except Exception as e:
            rospy.logerr(f"Service call failed: {e}")
            return False
    
    def arm_vehicle(self, arm=True):
        """解锁/上锁飞行器"""
        try:
            arm_cmd = CommandBoolRequest()
            arm_cmd.value = arm
            
            response = self.arming_client.call(arm_cmd)
            if response.success:
                status = "armed" if arm else "disarmed"
                rospy.loginfo(f"Vehicle {status}")
                return True
            else:
                rospy.logwarn(f"Failed to {'arm' if arm else 'disarm'} vehicle")
                return False
        except Exception as e:
            rospy.logerr(f"Arming service call failed: {e}")
            return False
    
    def publish_mission_state(self):
        """发布任务状态"""
        msg = MissionState()
        msg.current_state = self.mission_state.value
        msg.state_description = self.mission_state.name
        msg.timestamp = rospy.Time.now()
        
        # 计算任务进度
        total_states = len(MissionStates) - 2  # 排除ERROR和COMPLETE
        if self.mission_state == MissionStates.COMPLETE:
            msg.progress = 100.0
        elif self.mission_state == MissionStates.ERROR:
            msg.progress = 0.0
        else:
            msg.progress = (self.mission_state.value / total_states) * 100.0
        
        self.mission_state_pub.publish(msg)
    
    def check_state_timeout(self):
        """检查状态超时"""
        if self.state_start_time is None:
            return False
        
        elapsed = (rospy.Time.now() - self.state_start_time).to_sec()
        timeout_key = self.mission_state.name.lower()
        
        if timeout_key in self.config['state_machine']['timeouts']:
            timeout = self.config['state_machine']['timeouts'][timeout_key]
            if elapsed > timeout:
                rospy.logwarn(f"State {self.mission_state.name} timeout ({elapsed:.1f}s > {timeout}s)")
                return True
        
        return False
    
    def transition_to_state(self, new_state):
        """状态转换"""
        rospy.loginfo(f"State transition: {self.mission_state.name} -> {new_state.name}")
        self.mission_state = new_state
        self.state_start_time = rospy.Time.now()
        
        if self.mission_start_time is None:
            self.mission_start_time = rospy.Time.now()
    
    def state_machine(self):
        """主状态机"""
        if self.mission_state == MissionStates.INIT:
            self.handle_init_state()
        elif self.mission_state == MissionStates.TAKEOFF:
            self.handle_takeoff_state()
        elif self.mission_state == MissionStates.GOTO_MISSION:
            self.handle_goto_mission_state()
        elif self.mission_state == MissionStates.SCAN_TAG:
            self.handle_scan_tag_state()
        elif self.mission_state == MissionStates.LASER_FIRE:
            self.handle_laser_fire_state()
        elif self.mission_state == MissionStates.GOTO_LANDING:
            self.handle_goto_landing_state()
        elif self.mission_state == MissionStates.PRECISION_LAND:
            self.handle_precision_land_state()
        elif self.mission_state == MissionStates.LAND:
            self.handle_land_state()
        elif self.mission_state == MissionStates.COMPLETE:
            self.handle_complete_state()
        elif self.mission_state == MissionStates.ERROR:
            self.handle_error_state()
        
        # 检查超时
        if self.check_state_timeout():
            self.transition_to_state(MissionStates.ERROR)
    
    def handle_init_state(self):
        """处理初始化状态"""
        # 等待飞控连接
        if not self.current_state.connected:
            rospy.loginfo_throttle(2, "Waiting for FCU connection...")
            return
        
        # 设置初始目标位置为当前位置
        if self.current_pose.pose.position:
            self.set_target_position(
                self.current_pose.pose.position.x,
                self.current_pose.pose.position.y,
                self.current_pose.pose.position.z
            )
            
            # 发送几个设定点以确保飞控接收
            for _ in range(100):
                self.local_pos_pub.publish(self.target_pose)
                self.rate.sleep()
                if rospy.is_shutdown():
                    return
            
            rospy.loginfo("Initialization complete, ready for takeoff")
            self.transition_to_state(MissionStates.TAKEOFF)
    
    def handle_takeoff_state(self):
        """处理起飞状态"""
        takeoff_height = self.config['flight_control']['takeoff']['height']
        
        # 设置起飞高度
        if self.current_pose.pose.position:
            self.set_target_position(
                self.current_pose.pose.position.x,
                self.current_pose.pose.position.y,
                takeoff_height
            )
        
        # 切换到OFFBOARD模式并解锁
        if self.current_state.mode != "OFFBOARD":
            self.change_flight_mode("OFFBOARD")
        
        if not self.current_state.armed:
            self.arm_vehicle(True)
        
        # 检查是否到达起飞高度
        if (self.current_pose.pose.position and 
            abs(self.current_pose.pose.position.z - takeoff_height) < 0.2):
            rospy.loginfo(f"Takeoff complete at height: {self.current_pose.pose.position.z:.2f}m")
            self.transition_to_state(MissionStates.GOTO_MISSION)
    
    def handle_goto_mission_state(self):
        """处理前往任务区状态"""
        mission_waypoint = self.waypoints[0]  # 任务区航点
        
        # 设置目标位置
        self.set_target_position(mission_waypoint[0], mission_waypoint[1], mission_waypoint[2])
        
        # 检查是否到达任务区
        distance = self.distance_to_target(mission_waypoint)
        tolerance = self.config['flight_control']['flight']['position_tolerance']
        
        if distance < tolerance:
            rospy.loginfo("Arrived at mission area")
            self.transition_to_state(MissionStates.SCAN_TAG)
    
    def handle_scan_tag_state(self):
        """处理扫描二维码状态"""
        # 在任务区悬停，等待AprilTag检测
        mission_waypoint = self.waypoints[0]
        self.set_target_position(mission_waypoint[0], mission_waypoint[1], mission_waypoint[2])
        
        # 检查是否检测到任务区的AprilTag
        if (self.apriltag_detection.detected and 
            self.apriltag_detection.tag_id == 0):  # 任务区tag_id=0
            rospy.loginfo(f"AprilTag detected! ID: {self.apriltag_detection.tag_id}")
            self.transition_to_state(MissionStates.LASER_FIRE)
    
    def handle_laser_fire_state(self):
        """处理激光发射状态"""
        # 发送激光发射命令
        laser_msg = Bool()
        laser_msg.data = True
        self.laser_fire_pub.publish(laser_msg)
        
        # 等待激光发射完成（由laser_control节点自动关闭）
        fire_duration = self.config.get('laser_control', {}).get('laser', {}).get('fire_duration', 3.0)
        elapsed = (rospy.Time.now() - self.state_start_time).to_sec()
        
        if elapsed > fire_duration + 1.0:  # 多等待1秒确保激光关闭
            rospy.loginfo("Laser firing complete")
            self.transition_to_state(MissionStates.GOTO_LANDING)
    
    def handle_goto_landing_state(self):
        """处理前往降落区状态"""
        landing_waypoint = self.waypoints[1]  # 降落区航点
        
        # 设置目标位置
        self.set_target_position(landing_waypoint[0], landing_waypoint[1], landing_waypoint[2])
        
        # 检查是否到达降落区
        distance = self.distance_to_target(landing_waypoint)
        tolerance = self.config['flight_control']['flight']['position_tolerance']
        
        if distance < tolerance:
            rospy.loginfo("Arrived at landing area")
            self.transition_to_state(MissionStates.PRECISION_LAND)
    
    def handle_precision_land_state(self):
        """处理精确降落状态"""
        # 此状态由precision_landing节点接管控制
        # 这里只需要监控降落状态
        
        # 检查是否接近地面
        if (self.current_pose.pose.position and 
            self.current_pose.pose.position.z < 0.5):
            rospy.loginfo("Approaching ground, switching to AUTO.LAND")
            self.transition_to_state(MissionStates.LAND)
    
    def handle_land_state(self):
        """处理降落状态"""
        # 切换到AUTO.LAND模式
        if self.current_state.mode != "AUTO.LAND":
            self.change_flight_mode("AUTO.LAND")
        
        # 检查是否已经着陆
        if self.current_extended_state.landed_state == ExtendedState.LANDED_STATE_ON_GROUND:
            rospy.loginfo("Vehicle landed successfully")
            self.arm_vehicle(False)  # 上锁
            self.transition_to_state(MissionStates.COMPLETE)
    
    def handle_complete_state(self):
        """处理任务完成状态"""
        rospy.loginfo_throttle(5, "Mission completed successfully!")
        # 任务完成，保持此状态
    
    def handle_error_state(self):
        """处理错误状态"""
        rospy.logerr_throttle(5, "Mission in error state!")
        # 可以在这里添加错误恢复逻辑
        # 例如：切换到AUTO.LAND模式进行紧急降落
        if self.current_state.mode != "AUTO.LAND":
            self.change_flight_mode("AUTO.LAND")
    
    def run(self):
        """主运行循环"""
        rospy.loginfo("Starting mission execution...")
        
        while not rospy.is_shutdown():
            try:
                # 执行状态机
                self.state_machine()
                
                # 发布控制指令（除非在precision_land状态）
                if self.mission_state != MissionStates.PRECISION_LAND:
                    self.local_pos_pub.publish(self.target_pose)
                
                # 发布任务状态
                self.publish_mission_state()
                
                # 控制循环频率
                self.rate.sleep()
                
            except Exception as e:
                rospy.logerr(f"Error in main loop: {e}")
                self.transition_to_state(MissionStates.ERROR)


if __name__ == "__main__":
    try:
        controller = FlightController()
        controller.run()
    except rospy.ROSInterruptException:
        rospy.loginfo("Flight control node interrupted")
    except Exception as e:
        rospy.logerr(f"Flight control node failed: {e}")
