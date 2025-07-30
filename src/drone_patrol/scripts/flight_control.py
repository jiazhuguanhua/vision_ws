#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
飞行控制节点 - 集成MAVROS和T265，实现NED坐标系下的飞行控制
负责接收航点命令并执行飞行任务
"""

import rospy
import tf2_ros
import tf2_geometry_msgs
from math import sqrt, atan2, degrees
from std_msgs.msg import String, Bool
from geometry_msgs.msg import Point, PoseStamped, Twist, Vector3Stamped
from mavros_msgs.msg import State, PositionTarget
from mavros_msgs.srv import CommandBool, CommandTOL, SetMode
from sensor_msgs.msg import NavSatFix
from drone_patrol.msg import FlightStatus, LandingCommand

class FlightControlNode:
    def __init__(self):
        rospy.init_node('flight_control', anonymous=True)
        
        # 参数配置
        self.takeoff_altitude = rospy.get_param('~takeoff_altitude', 1.2)
        self.cruise_speed = rospy.get_param('~cruise_speed', 1.0)
        self.position_tolerance = rospy.get_param('~position_tolerance', 0.3)
        self.status_rate = rospy.get_param('~status_rate', 10.0)  # Hz
        
        # MAVROS服务客户端
        rospy.wait_for_service('/mavros/cmd/arming')
        rospy.wait_for_service('/mavros/set_mode')
        self.arming_client = rospy.ServiceProxy('/mavros/cmd/arming', CommandBool)
        self.set_mode_client = rospy.ServiceProxy('/mavros/set_mode', SetMode)
        self.takeoff_client = rospy.ServiceProxy('/mavros/cmd/takeoff', CommandTOL)
        self.land_client = rospy.ServiceProxy('/mavros/cmd/land', CommandTOL)
        
        # 发布者
        self.status_pub = rospy.Publisher('/drone_patrol/flight_status', FlightStatus, queue_size=10)
        self.setpoint_pub = rospy.Publisher('/mavros/setpoint_raw/local', PositionTarget, queue_size=10)
        
        # 订阅者
        rospy.Subscriber('/mavros/state', State, self.state_callback)
        rospy.Subscriber('/mavros/local_position/pose', PoseStamped, self.local_pose_callback)
        rospy.Subscriber('/mavros/local_position/velocity_local', Twist, self.velocity_callback)
        rospy.Subscriber('/mavros/global_position/global', NavSatFix, self.global_position_callback)
        rospy.Subscriber('/drone_patrol/takeoff_command', Bool, self.takeoff_callback)
        rospy.Subscriber('/drone_patrol/landing_command', LandingCommand, self.landing_callback)
        rospy.Subscriber('/drone_patrol/waypoint_command', Point, self.waypoint_callback)
        rospy.Subscriber('/drone_patrol/ground_command', String, self.ground_command_callback)
        
        # TF2监听器
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        
        # 状态变量
        self.current_state = State()
        self.current_pose = PoseStamped()
        self.current_velocity = Twist()
        self.global_position = NavSatFix()
        
        # 飞行任务变量
        self.flight_mode = "MANUAL"
        self.target_position = Point()
        self.takeoff_position = Point()
        self.mission_waypoints = []
        self.current_waypoint_index = 0
        self.mission_active = False
        self.takeoff_complete = False
        
        # NED坐标系原点 (起飞点)
        self.ned_origin_set = False
        self.ned_origin = Point()
        
        # 启动状态发布定时器
        rospy.Timer(rospy.Duration(1.0/self.status_rate), self.publish_status)
        
        # 启动控制循环
        rospy.Timer(rospy.Duration(0.1), self.control_loop)  # 10Hz控制循环
        
        rospy.loginfo("飞行控制节点已启动")
    
    def state_callback(self, msg):
        """MAVROS状态回调"""
        self.current_state = msg
        if msg.mode != self.flight_mode:
            rospy.loginfo(f"飞行模式切换: {self.flight_mode} -> {msg.mode}")
    
    def local_pose_callback(self, msg):
        """本地位置回调"""
        self.current_pose = msg
        
        # 设置NED坐标系原点（首次接收位置时）
        if not self.ned_origin_set and msg.pose.position.z != 0:
            self.ned_origin.x = msg.pose.position.x
            self.ned_origin.y = msg.pose.position.y
            self.ned_origin.z = msg.pose.position.z
            self.ned_origin_set = True
            rospy.loginfo(f"NED坐标系原点设置: ({self.ned_origin.x:.2f}, {self.ned_origin.y:.2f}, {self.ned_origin.z:.2f})")
    
    def velocity_callback(self, msg):
        """速度回调"""
        self.current_velocity = msg
    
    def global_position_callback(self, msg):
        """全局位置回调"""
        self.global_position = msg
    
    def takeoff_callback(self, msg):
        """起飞命令回调"""
        if msg.data:
            self.execute_takeoff()
    
    def landing_callback(self, msg):
        """降落命令回调"""
        if msg.command == "START_LANDING":
            self.execute_landing(msg)
        elif msg.command == "ABORT_LANDING":
            self.abort_landing()
        elif msg.command == "EMERGENCY_LAND":
            self.emergency_landing()
    
    def waypoint_callback(self, msg):
        """航点命令回调"""
        self.set_target_position(msg)
    
    def ground_command_callback(self, msg):
        """地面站命令回调"""
        command = msg.data
        
        if command == "START_PATROL":
            self.start_patrol_mission()
        elif command == "RETURN_HOME":
            self.return_to_home()
        elif command == "ABORT_MISSION":
            self.abort_mission()
        elif command == "HOVER":
            self.hover_at_current_position()
    
    def execute_takeoff(self):
        """执行起飞"""
        if not self.current_state.connected:
            rospy.logwarn("MAVROS未连接，无法起飞")
            return
        
        rospy.loginfo("开始起飞序列")
        
        # 切换到OFFBOARD模式
        if self.current_state.mode != "OFFBOARD":
            if self.set_mode_client(0, "OFFBOARD").mode_sent:
                rospy.loginfo("切换到OFFBOARD模式")
            else:
                rospy.logwarn("切换OFFBOARD模式失败")
                return
        
        # 解锁
        if not self.current_state.armed:
            if self.arming_client(True).success:
                rospy.loginfo("无人机已解锁")
            else:
                rospy.logwarn("无人机解锁失败")
                return
        
        # 设置起飞目标
        self.takeoff_position.x = self.current_pose.pose.position.x
        self.takeoff_position.y = self.current_pose.pose.position.y
        self.takeoff_position.z = self.current_pose.pose.position.z + self.takeoff_altitude
        
        self.target_position = self.takeoff_position
        self.flight_mode = "TAKEOFF"
        
        rospy.loginfo(f"起飞到高度: {self.takeoff_altitude}m")
    
    def execute_landing(self, landing_cmd):
        """执行降落"""
        rospy.loginfo(f"开始降落: {landing_cmd.landing_reason}")
        
        if landing_cmd.precision_landing:
            # 精确降落 - 45度下降
            self.precision_landing(landing_cmd)
        else:
            # 普通降落
            self.flight_mode = "LANDING"
            if self.land_client().success:
                rospy.loginfo("降落命令已发送")
            else:
                rospy.logwarn("降落命令发送失败")
    
    def precision_landing(self, landing_cmd):
        """精确降落 - 45度角下降"""
        self.flight_mode = "PRECISION_LANDING"
        
        # 计算45度下降轨迹
        current_height = self.get_relative_altitude()
        horizontal_distance = current_height / 1.0  # tan(45°) = 1
        
        # 设置降落目标点
        if landing_cmd.landing_position.x != 0 or landing_cmd.landing_position.y != 0:
            self.target_position = landing_cmd.landing_position
        else:
            # 默认在当前位置下方降落
            self.target_position.x = self.current_pose.pose.position.x
            self.target_position.y = self.current_pose.pose.position.y
            self.target_position.z = self.ned_origin.z
        
        rospy.loginfo(f"45度精确降落到: ({self.target_position.x:.2f}, {self.target_position.y:.2f})")
    
    def abort_landing(self):
        """中止降落"""
        rospy.loginfo("中止降落，保持当前高度")
        self.hover_at_current_position()
    
    def emergency_landing(self):
        """紧急降落"""
        rospy.logwarn("执行紧急降落")
        self.flight_mode = "EMERGENCY"
        if self.land_client().success:
            rospy.loginfo("紧急降落命令已发送")
    
    def set_target_position(self, position):
        """设置目标位置（NED坐标系）"""
        # 转换到本地坐标系
        self.target_position.x = self.ned_origin.x + position.x
        self.target_position.y = self.ned_origin.y + position.y
        self.target_position.z = self.ned_origin.z + position.z
        
        self.flight_mode = "GOTO_WAYPOINT"
        rospy.loginfo(f"设置目标位置: NED({position.x:.2f}, {position.y:.2f}, {position.z:.2f}) -> Local({self.target_position.x:.2f}, {self.target_position.y:.2f}, {self.target_position.z:.2f})")
    
    def start_patrol_mission(self):
        """开始巡逻任务"""
        rospy.loginfo("开始巡逻任务")
        self.mission_active = True
        self.flight_mode = "PATROL"
        # 这里应该从waypoint_planner获取航点列表
        # 暂时使用简单的矩形巡逻路径作为示例
        self.setup_demo_patrol_waypoints()
    
    def setup_demo_patrol_waypoints(self):
        """设置演示巡逻航点"""
        # 简单的矩形巡逻路径 (NED坐标)
        waypoints = [
            Point(x=10, y=0, z=1.2),
            Point(x=10, y=10, z=1.2),
            Point(x=0, y=10, z=1.2),
            Point(x=0, y=0, z=1.2)
        ]
        self.mission_waypoints = waypoints
        self.current_waypoint_index = 0
        
        if self.mission_waypoints:
            self.set_target_position(self.mission_waypoints[0])
    
    def return_to_home(self):
        """返回起飞点"""
        rospy.loginfo("返回起飞点")
        home_position = Point(x=0, y=0, z=1.2)  # NED坐标系下的起飞点
        self.set_target_position(home_position)
        self.flight_mode = "RETURN_HOME"
    
    def abort_mission(self):
        """中止任务"""
        rospy.loginfo("中止当前任务")
        self.mission_active = False
        self.hover_at_current_position()
    
    def hover_at_current_position(self):
        """在当前位置悬停"""
        self.target_position.x = self.current_pose.pose.position.x
        self.target_position.y = self.current_pose.pose.position.y
        self.target_position.z = self.current_pose.pose.position.z
        self.flight_mode = "HOVER"
    
    def control_loop(self, event):
        """主控制循环"""
        if not self.current_state.connected:
            return
        
        # 检查是否到达目标位置
        if self.is_at_target():
            self.handle_waypoint_reached()
        
        # 发送位置设定点
        self.publish_setpoint()
    
    def is_at_target(self):
        """检查是否到达目标位置"""
        if not hasattr(self, 'target_position'):
            return False
        
        current_pos = self.current_pose.pose.position
        distance = sqrt(
            (current_pos.x - self.target_position.x)**2 +
            (current_pos.y - self.target_position.y)**2 +
            (current_pos.z - self.target_position.z)**2
        )
        
        return distance < self.position_tolerance
    
    def handle_waypoint_reached(self):
        """处理到达航点"""
        if self.flight_mode == "TAKEOFF":
            self.takeoff_complete = True
            self.flight_mode = "HOVER"
            rospy.loginfo("起飞完成")
        
        elif self.flight_mode == "PATROL" and self.mission_active:
            # 巡逻模式 - 前往下一个航点
            self.current_waypoint_index += 1
            if self.current_waypoint_index >= len(self.mission_waypoints):
                self.current_waypoint_index = 0  # 循环巡逻
            
            next_waypoint = self.mission_waypoints[self.current_waypoint_index]
            self.set_target_position(next_waypoint)
            rospy.loginfo(f"前往航点 {self.current_waypoint_index + 1}")
        
        elif self.flight_mode == "RETURN_HOME":
            rospy.loginfo("已返回起飞点")
            self.flight_mode = "HOVER"
    
    def publish_setpoint(self):
        """发布位置设定点"""
        setpoint = PositionTarget()
        setpoint.header.stamp = rospy.Time.now()
        setpoint.header.frame_id = "map"
        setpoint.coordinate_frame = PositionTarget.FRAME_LOCAL_NED
        
        # 设置位置控制
        setpoint.type_mask = (
            PositionTarget.IGNORE_VX |
            PositionTarget.IGNORE_VY |
            PositionTarget.IGNORE_VZ |
            PositionTarget.IGNORE_AFX |
            PositionTarget.IGNORE_AFY |
            PositionTarget.IGNORE_AFZ |
            PositionTarget.IGNORE_YAW_RATE
        )
        
        setpoint.position.x = self.target_position.x
        setpoint.position.y = self.target_position.y
        setpoint.position.z = self.target_position.z
        setpoint.yaw = 0.0  # 保持朝向北方
        
        self.setpoint_pub.publish(setpoint)
    
    def get_relative_altitude(self):
        """获取相对起飞点的高度"""
        if self.ned_origin_set:
            return self.current_pose.pose.position.z - self.ned_origin.z
        return 0.0
    
    def publish_status(self, event):
        """发布飞行状态"""
        status = FlightStatus()
        status.header.stamp = rospy.Time.now()
        status.flight_mode = self.flight_mode
        status.current_pose = self.current_pose.pose
        status.velocity = self.current_velocity
        status.relative_altitude = self.get_relative_altitude()
        status.armed = self.current_state.armed
        status.gps_fix = self.global_position.status.status >= 0
        status.satellites = 8  # 模拟值，实际应从MAVROS获取
        status.battery_voltage = 12.6  # 模拟值，实际应从MAVROS获取
        status.mission_active = self.mission_active
        
        # 当前目标航点
        if self.mission_active and self.mission_waypoints:
            waypoint = self.mission_waypoints[self.current_waypoint_index]
            status.current_waypoint = f"WP{self.current_waypoint_index + 1}: ({waypoint.x:.1f}, {waypoint.y:.1f}, {waypoint.z:.1f})"
        else:
            status.current_waypoint = "None"
        
        self.status_pub.publish(status)

if __name__ == '__main__':
    try:
        node = FlightControlNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
