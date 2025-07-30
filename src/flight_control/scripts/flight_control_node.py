#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
简单飞行控制节点 - 基础MAVROS控制
"""

import rospy
from math import sqrt
from std_msgs.msg import String, Bool
from geometry_msgs.msg import Point, PoseStamped
from mavros_msgs.msg import State, PositionTarget
from mavros_msgs.srv import CommandBool, SetMode
from waypoint_planner.msg import PointArray

class FlightControlNode:
    def __init__(self):
        rospy.init_node('flight_control', anonymous=True)
        
        # 参数
        self.takeoff_altitude = rospy.get_param('~takeoff_altitude', 1.2)
        self.position_tolerance = rospy.get_param('~position_tolerance', 0.3)
        
        # MAVROS服务
        rospy.wait_for_service('/mavros/cmd/arming')
        rospy.wait_for_service('/mavros/set_mode')
        self.arming_client = rospy.ServiceProxy('/mavros/cmd/arming', CommandBool)
        self.set_mode_client = rospy.ServiceProxy('/mavros/set_mode', SetMode)
        
        # 发布者
        self.setpoint_pub = rospy.Publisher('/mavros/setpoint_raw/local', PositionTarget, queue_size=10)
        
        # 订阅者
        rospy.Subscriber('/mavros/state', State, self.state_callback)
        rospy.Subscriber('/mavros/local_position/pose', PoseStamped, self.pose_callback)
        rospy.Subscriber('/takeoff_command', Bool, self.takeoff_callback)
        rospy.Subscriber('/waypoint_command', Point, self.waypoint_callback)
        rospy.Subscriber('/ground_command', String, self.command_callback)
        rospy.Subscriber('/waypoints', PointArray, self.waypoints_callback)
        
        # 状态变量
        self.current_state = State()
        self.current_pose = PoseStamped()
        self.target_position = Point()
        self.flight_mode = "MANUAL"
        self.mission_waypoints = []
        self.current_waypoint_index = 0
        self.planned_waypoints = []  # 从waypoint_planner接收的航点
        
        # 控制循环
        rospy.Timer(rospy.Duration(0.1), self.control_loop)
        
        rospy.loginfo("飞行控制节点启动")
        rospy.logwarn("⚠️  等待waypoint_planner提供航点数据...")
        rospy.logwarn("请确保waypoint_planner正在运行并发布/waypoints话题")
        rospy.logwarn("在接收到航点数据之前，巡逻和规划任务命令将被拒绝")
    
    def state_callback(self, msg):
        self.current_state = msg
    
    def pose_callback(self, msg):
        self.current_pose = msg
    
    def takeoff_callback(self, msg):
        if msg.data:
            self.execute_takeoff()
    
    def waypoint_callback(self, msg):
        self.target_position = msg
        self.flight_mode = "GOTO"
        rospy.loginfo(f"前往: ({msg.x:.1f}, {msg.y:.1f}, {msg.z:.1f})")
    
    def command_callback(self, msg):
        command = msg.data
        if command == "START_PATROL":
            self.start_patrol()
        elif command == "RETURN_HOME":
            self.return_home()
        elif command == "START_PLANNED_MISSION":
            self.start_planned_mission()
        elif command == "CHECK_WAYPOINTS":
            self.check_waypoints_status()
    
    def check_waypoints_status(self):
        """检查航点状态"""
        if self.planned_waypoints:
            rospy.loginfo(f"航点状态: 已接收 {len(self.planned_waypoints)} 个规划航点")
            rospy.loginfo("系统准备就绪，可以执行巡逻或规划任务")
        else:
            rospy.logwarn("航点状态: 未接收到任何规划航点")
            rospy.logwarn("请先启动waypoint_planner并等待航点规划完成")
    
    def waypoints_callback(self, msg):
        """接收waypoint_planner规划的航点"""
        self.planned_waypoints = msg.points
        rospy.loginfo("=" * 50)
        rospy.loginfo(f"✅ 成功接收规划航点: {len(self.planned_waypoints)}个")
        rospy.loginfo(f"航点规划完成，系统准备就绪")
        for i, wp in enumerate(self.planned_waypoints[:5]):  # 只显示前5个
            rospy.loginfo(f"航点{i+1}: ({wp.x:.1f}, {wp.y:.1f}, {wp.z:.1f})")
        if len(self.planned_waypoints) > 5:
            rospy.loginfo(f"... 还有{len(self.planned_waypoints)-5}个航点")
        rospy.loginfo("现在可以发送 START_PATROL 或 START_PLANNED_MISSION 命令")
        rospy.loginfo("=" * 50)
    
    def execute_takeoff(self):
        """执行起飞"""
        rospy.loginfo("开始起飞")
        
        # 切换OFFBOARD模式
        if self.set_mode_client(0, "OFFBOARD").mode_sent:
            rospy.loginfo("切换OFFBOARD模式")
        
        # 解锁
        if self.arming_client(True).success:
            rospy.loginfo("解锁成功")
        
        # 设置起飞目标
        self.target_position.x = self.current_pose.pose.position.x
        self.target_position.y = self.current_pose.pose.position.y
        self.target_position.z = self.current_pose.pose.position.z + self.takeoff_altitude
        self.flight_mode = "TAKEOFF"
    
    def start_patrol(self):
        """开始巡逻 - 严格使用规划的航点"""
        if self.planned_waypoints:
            rospy.loginfo(f"开始规划航点巡逻 - {len(self.planned_waypoints)}个航点")
            self.mission_waypoints = self.planned_waypoints[:]  # 复制规划的航点
            self.current_waypoint_index = 0
            self.flight_mode = "PATROL"
            if self.mission_waypoints:
                self.target_position = self.mission_waypoints[0]
                rospy.loginfo(f"前往第一个航点: ({self.target_position.x:.1f}, {self.target_position.y:.1f}, {self.target_position.z:.1f})")
        else:
            rospy.logwarn("ERROR: 没有规划航点数据！")
            rospy.logwarn("请先启动waypoint_planner或等待航点规划完成")
            rospy.logwarn("巡逻命令被拒绝 - 必须先有有效的航点规划")
            return  # 直接返回，不执行任何飞行动作
    
    def start_planned_mission(self):
        """开始执行规划的任务"""
        if not self.planned_waypoints:
            rospy.logerr("ERROR: 无法启动规划任务 - 没有航点数据！")
            rospy.logerr("必须先启动waypoint_planner并接收到有效的航点规划")
            rospy.logerr("请检查waypoint_planner是否正在运行并发布/waypoints话题")
            return
        
        rospy.loginfo("=" * 50)
        rospy.loginfo(f"🚀 开始执行规划任务 - {len(self.planned_waypoints)}个航点")
        self.mission_waypoints = self.planned_waypoints[:]
        self.current_waypoint_index = 0
        self.flight_mode = "PLANNED_MISSION"
        
        if self.mission_waypoints:
            self.target_position = self.mission_waypoints[0]
            rospy.loginfo(f"目标: 航点1/{len(self.mission_waypoints)}: ({self.target_position.x:.1f}, {self.target_position.y:.1f}, {self.target_position.z:.1f})")
        rospy.loginfo("=" * 50)
    
    def return_home(self):
        """返回起飞点"""
        rospy.loginfo("返回起飞点")
        self.target_position = Point(x=0, y=0, z=1.2)
        self.flight_mode = "HOME"
    
    def control_loop(self, event):
        """控制循环"""
        if not self.current_state.connected:
            return
        
        # 检查是否到达目标
        if self.is_at_target():
            if self.flight_mode in ["PATROL", "PLANNED_MISSION"] and self.mission_waypoints:
                # 下一个航点
                self.current_waypoint_index = (self.current_waypoint_index + 1) % len(self.mission_waypoints)
                self.target_position = self.mission_waypoints[self.current_waypoint_index]
                rospy.loginfo(f"前往航点 {self.current_waypoint_index + 1}/{len(self.mission_waypoints)}: ({self.target_position.x:.1f}, {self.target_position.y:.1f}, {self.target_position.z:.1f})")
                
                # 如果是规划任务且已完成一轮，可以选择停止
                if self.flight_mode == "PLANNED_MISSION" and self.current_waypoint_index == 0:
                    rospy.loginfo("规划任务完成一轮，继续循环巡逻...")
        
        # 发布设定点
        self.publish_setpoint()
    
    def is_at_target(self):
        """检查是否到达目标"""
        current_pos = self.current_pose.pose.position
        distance = sqrt(
            (current_pos.x - self.target_position.x)**2 +
            (current_pos.y - self.target_position.y)**2 +
            (current_pos.z - self.target_position.z)**2
        )
        return distance < self.position_tolerance
    
    def publish_setpoint(self):
        """发布设定点"""
        setpoint = PositionTarget()
        setpoint.header.stamp = rospy.Time.now()
        setpoint.coordinate_frame = PositionTarget.FRAME_LOCAL_NED
        
        setpoint.type_mask = (
            PositionTarget.IGNORE_VX | PositionTarget.IGNORE_VY | PositionTarget.IGNORE_VZ |
            PositionTarget.IGNORE_AFX | PositionTarget.IGNORE_AFY | PositionTarget.IGNORE_AFZ |
            PositionTarget.IGNORE_YAW_RATE
        )
        
        setpoint.position.x = self.target_position.x
        setpoint.position.y = self.target_position.y
        setpoint.position.z = self.target_position.z
        setpoint.yaw = 0.0
        
        self.setpoint_pub.publish(setpoint)

if __name__ == '__main__':
    try:
        node = FlightControlNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
