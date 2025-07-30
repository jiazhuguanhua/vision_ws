#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
简单降落模式节点 - 基础降落控制
"""

import rospy
import math
from std_msgs.msg import String, Bool
from geometry_msgs.msg import Point, PoseStamped
from mavros_msgs.msg import PositionTarget
from mavros_msgs.srv import CommandTOL

class LandingModeNode:
    def __init__(self):
        rospy.init_node('landing_mode', anonymous=True)
        
        # 参数
        self.descent_angle = rospy.get_param('~descent_angle', 45.0)  # 度
        self.landing_speed = rospy.get_param('~landing_speed', 0.5)   # m/s
        
        # 服务
        self.land_client = rospy.ServiceProxy('/mavros/cmd/land', CommandTOL)
        
        # 发布者
        self.setpoint_pub = rospy.Publisher('/mavros/setpoint_raw/local', PositionTarget, queue_size=10)
        self.status_pub = rospy.Publisher('/landing_status', String, queue_size=10)
        
        # 订阅者
        rospy.Subscriber('/mavros/local_position/pose', PoseStamped, self.pose_callback)
        rospy.Subscriber('/landing_command', String, self.landing_callback)
        
        # 状态
        self.current_pose = PoseStamped()
        self.landing_active = False
        self.landing_target = Point()
        
        # 控制循环
        rospy.Timer(rospy.Duration(0.1), self.control_loop)
        
        rospy.loginfo("降落模式节点启动")
    
    def pose_callback(self, msg):
        self.current_pose = msg
    
    def landing_callback(self, msg):
        command = msg.data
        
        if command == "START_LANDING":
            self.start_landing()
        elif command == "STOP_LANDING":
            self.stop_landing()
        elif command == "EMERGENCY_LAND":
            self.emergency_landing()
    
    def start_landing(self):
        """开始降落"""
        rospy.loginfo("开始45度降落")
        self.landing_active = True
        
        # 设置降落目标点（当前位置下方）
        self.landing_target.x = self.current_pose.pose.position.x
        self.landing_target.y = self.current_pose.pose.position.y
        self.landing_target.z = 0.0
        
        self.status_pub.publish(String(data="LANDING_STARTED"))
    
    def stop_landing(self):
        """停止降落"""
        rospy.loginfo("停止降落")
        self.landing_active = False
        self.status_pub.publish(String(data="LANDING_STOPPED"))
    
    def emergency_landing(self):
        """紧急降落"""
        rospy.logwarn("紧急降落")
        try:
            self.land_client()
            rospy.loginfo("紧急降落命令已发送")
            self.status_pub.publish(String(data="EMERGENCY_LANDING"))
        except Exception as e:
            rospy.logerr(f"紧急降落失败: {e}")
    
    def control_loop(self, event):
        """控制循环"""
        if not self.landing_active:
            return
        
        # 计算45度下降轨迹
        current_height = self.current_pose.pose.position.z
        
        if current_height <= 0.2:
            # 接近地面，完成降落
            self.landing_complete()
            return
        
        # 45度下降的目标位置
        target_pos = self.calculate_descent_position()
        self.publish_setpoint(target_pos)
    
    def calculate_descent_position(self):
        """计算下降位置"""
        current_height = self.current_pose.pose.position.z
        
        # 简单的垂直下降（可以改为45度轨迹）
        target = Point()
        target.x = self.landing_target.x
        target.y = self.landing_target.y
        target.z = max(0.0, current_height - self.landing_speed * 0.1)  # 10Hz控制循环
        
        return target
    
    def publish_setpoint(self, target):
        """发布设定点"""
        setpoint = PositionTarget()
        setpoint.header.stamp = rospy.Time.now()
        setpoint.coordinate_frame = PositionTarget.FRAME_LOCAL_NED
        
        setpoint.type_mask = (
            PositionTarget.IGNORE_VX | PositionTarget.IGNORE_VY | PositionTarget.IGNORE_VZ |
            PositionTarget.IGNORE_AFX | PositionTarget.IGNORE_AFY | PositionTarget.IGNORE_AFZ |
            PositionTarget.IGNORE_YAW_RATE
        )
        
        setpoint.position.x = target.x
        setpoint.position.y = target.y
        setpoint.position.z = target.z
        setpoint.yaw = 0.0
        
        self.setpoint_pub.publish(setpoint)
    
    def landing_complete(self):
        """降落完成"""
        rospy.loginfo("降落完成")
        self.landing_active = False
        self.status_pub.publish(String(data="LANDING_COMPLETE"))

if __name__ == '__main__':
    try:
        node = LandingModeNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
