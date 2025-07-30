#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
简单通信链路节点 - 串口通信
"""

import rospy
import serial
import threading
import json
from std_msgs.msg import String, Bool
from geometry_msgs.msg import Point

class LinkCommNode:
    def __init__(self):
        rospy.init_node('link_comm', anonymous=True)
        
        # 参数
        self.uart_port = rospy.get_param('~uart_port', '/dev/ttyUSB0')
        self.uart_baudrate = rospy.get_param('~uart_baudrate', 115200)
        
        # 串口
        self.serial_conn = None
        self.setup_serial()
        
        # 发布者
        self.command_pub = rospy.Publisher('/ground_command', String, queue_size=10)
        self.takeoff_pub = rospy.Publisher('/takeoff_command', Bool, queue_size=10)
        self.waypoint_pub = rospy.Publisher('/waypoint_command', Point, queue_size=10)
        
        rospy.loginfo("通信链路节点启动")
    
    def setup_serial(self):
        """设置串口"""
        try:
            self.serial_conn = serial.Serial(
                port=self.uart_port,
                baudrate=self.uart_baudrate,
                timeout=1.0
            )
            rospy.loginfo(f"串口连接成功: {self.uart_port}")
            
            # 启动读取线程
            thread = threading.Thread(target=self.serial_read_loop)
            thread.daemon = True
            thread.start()
            
        except Exception as e:
            rospy.logwarn(f"串口连接失败: {e}")
    
    def serial_read_loop(self):
        """串口读取循环"""
        while not rospy.is_shutdown():
            try:
                if self.serial_conn and self.serial_conn.in_waiting > 0:
                    line = self.serial_conn.readline().decode('utf-8').strip()
                    if line:
                        self.process_command(line)
            except Exception as e:
                rospy.logwarn(f"串口读取错误: {e}")
    
    def process_command(self, command_str):
        """处理命令"""
        try:
            rospy.loginfo(f"接收命令: {command_str}")
            
            if command_str.startswith('{'):
                # JSON命令
                command = json.loads(command_str)
                cmd_type = command.get('type', '')
                
                if cmd_type == 'takeoff':
                    self.takeoff_pub.publish(Bool(data=True))
                elif cmd_type == 'goto':
                    point = Point()
                    point.x = command.get('x', 0.0)
                    point.y = command.get('y', 0.0)
                    point.z = command.get('z', 1.2)
                    self.waypoint_pub.publish(point)
                elif cmd_type == 'patrol':
                    self.command_pub.publish(String(data="START_PATROL"))
                elif cmd_type == 'return':
                    self.command_pub.publish(String(data="RETURN_HOME"))
                elif cmd_type == 'check':
                    self.command_pub.publish(String(data="CHECK_WAYPOINTS"))
            else:
                # 简单文本命令
                self.command_pub.publish(String(data=command_str))
                
        except Exception as e:
            rospy.logwarn(f"命令处理错误: {e}")

if __name__ == '__main__':
    try:
        node = LinkCommNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
