#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
通信链路节点 - 处理串口和BLE通信
负责接收地面站命令并转发给其他节点
"""

import rospy
import serial
import threading
import time
import json
from std_msgs.msg import String, Bool
from geometry_msgs.msg import Point
from drone_patrol.msg import LinkStatus, AnimalDetection, FlightStatus, LandingCommand

class LinkCommNode:
    def __init__(self):
        rospy.init_node('link_comm', anonymous=True)
        
        # 参数配置
        self.uart_port = rospy.get_param('~uart_port', '/dev/ttyUSB0')
        self.uart_baudrate = rospy.get_param('~uart_baudrate', 115200)
        self.ble_enabled = rospy.get_param('~ble_enabled', True)
        self.status_rate = rospy.get_param('~status_rate', 1.0)  # Hz
        
        # 串口连接
        self.serial_conn = None
        self.serial_connected = False
        self.ble_connected = False
        
        # 发布者
        self.link_status_pub = rospy.Publisher('/drone_patrol/link_status', LinkStatus, queue_size=10)
        self.command_pub = rospy.Publisher('/drone_patrol/ground_command', String, queue_size=10)
        self.takeoff_pub = rospy.Publisher('/drone_patrol/takeoff_command', Bool, queue_size=10)
        self.landing_pub = rospy.Publisher('/drone_patrol/landing_command', LandingCommand, queue_size=10)
        self.waypoint_pub = rospy.Publisher('/drone_patrol/waypoint_command', Point, queue_size=10)
        
        # 订阅者
        rospy.Subscriber('/drone_patrol/flight_status', FlightStatus, self.flight_status_callback)
        rospy.Subscriber('/drone_patrol/animal_detection', AnimalDetection, self.animal_detection_callback)
        
        # 状态变量
        self.last_command = ""
        self.signal_strength = -50.0  # dBm
        self.packet_loss = 0
        self.battery_level = 100.0
        
        # 启动串口连接
        self.setup_serial()
        
        # 启动状态发布定时器
        rospy.Timer(rospy.Duration(1.0/self.status_rate), self.publish_status)
        
        rospy.loginfo("通信链路节点已启动")
    
    def setup_serial(self):
        """设置串口连接"""
        try:
            self.serial_conn = serial.Serial(
                port=self.uart_port,
                baudrate=self.uart_baudrate,
                timeout=1.0
            )
            self.serial_connected = True
            rospy.loginfo(f"串口连接成功: {self.uart_port}")
            
            # 启动串口读取线程
            self.serial_thread = threading.Thread(target=self.serial_read_loop)
            self.serial_thread.daemon = True
            self.serial_thread.start()
            
        except Exception as e:
            rospy.logwarn(f"串口连接失败: {e}")
            self.serial_connected = False
    
    def serial_read_loop(self):
        """串口读取循环"""
        while not rospy.is_shutdown() and self.serial_connected:
            try:
                if self.serial_conn and self.serial_conn.in_waiting > 0:
                    line = self.serial_conn.readline().decode('utf-8').strip()
                    if line:
                        self.process_command(line)
            except Exception as e:
                rospy.logwarn(f"串口读取错误: {e}")
                time.sleep(0.1)
    
    def process_command(self, command_str):
        """处理接收到的命令"""
        try:
            self.last_command = command_str
            rospy.loginfo(f"接收到命令: {command_str}")
            
            # 尝试解析JSON格式命令
            if command_str.startswith('{'):
                command = json.loads(command_str)
                cmd_type = command.get('type', '')
                
                if cmd_type == 'takeoff':
                    self.takeoff_pub.publish(Bool(data=True))
                    rospy.loginfo("发送起飞命令")
                    
                elif cmd_type == 'land':
                    landing_cmd = LandingCommand()
                    landing_cmd.header.stamp = rospy.Time.now()
                    landing_cmd.command = 'START_LANDING'
                    landing_cmd.descent_angle = command.get('angle', 45.0)
                    landing_cmd.descent_speed = command.get('speed', 0.5)
                    landing_cmd.led_enable = True
                    landing_cmd.led_pattern = 1
                    landing_cmd.landing_reason = command.get('reason', 'Manual command')
                    self.landing_pub.publish(landing_cmd)
                    rospy.loginfo("发送降落命令")
                    
                elif cmd_type == 'goto':
                    point = Point()
                    point.x = command.get('x', 0.0)
                    point.y = command.get('y', 0.0)
                    point.z = command.get('z', 1.2)
                    self.waypoint_pub.publish(point)
                    rospy.loginfo(f"发送航点命令: ({point.x}, {point.y}, {point.z})")
                    
                elif cmd_type == 'patrol':
                    # 开始巡逻模式
                    self.command_pub.publish(String(data="START_PATROL"))
                    rospy.loginfo("开始巡逻模式")
                    
                elif cmd_type == 'return':
                    # 返回起飞点
                    self.command_pub.publish(String(data="RETURN_HOME"))
                    rospy.loginfo("返回起飞点")
            
            else:
                # 简单文本命令
                self.command_pub.publish(String(data=command_str))
                
        except Exception as e:
            rospy.logwarn(f"命令处理错误: {e}")
    
    def send_data(self, data):
        """发送数据到地面站"""
        try:
            if self.serial_connected and self.serial_conn:
                if isinstance(data, str):
                    self.serial_conn.write((data + '\n').encode('utf-8'))
                else:
                    self.serial_conn.write((json.dumps(data) + '\n').encode('utf-8'))
        except Exception as e:
            rospy.logwarn(f"数据发送错误: {e}")
    
    def flight_status_callback(self, msg):
        """飞行状态回调 - 转发给地面站"""
        status_data = {
            'type': 'flight_status',
            'mode': msg.flight_mode,
            'armed': msg.armed,
            'altitude': msg.relative_altitude,
            'battery': msg.battery_voltage,
            'gps_fix': msg.gps_fix,
            'satellites': msg.satellites,
            'position': {
                'x': msg.current_pose.position.x,
                'y': msg.current_pose.position.y,
                'z': msg.current_pose.position.z
            }
        }
        self.send_data(status_data)
    
    def animal_detection_callback(self, msg):
        """动物检测回调 - 转发给地面站"""
        detection_data = {
            'type': 'animal_detection',
            'animal_type': msg.animal_type,
            'confidence': msg.confidence,
            'position_2d': {
                'x': msg.position_2d.x,
                'y': msg.position_2d.y
            },
            'distance': msg.distance,
            'tracking': msg.tracking_active,
            'timestamp': msg.header.stamp.to_sec()
        }
        self.send_data(detection_data)
        rospy.loginfo(f"检测到动物: {msg.animal_type}, 置信度: {msg.confidence:.2f}")
    
    def publish_status(self, event):
        """发布链路状态"""
        status = LinkStatus()
        status.header.stamp = rospy.Time.now()
        status.ble_connected = self.ble_connected
        status.uart_connected = self.serial_connected
        status.signal_strength = self.signal_strength
        status.packet_loss = self.packet_loss
        status.last_command = self.last_command
        status.device_status = "ONLINE" if self.serial_connected else "OFFLINE"
        status.battery_level = self.battery_level
        
        self.link_status_pub.publish(status)
    
    def shutdown(self):
        """节点关闭处理"""
        if self.serial_conn:
            self.serial_conn.close()
        rospy.loginfo("通信链路节点已关闭")

if __name__ == '__main__':
    try:
        node = LinkCommNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
    finally:
        if 'node' in locals():
            node.shutdown()
