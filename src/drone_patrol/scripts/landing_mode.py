#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
降落模式节点 - 实现45度角精确降落和LED控制
集成安全检查和紧急处理
"""

import rospy
import math
import time
from std_msgs.msg import Bool, String, Int32
from geometry_msgs.msg import Point, PoseStamped, Twist
from mavros_msgs.msg import State, PositionTarget
from mavros_msgs.srv import CommandTOL, SetMode
from drone_patrol.msg import LandingCommand, FlightStatus, AnimalDetection

class LandingModeNode:
    def __init__(self):
        rospy.init_node('landing_mode', anonymous=True)
        
        # 参数配置
        self.landing_speed = rospy.get_param('~landing_speed', 0.5)  # m/s
        self.descent_angle = rospy.get_param('~descent_angle', 45.0)  # degrees
        self.minimum_altitude = rospy.get_param('~minimum_altitude', 0.2)  # m
        self.safety_radius = rospy.get_param('~safety_radius', 2.0)  # m
        self.led_enabled = rospy.get_param('~led_enabled', True)
        
        # LED控制GPIO引脚 (如果使用GPIO)
        self.led_pin = rospy.get_param('~led_pin', 18)
        
        # 状态变量
        self.landing_active = False
        self.landing_phase = "IDLE"  # IDLE, PREPARE, DESCENDING, FINAL, LANDED
        self.current_pose = PoseStamped()
        self.target_landing_point = Point()
        self.descent_start_point = Point()
        self.landing_command = None
        self.mavros_state = State()
        
        # 安全检查
        self.ground_detection = False
        self.obstacle_detected = False
        self.battery_critical = False
        
        # LED状态
        self.led_pattern = 0
        self.led_state = False
        self.led_last_toggle = time.time()
        
        # MAVROS服务
        self.land_client = rospy.ServiceProxy('/mavros/cmd/land', CommandTOL)
        self.set_mode_client = rospy.ServiceProxy('/mavros/set_mode', SetMode)
        
        # 发布者
        self.setpoint_pub = rospy.Publisher('/mavros/setpoint_raw/local', PositionTarget, queue_size=10)
        self.status_pub = rospy.Publisher('/drone_patrol/landing_status', String, queue_size=10)
        self.led_pub = rospy.Publisher('/drone_patrol/led_control', Int32, queue_size=10)
        
        # 订阅者
        rospy.Subscriber('/drone_patrol/landing_command', LandingCommand, self.landing_command_callback)
        rospy.Subscriber('/mavros/local_position/pose', PoseStamped, self.pose_callback)
        rospy.Subscriber('/mavros/state', State, self.state_callback)
        rospy.Subscriber('/drone_patrol/flight_status', FlightStatus, self.flight_status_callback)
        rospy.Subscriber('/drone_patrol/animal_detection', AnimalDetection, self.animal_detection_callback)
        
        # 控制循环定时器
        rospy.Timer(rospy.Duration(0.1), self.control_loop)  # 10Hz
        rospy.Timer(rospy.Duration(0.2), self.led_control_loop)  # 5Hz LED控制
        
        # 初始化LED GPIO (如果需要)
        self.setup_led()
        
        rospy.loginfo("降落模式节点已启动")
    
    def setup_led(self):
        """设置LED控制"""
        if self.led_enabled:
            try:
                import RPi.GPIO as GPIO
                GPIO.setmode(GPIO.BCM)
                GPIO.setup(self.led_pin, GPIO.OUT)
                GPIO.output(self.led_pin, GPIO.LOW)
                rospy.loginfo(f"LED控制初始化成功: GPIO {self.led_pin}")
            except ImportError:
                rospy.logwarn("RPi.GPIO库未安装，使用软件LED控制")
                self.led_enabled = False
            except Exception as e:
                rospy.logwarn(f"LED初始化失败: {e}")
                self.led_enabled = False
    
    def landing_command_callback(self, msg):
        """降落命令回调"""
        self.landing_command = msg
        
        if msg.command == "START_LANDING":
            self.start_landing(msg)
        elif msg.command == "ABORT_LANDING":
            self.abort_landing()
        elif msg.command == "EMERGENCY_LAND":
            self.emergency_landing()
    
    def pose_callback(self, msg):
        """位置回调"""
        self.current_pose = msg
    
    def state_callback(self, msg):
        """MAVROS状态回调"""
        self.mavros_state = msg
    
    def flight_status_callback(self, msg):
        """飞行状态回调"""
        # 检查电池状态
        if msg.battery_voltage < 11.0:  # 假设3S电池，低于11V为危险
            self.battery_critical = True
        else:
            self.battery_critical = False
    
    def animal_detection_callback(self, msg):
        """动物检测回调 - 降落时的安全检查"""
        if self.landing_active and self.landing_phase in ["DESCENDING", "FINAL"]:
            # 检查降落区域是否有动物
            if msg.distance < self.safety_radius and msg.confidence > 0.7:
                rospy.logwarn(f"降落区域检测到动物: {msg.animal_type}")
                # 可以选择暂停降落或改变降落点
                self.pause_landing_for_safety()
    
    def start_landing(self, landing_cmd):
        """开始降落序列"""
        if self.landing_active:
            rospy.logwarn("降落已在进行中")
            return
        
        rospy.loginfo(f"开始降落序列: {landing_cmd.landing_reason}")
        
        self.landing_active = True
        self.landing_phase = "PREPARE"
        self.landing_command = landing_cmd
        
        # 设置降落目标点
        if landing_cmd.landing_position.x != 0 or landing_cmd.landing_position.y != 0:
            self.target_landing_point = landing_cmd.landing_position
        else:
            # 默认在当前位置下方降落
            self.target_landing_point.x = self.current_pose.pose.position.x
            self.target_landing_point.y = self.current_pose.pose.position.y
            self.target_landing_point.z = 0.0
        
        # 计算45度下降起始点
        current_altitude = self.current_pose.pose.position.z
        horizontal_distance = current_altitude * math.tan(math.radians(45.0))
        
        # 计算下降起始点（在目标点的相反方向）
        direction_x = self.target_landing_point.x - self.current_pose.pose.position.x
        direction_y = self.target_landing_point.y - self.current_pose.pose.position.y
        direction_length = math.sqrt(direction_x**2 + direction_y**2)
        
        if direction_length > 0:
            unit_x = direction_x / direction_length
            unit_y = direction_y / direction_length
        else:
            # 如果目标就在当前位置下方，选择一个默认方向
            unit_x = 1.0
            unit_y = 0.0
        
        self.descent_start_point.x = self.target_landing_point.x - unit_x * horizontal_distance
        self.descent_start_point.y = self.target_landing_point.y - unit_y * horizontal_distance
        self.descent_start_point.z = current_altitude
        
        # 设置LED模式
        if landing_cmd.led_enable:
            self.led_pattern = landing_cmd.led_pattern
            self.start_led_pattern()
        
        rospy.loginfo(f"降落参数:")
        rospy.loginfo(f"  目标点: ({self.target_landing_point.x:.2f}, {self.target_landing_point.y:.2f}, {self.target_landing_point.z:.2f})")
        rospy.loginfo(f"  下降起始点: ({self.descent_start_point.x:.2f}, {self.descent_start_point.y:.2f}, {self.descent_start_point.z:.2f})")
        rospy.loginfo(f"  下降角度: {landing_cmd.descent_angle:.1f}度")
        rospy.loginfo(f"  下降速度: {landing_cmd.descent_speed:.1f}m/s")
    
    def abort_landing(self):
        """中止降落"""
        if not self.landing_active:
            return
        
        rospy.loginfo("中止降落，保持当前位置")
        self.landing_active = False
        self.landing_phase = "ABORTED"
        self.stop_led_pattern()
        
        # 悬停在当前位置
        self.hover_at_current_position()
    
    def emergency_landing(self):
        """紧急降落"""
        rospy.logwarn("执行紧急降落")
        self.landing_active = True
        self.landing_phase = "EMERGENCY"
        
        # 使用MAVROS直接降落
        try:
            result = self.land_client()
            if result.success:
                rospy.loginfo("紧急降落命令已发送")
            else:
                rospy.logwarn("紧急降落命令发送失败")
        except Exception as e:
            rospy.logerr(f"紧急降落服务调用失败: {e}")
        
        # 启用紧急LED模式
        self.led_pattern = 99  # 紧急模式
        self.start_led_pattern()
    
    def pause_landing_for_safety(self):
        """因安全原因暂停降落"""
        if self.landing_phase in ["DESCENDING", "FINAL"]:
            rospy.logwarn("因安全原因暂停降落")
            self.landing_phase = "PAUSED"
            
            # 悬停在当前位置
            self.hover_at_current_position()
            
            # 等待安全清理后继续
            rospy.Timer(rospy.Duration(5.0), self.resume_landing_check, oneshot=True)
    
    def resume_landing_check(self, event):
        """检查是否可以恢复降落"""
        if self.landing_phase == "PAUSED":
            # 这里可以添加更复杂的安全检查逻辑
            rospy.loginfo("恢复降落序列")
            self.landing_phase = "DESCENDING"
    
    def control_loop(self, event):
        """主控制循环"""
        if not self.landing_active:
            return
        
        if self.landing_phase == "PREPARE":
            self.prepare_for_landing()
        elif self.landing_phase == "DESCENDING":
            self.execute_descent()
        elif self.landing_phase == "FINAL":
            self.final_landing()
        elif self.landing_phase == "PAUSED":
            self.maintain_hover()
    
    def prepare_for_landing(self):
        """准备降落阶段"""
        # 飞到下降起始点
        distance_to_start = self.calculate_distance_to_point(self.descent_start_point)
        
        if distance_to_start < 0.5:  # 到达起始点
            self.landing_phase = "DESCENDING"
            rospy.loginfo("到达下降起始点，开始45度下降")
        else:
            # 继续飞向起始点
            self.publish_setpoint(self.descent_start_point)
    
    def execute_descent(self):
        """执行45度下降"""
        # 计算当前应该在的位置（沿45度轨迹）
        current_time = time.time()
        elapsed_time = current_time - getattr(self, '_descent_start_time', current_time)
        
        if not hasattr(self, '_descent_start_time'):
            self._descent_start_time = current_time
            elapsed_time = 0
        
        # 计算沿轨迹的距离
        descent_distance = self.landing_command.descent_speed * elapsed_time
        total_distance = self.calculate_distance_to_point(self.target_landing_point)
        
        if descent_distance >= total_distance:
            # 到达目标点附近，进入最终降落阶段
            self.landing_phase = "FINAL"
            rospy.loginfo("进入最终降落阶段")
            return
        
        # 计算当前目标位置
        progress = descent_distance / total_distance
        current_target = Point()
        current_target.x = self.descent_start_point.x + progress * (self.target_landing_point.x - self.descent_start_point.x)
        current_target.y = self.descent_start_point.y + progress * (self.target_landing_point.y - self.descent_start_point.y)
        current_target.z = self.descent_start_point.z + progress * (self.target_landing_point.z - self.descent_start_point.z)
        
        # 安全检查
        if current_target.z < self.minimum_altitude:
            current_target.z = self.minimum_altitude
            self.landing_phase = "FINAL"
        
        self.publish_setpoint(current_target)
        
        # 检查是否接近地面
        if self.current_pose.pose.position.z < self.minimum_altitude + 0.3:
            self.landing_phase = "FINAL"
    
    def final_landing(self):
        """最终降落阶段"""
        # 使用MAVROS的降落命令
        if not hasattr(self, '_final_landing_sent'):
            try:
                result = self.land_client()
                if result.success:
                    rospy.loginfo("最终降落命令已发送")
                    self._final_landing_sent = True
                else:
                    rospy.logwarn("最终降落命令发送失败，使用手动降落")
                    # 手动控制最终下降
                    target = Point()
                    target.x = self.target_landing_point.x
                    target.y = self.target_landing_point.y
                    target.z = max(0.0, self.current_pose.pose.position.z - 0.1)
                    self.publish_setpoint(target)
            except Exception as e:
                rospy.logerr(f"降落服务调用失败: {e}")
        
        # 检查是否已着陆
        if self.current_pose.pose.position.z < 0.1 and not self.mavros_state.armed:
            self.landing_complete()
    
    def landing_complete(self):
        """降落完成"""
        rospy.loginfo("降落完成")
        self.landing_active = False
        self.landing_phase = "LANDED"
        self.stop_led_pattern()
        
        # 发布降落完成状态
        self.status_pub.publish(String(data="LANDING_COMPLETE"))
    
    def maintain_hover(self):
        """保持悬停"""
        self.hover_at_current_position()
    
    def hover_at_current_position(self):
        """在当前位置悬停"""
        hover_point = Point()
        hover_point.x = self.current_pose.pose.position.x
        hover_point.y = self.current_pose.pose.position.y
        hover_point.z = self.current_pose.pose.position.z
        self.publish_setpoint(hover_point)
    
    def publish_setpoint(self, target_point):
        """发布位置设定点"""
        setpoint = PositionTarget()
        setpoint.header.stamp = rospy.Time.now()
        setpoint.header.frame_id = "map"
        setpoint.coordinate_frame = PositionTarget.FRAME_LOCAL_NED
        
        setpoint.type_mask = (
            PositionTarget.IGNORE_VX |
            PositionTarget.IGNORE_VY |
            PositionTarget.IGNORE_VZ |
            PositionTarget.IGNORE_AFX |
            PositionTarget.IGNORE_AFY |
            PositionTarget.IGNORE_AFZ |
            PositionTarget.IGNORE_YAW_RATE
        )
        
        setpoint.position.x = target_point.x
        setpoint.position.y = target_point.y
        setpoint.position.z = target_point.z
        setpoint.yaw = 0.0
        
        self.setpoint_pub.publish(setpoint)
    
    def calculate_distance_to_point(self, point):
        """计算到指定点的距离"""
        dx = self.current_pose.pose.position.x - point.x
        dy = self.current_pose.pose.position.y - point.y
        dz = self.current_pose.pose.position.z - point.z
        return math.sqrt(dx*dx + dy*dy + dz*dz)
    
    def start_led_pattern(self):
        """启动LED模式"""
        self.led_state = True
        rospy.loginfo(f"启动LED模式: {self.led_pattern}")
    
    def stop_led_pattern(self):
        """停止LED模式"""
        self.led_pattern = 0
        self.led_state = False
        self.control_led(False)
    
    def led_control_loop(self, event):
        """LED控制循环"""
        if self.led_pattern == 0:
            return
        
        current_time = time.time()
        
        if self.led_pattern == 1:
            # 慢速闪烁 (1Hz)
            if current_time - self.led_last_toggle > 0.5:
                self.led_state = not self.led_state
                self.led_last_toggle = current_time
                
        elif self.led_pattern == 2:
            # 快速闪烁 (2Hz)
            if current_time - self.led_last_toggle > 0.25:
                self.led_state = not self.led_state
                self.led_last_toggle = current_time
                
        elif self.led_pattern == 3:
            # 呼吸灯效果
            # 简化为持续亮起
            self.led_state = True
            
        elif self.led_pattern == 99:
            # 紧急模式 - 极快闪烁
            if current_time - self.led_last_toggle > 0.1:
                self.led_state = not self.led_state
                self.led_last_toggle = current_time
        
        self.control_led(self.led_state)
    
    def control_led(self, state):
        """控制LED开关"""
        if self.led_enabled:
            try:
                import RPi.GPIO as GPIO
                GPIO.output(self.led_pin, GPIO.HIGH if state else GPIO.LOW)
            except:
                pass
        
        # 发布LED状态 (用于软件显示)
        self.led_pub.publish(Int32(data=1 if state else 0))

if __name__ == '__main__':
    try:
        node = LandingModeNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
