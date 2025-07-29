#!/usr/bin/env python3
"""
Laser Control Node for Autonomous Drone Mission System
Author: AI Assistant
Description: 控制激光发射器，支持GPIO和串口控制
"""

import rospy
import yaml
import os
import time
import threading
from std_msgs.msg import Bool

# 尝试导入GPIO库（仅在树莓派上可用）
try:
    import RPi.GPIO as GPIO
    GPIO_AVAILABLE = True
except ImportError:
    GPIO_AVAILABLE = False
    rospy.logwarn("RPi.GPIO not available, using simulation mode")

# 尝试导入串口库
try:
    import serial
    SERIAL_AVAILABLE = True
except ImportError:
    SERIAL_AVAILABLE = False
    rospy.logwarn("pyserial not available")


class LaserController:
    """激光控制器"""
    
    def __init__(self):
        """初始化激光控制器"""
        rospy.init_node('laser_control_node', anonymous=True)
        rospy.loginfo("Laser Control Node Started")
        
        # 加载配置参数
        self.load_config()
        
        # 激光状态
        self.laser_on = False
        self.fire_timer = None
        self.safety_timer = None
        
        # 控制方式
        self.control_method = self.determine_control_method()
        
        # 初始化硬件控制
        self.init_hardware_control()
        
        # 初始化ROS通信
        self.init_ros_communication()
        
        rospy.loginfo(f"Laser Controller initialized with {self.control_method} control")
    
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
            'laser_control': {
                'gpio': {
                    'pin': 18,
                    'active_high': True
                },
                'laser': {
                    'fire_duration': 3.0,
                    'safety_timeout': 10.0
                },
                'serial': {
                    'port': '/dev/ttyUSB0',
                    'baudrate': 9600,
                    'on_command': 'LASER_ON\n',
                    'off_command': 'LASER_OFF\n'
                },
                'topics': {
                    'fire_command': '/laser_fire'
                }
            }
        }
    
    def determine_control_method(self):
        """确定控制方式"""
        # 优先级：GPIO > Serial > Simulation
        if GPIO_AVAILABLE:
            return "GPIO"
        elif SERIAL_AVAILABLE:
            return "Serial"
        else:
            return "Simulation"
    
    def init_hardware_control(self):
        """初始化硬件控制"""
        if self.control_method == "GPIO":
            self.init_gpio_control()
        elif self.control_method == "Serial":
            self.init_serial_control()
        else:
            self.init_simulation_control()
    
    def init_gpio_control(self):
        """初始化GPIO控制"""
        try:
            self.gpio_pin = self.config['laser_control']['gpio']['pin']
            self.active_high = self.config['laser_control']['gpio']['active_high']
            
            GPIO.setmode(GPIO.BCM)
            GPIO.setup(self.gpio_pin, GPIO.OUT)
            
            # 确保激光关闭
            GPIO.output(self.gpio_pin, GPIO.LOW if self.active_high else GPIO.HIGH)
            
            rospy.loginfo(f"GPIO control initialized on pin {self.gpio_pin}")
            
        except Exception as e:
            rospy.logerr(f"Failed to initialize GPIO: {e}")
            self.control_method = "Simulation"
            self.init_simulation_control()
    
    def init_serial_control(self):
        """初始化串口控制"""
        try:
            serial_config = self.config['laser_control']['serial']
            
            self.serial_port = serial.Serial(
                port=serial_config['port'],
                baudrate=serial_config['baudrate'],
                timeout=1
            )
            
            self.on_command = serial_config['on_command'].encode()
            self.off_command = serial_config['off_command'].encode()
            
            # 确保激光关闭
            self.serial_port.write(self.off_command)
            
            rospy.loginfo(f"Serial control initialized on {serial_config['port']}")
            
        except Exception as e:
            rospy.logerr(f"Failed to initialize serial: {e}")
            self.control_method = "Simulation"
            self.init_simulation_control()
    
    def init_simulation_control(self):
        """初始化仿真控制"""
        rospy.loginfo("Using simulation mode for laser control")
    
    def init_ros_communication(self):
        """初始化ROS通信"""
        # 订阅激光控制命令
        fire_topic = self.config['laser_control']['topics']['fire_command']
        self.fire_sub = rospy.Subscriber(fire_topic, Bool, self.fire_callback)
        
        # 发布激光状态
        self.status_pub = rospy.Publisher("/laser_status", Bool, queue_size=10)
        
        rospy.loginfo("ROS communication initialized")
    
    def fire_callback(self, msg):
        """激光发射命令回调"""
        if msg.data:
            self.fire_laser()
        else:
            self.stop_laser()
    
    def fire_laser(self):
        """发射激光"""
        if self.laser_on:
            rospy.logwarn("Laser is already on")
            return
        
        try:
            # 启动激光
            self.set_laser_state(True)
            
            # 设置自动关闭定时器
            fire_duration = self.config['laser_control']['laser']['fire_duration']
            self.fire_timer = threading.Timer(fire_duration, self.auto_stop_laser)
            self.fire_timer.start()
            
            # 设置安全超时定时器
            safety_timeout = self.config['laser_control']['laser']['safety_timeout']
            self.safety_timer = threading.Timer(safety_timeout, self.emergency_stop_laser)
            self.safety_timer.start()
            
            rospy.loginfo(f"Laser fired for {fire_duration} seconds")
            
        except Exception as e:
            rospy.logerr(f"Error firing laser: {e}")
            self.emergency_stop_laser()
    
    def stop_laser(self):
        """停止激光"""
        if not self.laser_on:
            return
        
        try:
            # 取消定时器
            if self.fire_timer:
                self.fire_timer.cancel()
                self.fire_timer = None
            
            if self.safety_timer:
                self.safety_timer.cancel()
                self.safety_timer = None
            
            # 关闭激光
            self.set_laser_state(False)
            
            rospy.loginfo("Laser stopped")
            
        except Exception as e:
            rospy.logerr(f"Error stopping laser: {e}")
    
    def auto_stop_laser(self):
        """自动停止激光（定时器回调）"""
        rospy.loginfo("Auto-stopping laser after timeout")
        self.stop_laser()
    
    def emergency_stop_laser(self):
        """紧急停止激光（安全超时）"""
        rospy.logwarn("Emergency laser stop - safety timeout reached!")
        self.set_laser_state(False)
        self.laser_on = False
        
        # 取消所有定时器
        if self.fire_timer:
            self.fire_timer.cancel()
            self.fire_timer = None
        if self.safety_timer:
            self.safety_timer.cancel()
            self.safety_timer = None
    
    def set_laser_state(self, state):
        """设置激光状态"""
        try:
            if self.control_method == "GPIO":
                self.set_gpio_state(state)
            elif self.control_method == "Serial":
                self.set_serial_state(state)
            else:
                self.set_simulation_state(state)
            
            self.laser_on = state
            
            # 发布状态
            status_msg = Bool()
            status_msg.data = state
            self.status_pub.publish(status_msg)
            
        except Exception as e:
            rospy.logerr(f"Error setting laser state: {e}")
            raise
    
    def set_gpio_state(self, state):
        """设置GPIO状态"""
        if self.active_high:
            gpio_state = GPIO.HIGH if state else GPIO.LOW
        else:
            gpio_state = GPIO.LOW if state else GPIO.HIGH
        
        GPIO.output(self.gpio_pin, gpio_state)
        rospy.loginfo(f"GPIO pin {self.gpio_pin} set to {'HIGH' if gpio_state else 'LOW'}")
    
    def set_serial_state(self, state):
        """设置串口状态"""
        command = self.on_command if state else self.off_command
        self.serial_port.write(command)
        self.serial_port.flush()
        
        rospy.loginfo(f"Serial command sent: {command.decode().strip()}")
    
    def set_simulation_state(self, state):
        """设置仿真状态"""
        status = "ON" if state else "OFF"
        rospy.loginfo(f"[SIMULATION] Laser {status}")
    
    def publish_status(self):
        """定期发布激光状态"""
        status_msg = Bool()
        status_msg.data = self.laser_on
        self.status_pub.publish(status_msg)
    
    def cleanup(self):
        """清理资源"""
        try:
            # 确保激光关闭
            if self.laser_on:
                self.emergency_stop_laser()
            
            # 清理GPIO资源
            if self.control_method == "GPIO" and GPIO_AVAILABLE:
                GPIO.cleanup()
                rospy.loginfo("GPIO cleanup completed")
            
            # 关闭串口
            if self.control_method == "Serial" and hasattr(self, 'serial_port'):
                if self.serial_port.is_open:
                    self.serial_port.close()
                rospy.loginfo("Serial port closed")
                
        except Exception as e:
            rospy.logerr(f"Error during cleanup: {e}")
    
    def run(self):
        """主运行循环"""
        rospy.loginfo("Laser Controller running...")
        
        # 状态发布频率
        rate = rospy.Rate(2)  # 2Hz
        
        try:
            while not rospy.is_shutdown():
                self.publish_status()
                rate.sleep()
                
        except KeyboardInterrupt:
            rospy.loginfo("Laser controller interrupted by user")
        except Exception as e:
            rospy.logerr(f"Error in main loop: {e}")
        finally:
            self.cleanup()


if __name__ == "__main__":
    try:
        controller = LaserController()
        controller.run()
    except rospy.ROSInterruptException:
        rospy.loginfo("Laser control node interrupted")
    except Exception as e:
        rospy.logerr(f"Laser control node failed: {e}")
    finally:
        # 确保清理GPIO资源
        if GPIO_AVAILABLE:
            try:
                GPIO.cleanup()
            except:
                pass
