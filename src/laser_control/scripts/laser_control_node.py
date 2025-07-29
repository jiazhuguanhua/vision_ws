#!/usr/bin/env python3
"""
Simple Laser Control Node v2.0
Description: 激光控制器，仅支持GPIO控制
Date: 2025-07-29
Version: 2.0.0
"""

import rospy
import yaml
import os
import threading
from std_msgs.msg import Bool
import RPi.GPIO as GPIO


class LaserController:
    """简单激光控制器 - 仅GPIO控制"""
    
    def __init__(self):
        """初始化激光控制器"""
        rospy.init_node('laser_control_node', anonymous=True)
        rospy.loginfo("🔫 Simple Laser Control Node v2.0 Started")
        
        # 加载配置参数
        self.load_config()
        
        # 激光状态
        self.laser_on = False
        self.fire_timer = None
        
        # 初始化GPIO控制
        self.init_gpio_control()
        
        # 初始化ROS通信
        self.init_ros_communication()
        
        rospy.loginfo("✅ Laser Controller initialized")
    
    def load_config(self):
        """加载配置文件"""
        try:
            config_path = rospy.get_param('~config_file', 
                                        os.path.join(os.path.dirname(__file__), 
                                                   '../config/mission_config.yaml'))
            with open(config_path, 'r', encoding='utf-8') as file:
                self.config = yaml.safe_load(file)
            rospy.loginfo(f"📋 Config loaded from: {config_path}")
            
        except Exception as e:
            rospy.logwarn(f"⚠️ Failed to load config: {e}, using defaults")
            self.config = self.get_default_config()
        
        # 从ROS参数覆盖配置
        self.override_from_params()
    
    def override_from_params(self):
        """从ROS参数覆盖配置"""
        # GPIO引脚
        if rospy.has_param('~gpio_pin'):
            self.gpio_pin = rospy.get_param('~gpio_pin', 18)
        else:
            self.gpio_pin = self.config.get('laser_control', {}).get('gpio', {}).get('pin', 18)
        
        # 高/低电平有效
        if rospy.has_param('~active_high'):
            self.active_high = rospy.get_param('~active_high', True)
        else:
            self.active_high = self.config.get('laser_control', {}).get('gpio', {}).get('active_high', True)
        
        # 发射持续时间
        if rospy.has_param('~fire_duration'):
            self.fire_duration = rospy.get_param('~fire_duration', 3.0)
        else:
            self.fire_duration = self.config.get('laser_control', {}).get('laser', {}).get('fire_duration', 3.0)
        
        rospy.loginfo(f"� GPIO pin: {self.gpio_pin}, Active high: {self.active_high}, Duration: {self.fire_duration}s")
    
    def get_default_config(self):
        """获取默认配置"""
        return {
            'laser_control': {
                'gpio': {
                    'pin': 18,
                    'active_high': True
                },
                'laser': {
                    'fire_duration': 3.0
                }
            }
        }
    
    def init_gpio_control(self):
        """初始化GPIO控制"""
        try:
            # 设置GPIO模式
            GPIO.setmode(GPIO.BCM)
            GPIO.setup(self.gpio_pin, GPIO.OUT)
            
            # 确保激光关闭
            self.set_gpio_pin(False)
            
            rospy.loginfo(f"✅ GPIO initialized on pin {self.gpio_pin}")
            
        except Exception as e:
            rospy.logfatal(f"❌ Failed to initialize GPIO: {e}")
            rospy.signal_shutdown("GPIO initialization failed")
    
    def set_gpio_pin(self, state):
        """设置GPIO引脚状态"""
        if self.active_high:
            gpio_state = GPIO.HIGH if state else GPIO.LOW
        else:
            gpio_state = GPIO.LOW if state else GPIO.HIGH
        
        GPIO.output(self.gpio_pin, gpio_state)
        rospy.logdebug(f"🔌 GPIO pin {self.gpio_pin} set to {'HIGH' if gpio_state == GPIO.HIGH else 'LOW'}")
    
    def init_ros_communication(self):
        """初始化ROS通信"""
        # 订阅激光控制命令
        self.fire_sub = rospy.Subscriber('/laser_fire', Bool, self.fire_callback, queue_size=1)
        
        # 发布激光状态
        self.status_pub = rospy.Publisher('/laser_status', Bool, queue_size=10)
        
        rospy.loginfo("📡 ROS communication initialized")
    
    def fire_callback(self, msg):
        """激光发射命令回调"""
        if msg.data:
            self.fire_laser()
        else:
            self.stop_laser()
    
    def fire_laser(self):
        """发射激光"""
        if self.laser_on:
            rospy.logwarn("⚠️ Laser is already on")
            return
        
        try:
            rospy.logwarn(f"🔥 LASER FIRED! Duration: {self.fire_duration}s")
            
            # 启动激光
            self.set_gpio_pin(True)
            self.laser_on = True
            self.publish_status()
            
            # 设置自动关闭定时器
            self.fire_timer = threading.Timer(self.fire_duration, self.auto_stop_laser)
            self.fire_timer.start()
            
        except Exception as e:
            rospy.logerr(f"❌ Error firing laser: {e}")
            self.stop_laser()
    
    def stop_laser(self):
        """停止激光"""
        if not self.laser_on:
            return
        
        try:
            # 取消定时器
            if self.fire_timer:
                self.fire_timer.cancel()
                self.fire_timer = None
            
            # 关闭激光
            self.set_gpio_pin(False)
            self.laser_on = False
            self.publish_status()
            
            rospy.loginfo("💡 Laser stopped")
            
        except Exception as e:
            rospy.logerr(f"❌ Error stopping laser: {e}")
    
    def auto_stop_laser(self):
        """自动停止激光（定时器回调）"""
        rospy.loginfo("⏰ Auto-stopping laser after timeout")
        self.stop_laser()
    
    def publish_status(self):
        """发布激光状态"""
        try:
            status_msg = Bool()
            status_msg.data = self.laser_on
            self.status_pub.publish(status_msg)
        except Exception as e:
            rospy.logwarn(f"⚠️ Error publishing status: {e}")
    
    def cleanup(self):
        """清理资源"""
        try:
            rospy.loginfo("🧹 Cleaning up...")
            
            # 确保激光关闭
            if self.laser_on:
                self.stop_laser()
            
            # 清理GPIO资源
            GPIO.cleanup()
            rospy.loginfo("✅ GPIO cleanup completed")
                
        except Exception as e:
            rospy.logerr(f"❌ Error during cleanup: {e}")
    
    def run(self):
        """主运行循环"""
        rospy.loginfo("🚀 Laser Controller running...")
        
        try:
            rospy.spin()
        except KeyboardInterrupt:
            rospy.loginfo("🛑 Laser controller interrupted by user")
        finally:
            self.cleanup()


if __name__ == "__main__":
    try:
        controller = LaserController()
        controller.run()
    except rospy.ROSInterruptException:
        rospy.loginfo("🛑 Laser control node interrupted")
    except Exception as e:
        rospy.logerr(f"❌ Laser control node failed: {e}")
    finally:
        # 确保清理GPIO资源
        try:
            GPIO.cleanup()
        except:
            pass
