#!/usr/bin/env python3
"""
Dual Laser Control Node v4.0
Description: 双激光头控制器，支持GPIO控制和LaserControl消息
Date: 2025-08-01
Version: 4.0.0
"""

import rospy
import yaml
import os
import threading
from std_msgs.msg import Bool
from common_msgs.msg import LaserControl
import RPi.GPIO as GPIO


class DualLaserController:
    """双激光头控制器 - 支持GPIO18和GPIO19"""
    
    def __init__(self):
        """初始化双激光控制器"""
        rospy.init_node('laser_control_node', anonymous=True)
        rospy.loginfo("🔫 Dual Laser Control Node v4.0 Started")
        
        # 激光头配置
        self.laser_configs = {
            1: {'gpio_pin': 18, 'name': 'Laser_1'},
            2: {'gpio_pin': 19, 'name': 'Laser_2'}
        }
        
        # 激光状态跟踪
        self.laser_states = {1: False, 2: False}
        self.continuous_modes = {1: False, 2: False}
        self.fire_timers = {1: None, 2: None}
        
        # 加载配置参数
        self.load_config()
        
        # 初始化GPIO控制
        self.init_gpio_control()
        
        # 初始化ROS通信
        self.init_ros_communication()
        
        rospy.loginfo("✅ Dual Laser Controller initialized")
        rospy.loginfo(f"🔫 Laser 1: GPIO {self.laser_configs[1]['gpio_pin']}")
        rospy.loginfo(f"🔫 Laser 2: GPIO {self.laser_configs[2]['gpio_pin']}")
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
        
        rospy.loginfo(f"⚙️ Active high: {self.active_high}, Duration: {self.fire_duration}s")
    
    def get_default_config(self):
        """获取默认配置"""
        return {
            'laser_control': {
                'gpio': {
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
            
            # 初始化所有激光头GPIO
            for laser_id, config in self.laser_configs.items():
                gpio_pin = config['gpio_pin']
                GPIO.setup(gpio_pin, GPIO.OUT)
                # 确保激光关闭
                self.set_gpio_pin(laser_id, False)
                rospy.loginfo(f"✅ {config['name']} GPIO initialized on pin {gpio_pin}")
            
        except Exception as e:
            rospy.logfatal(f"❌ Failed to initialize GPIO: {e}")
            rospy.signal_shutdown("GPIO initialization failed")
    
    def set_gpio_pin(self, laser_id, state):
        """设置GPIO引脚状态"""
        if laser_id not in self.laser_configs:
            rospy.logwarn(f"⚠️ Invalid laser ID: {laser_id}")
            return
            
        gpio_pin = self.laser_configs[laser_id]['gpio_pin']
        
        if self.active_high:
            gpio_state = GPIO.HIGH if state else GPIO.LOW
        else:
            gpio_state = GPIO.LOW if state else GPIO.HIGH
        
        GPIO.output(gpio_pin, gpio_state)
        rospy.logdebug(f"🔌 Laser {laser_id} GPIO pin {gpio_pin} set to {'HIGH' if gpio_state == GPIO.HIGH else 'LOW'}")
    
    def init_ros_communication(self):
        """初始化ROS通信"""
        # 订阅新的LaserControl消息
        self.laser_control_sub = rospy.Subscriber('/laser_control', LaserControl, 
                                                 self.laser_control_callback, queue_size=10)
        
        # 兼容性：保留旧的Bool消息支持
        self.fire_sub = rospy.Subscriber('/laser_fire', Bool, self.fire_callback, queue_size=1)
        self.continuous_sub = rospy.Subscriber('/laser_continuous', Bool, self.continuous_callback, queue_size=1)
        
        # 发布激光状态
        self.status_pub = rospy.Publisher('/laser_status', Bool, queue_size=10)
        
        rospy.loginfo("📡 ROS communication initialized")
        rospy.loginfo("📬 Subscribed to /laser_control (LaserControl)")
        rospy.loginfo("📬 Subscribed to /laser_fire, /laser_continuous (Bool - legacy)")
    
    def laser_control_callback(self, msg):
        """新的LaserControl消息回调"""
        laser_id = int(msg.laser_id)
        laser_state = bool(msg.laser_state)
        
        if laser_id not in self.laser_configs:
            rospy.logwarn(f"⚠️ Invalid laser ID received: {laser_id}")
            return
            
        laser_name = self.laser_configs[laser_id]['name']
        rospy.loginfo(f"🔫 {laser_name} command: {'ON' if laser_state else 'OFF'}")
        
        if laser_state:
            self.fire_laser(laser_id)
        else:
            self.stop_laser(laser_id)
    
    
    def continuous_callback(self, msg):
        """常亮模式控制回调（兼容性支持，控制激光1）"""
        laser_id = 1  # 默认控制激光1
        self.continuous_modes[laser_id] = msg.data
        
        if self.continuous_modes[laser_id]:
            # 启用常亮模式
            rospy.loginfo(f"🔆 Laser {laser_id} continuous mode ENABLED")
            self.cancel_fire_timer(laser_id)  # 取消任何定时器
            self.set_laser_state(laser_id, True)
        else:
            # 关闭常亮模式
            rospy.loginfo(f"🌙 Laser {laser_id} continuous mode DISABLED")
            self.set_laser_state(laser_id, False)
    
    def fire_callback(self, msg):
        """激光发射命令回调（兼容性支持，控制激光1）"""
        laser_id = 1  # 默认控制激光1
        
        # 常亮模式下忽略fire命令
        if self.continuous_modes[laser_id]:
            rospy.logdebug(f"🔆 Fire command ignored - Laser {laser_id} continuous mode active")
            return
            
        if msg.data:
            self.fire_laser(laser_id)
        else:
            self.stop_laser(laser_id)
    
    def set_laser_state(self, laser_id, state):
        """设置激光状态（内部方法）"""
        if laser_id not in self.laser_configs:
            rospy.logwarn(f"⚠️ Invalid laser ID: {laser_id}")
            return
            
        if state != self.laser_states[laser_id]:
            self.set_gpio_pin(laser_id, state)
            self.laser_states[laser_id] = state
            self.publish_status(laser_id)
            
            laser_name = self.laser_configs[laser_id]['name']
            if state:
                rospy.logwarn(f"🔥 {laser_name} ON")
            else:
                rospy.loginfo(f"💡 {laser_name} OFF")
    
    def cancel_fire_timer(self, laser_id):
        """取消发射定时器"""
        if laser_id in self.fire_timers and self.fire_timers[laser_id]:
            self.fire_timers[laser_id].cancel()
            self.fire_timers[laser_id] = None
    
    def fire_laser(self, laser_id):
        """发射激光（定时模式）"""
        if laser_id not in self.laser_configs:
            rospy.logwarn(f"⚠️ Invalid laser ID: {laser_id}")
            return
            
        # 常亮模式下不执行定时发射
        if self.continuous_modes[laser_id]:
            rospy.logdebug(f"🔆 Fire ignored - Laser {laser_id} continuous mode active")
            return
            
        if self.laser_states[laser_id]:
            rospy.logwarn(f"⚠️ Laser {laser_id} is already on")
            return
        
        try:
            laser_name = self.laser_configs[laser_id]['name']
            rospy.logwarn(f"🔥 {laser_name} FIRED! Duration: {self.fire_duration}s")
            
            # 启动激光
            self.set_laser_state(laser_id, True)
            
            # 设置自动关闭定时器
            self.fire_timers[laser_id] = threading.Timer(
                self.fire_duration, 
                lambda: self.auto_stop_laser(laser_id)
            )
            self.fire_timers[laser_id].start()
            
        except Exception as e:
            rospy.logerr(f"❌ Error firing laser {laser_id}: {e}")
            self.stop_laser(laser_id)
    
    def stop_laser(self, laser_id):
        """停止激光（定时模式）"""
        if laser_id not in self.laser_configs:
            rospy.logwarn(f"⚠️ Invalid laser ID: {laser_id}")
            return
            
        # 常亮模式下不执行停止
        if self.continuous_modes[laser_id]:
            rospy.logdebug(f"🔆 Stop ignored - Laser {laser_id} continuous mode active")
            return
            
        if not self.laser_states[laser_id]:
            return
        
        try:
            # 取消定时器
            self.cancel_fire_timer(laser_id)
            
            # 关闭激光
            self.set_laser_state(laser_id, False)
            
        except Exception as e:
            rospy.logerr(f"❌ Error stopping laser {laser_id}: {e}")
    
    def auto_stop_laser(self, laser_id):
        """自动停止激光（定时器回调）"""
        # 常亮模式下不自动停止
        if self.continuous_modes[laser_id]:
            rospy.logdebug(f"🔆 Auto-stop ignored - Laser {laser_id} continuous mode active")
            return
            
        laser_name = self.laser_configs[laser_id]['name']
        rospy.loginfo(f"⏰ Auto-stopping {laser_name} after timeout")
        self.stop_laser(laser_id)
    
    def publish_status(self, laser_id):
        """发布激光状态"""
        try:
            # 发布总状态（任意一个激光开启就为True）
            any_laser_on = any(self.laser_states.values())
            status_msg = Bool()
            status_msg.data = any_laser_on
            self.status_pub.publish(status_msg)
            
        except Exception as e:
            rospy.logwarn(f"⚠️ Error publishing status: {e}")
    
    def cleanup(self):
        """清理资源"""
        try:
            rospy.loginfo("🧹 Cleaning up...")
            
            # 取消所有定时器
            for laser_id in self.laser_configs.keys():
                self.cancel_fire_timer(laser_id)
            
            # 确保所有激光关闭
            for laser_id in self.laser_configs.keys():
                if self.laser_states[laser_id]:
                    self.set_gpio_pin(laser_id, False)
                    self.laser_states[laser_id] = False
            
            # 清理GPIO资源
            GPIO.cleanup()
            rospy.loginfo("✅ GPIO cleanup completed")
                
        except Exception as e:
            rospy.logerr(f"❌ Error during cleanup: {e}")
    
    def run(self):
        """主运行循环"""
        rospy.loginfo("🚀 Dual Laser Controller running...")
        
        try:
            rospy.spin()
        except KeyboardInterrupt:
            rospy.loginfo("🛑 Dual laser controller interrupted by user")
        finally:
            self.cleanup()


if __name__ == "__main__":
    try:
        controller = DualLaserController()
        controller.run()
    except rospy.ROSInterruptException:
        rospy.loginfo("🛑 Dual laser control node interrupted")
    except Exception as e:
        rospy.logerr(f"❌ Dual laser control node failed: {e}")
    finally:
        # 确保清理GPIO资源
        try:
            GPIO.cleanup()
        except:
            pass
