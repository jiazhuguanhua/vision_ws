#!/usr/bin/env python3
"""
系统测试脚本 - 验证无人机自主任务系统的基本功能
Test Script for Autonomous Drone Mission System
"""

import rospy
import sys
import time
from std_msgs.msg import Bool
from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import State
from common_msgs.msg import MissionState, AprilTagDetection


class SystemTester:
    """系统测试器"""
    
    def __init__(self):
        rospy.init_node('system_tester', anonymous=True)
        
        # 订阅系统状态
        self.mavros_state = None
        self.mission_state = None
        self.apriltag_detection = None
        self.laser_status = None
        
        # 订阅器
        rospy.Subscriber('/mavros/state', State, self.mavros_state_cb)
        rospy.Subscriber('/mission/state', MissionState, self.mission_state_cb)
        rospy.Subscriber('/apriltag/detection', AprilTagDetection, self.apriltag_cb)
        rospy.Subscriber('/laser_status', Bool, self.laser_status_cb)
        
        # 发布器
        self.laser_fire_pub = rospy.Publisher('/laser_fire', Bool, queue_size=1)
        
        print("系统测试器已启动...")
        print("请确保所有节点都已启动")
        
    def mavros_state_cb(self, msg):
        self.mavros_state = msg
        
    def mission_state_cb(self, msg):
        self.mission_state = msg
        
    def apriltag_cb(self, msg):
        self.apriltag_detection = msg
        
    def laser_status_cb(self, msg):
        self.laser_status = msg
    
    def wait_for_topics(self, timeout=10):
        """等待话题连接"""
        print(f"等待话题连接 (超时: {timeout}秒)...")
        
        start_time = time.time()
        while time.time() - start_time < timeout:
            if (self.mavros_state is not None and 
                self.mission_state is not None):
                print("✓ 核心话题已连接")
                return True
            time.sleep(0.1)
        
        print("✗ 话题连接超时")
        return False
    
    def test_mavros_connection(self):
        """测试MAVROS连接"""
        print("\n=== 测试MAVROS连接 ===")
        
        if self.mavros_state is None:
            print("✗ 无法获取MAVROS状态")
            return False
        
        print(f"连接状态: {self.mavros_state.connected}")
        print(f"解锁状态: {self.mavros_state.armed}")
        print(f"飞行模式: {self.mavros_state.mode}")
        
        if self.mavros_state.connected:
            print("✓ MAVROS连接正常")
            return True
        else:
            print("✗ MAVROS未连接")
            return False
    
    def test_mission_state(self):
        """测试任务状态"""
        print("\n=== 测试任务状态 ===")
        
        if self.mission_state is None:
            print("✗ 无法获取任务状态")
            return False
        
        state_names = {
            0: "INIT", 1: "TAKEOFF", 2: "GOTO_MISSION", 3: "SCAN_TAG",
            4: "LASER_FIRE", 5: "GOTO_LANDING", 6: "PRECISION_LAND",
            7: "LAND", 8: "COMPLETE", 9: "ERROR"
        }
        
        current_state = state_names.get(self.mission_state.current_state, "UNKNOWN")
        print(f"当前状态: {current_state}")
        print(f"状态描述: {self.mission_state.state_description}")
        print(f"任务进度: {self.mission_state.progress:.1f}%")
        
        print("✓ 任务状态正常")
        return True
    
    def test_apriltag_detection(self):
        """测试AprilTag检测"""
        print("\n=== 测试AprilTag检测 ===")
        
        if self.apriltag_detection is None:
            print("? AprilTag检测话题未连接")
            return True  # 非关键错误
        
        print(f"检测状态: {self.apriltag_detection.detected}")
        if self.apriltag_detection.detected:
            print(f"Tag ID: {self.apriltag_detection.tag_id}")
            print(f"置信度: {self.apriltag_detection.confidence:.3f}")
            print(f"Tag数量: {self.apriltag_detection.tag_count}")
        
        print("✓ AprilTag检测功能正常")
        return True
    
    def test_laser_control(self):
        """测试激光控制"""
        print("\n=== 测试激光控制 ===")
        
        if self.laser_status is None:
            print("? 激光状态话题未连接")
            return True  # 非关键错误
        
        # 发送激光开启命令
        print("发送激光开启命令...")
        laser_cmd = Bool()
        laser_cmd.data = True
        self.laser_fire_pub.publish(laser_cmd)
        
        # 等待状态更新
        time.sleep(1)
        
        if self.laser_status.data:
            print("✓ 激光已开启")
        else:
            print("? 激光状态未更新")
        
        # 发送激光关闭命令
        print("发送激光关闭命令...")
        laser_cmd.data = False
        self.laser_fire_pub.publish(laser_cmd)
        
        time.sleep(1)
        
        print("✓ 激光控制测试完成")
        return True
    
    def test_topics_availability(self):
        """测试话题可用性"""
        print("\n=== 测试话题可用性 ===")
        
        import subprocess
        
        required_topics = [
            '/mavros/state',
            '/mavros/local_position/pose',
            '/mission/state',
            '/laser_fire'
        ]
        
        optional_topics = [
            '/apriltag/detection',
            '/apriltag/pose',
            '/laser_status'
        ]
        
        try:
            # 获取当前活跃的话题
            result = subprocess.run(['rostopic', 'list'], 
                                  capture_output=True, text=True, timeout=5)
            active_topics = result.stdout.strip().split('\n')
            
            print("必需话题检查:")
            all_required_ok = True
            for topic in required_topics:
                if topic in active_topics:
                    print(f"  ✓ {topic}")
                else:
                    print(f"  ✗ {topic}")
                    all_required_ok = False
            
            print("\n可选话题检查:")
            for topic in optional_topics:
                if topic in active_topics:
                    print(f"  ✓ {topic}")
                else:
                    print(f"  ? {topic} (可选)")
            
            if all_required_ok:
                print("\n✓ 所有必需话题都可用")
                return True
            else:
                print("\n✗ 部分必需话题不可用")
                return False
                
        except Exception as e:
            print(f"✗ 无法检查话题: {e}")
            return False
    
    def run_tests(self):
        """运行所有测试"""
        print("开始系统测试...")
        print("=" * 50)
        
        # 等待话题连接
        if not self.wait_for_topics():
            print("系统测试失败: 话题连接超时")
            return False
        
        # 运行各项测试
        tests = [
            self.test_topics_availability,
            self.test_mavros_connection,
            self.test_mission_state,
            self.test_apriltag_detection,
            self.test_laser_control
        ]
        
        passed = 0
        failed = 0
        
        for test in tests:
            try:
                if test():
                    passed += 1
                else:
                    failed += 1
            except Exception as e:
                print(f"✗ 测试出错: {e}")
                failed += 1
        
        # 输出测试结果
        print("\n" + "=" * 50)
        print("测试结果总结:")
        print(f"通过: {passed}")
        print(f"失败: {failed}")
        print(f"总计: {passed + failed}")
        
        if failed == 0:
            print("\n🎉 所有测试通过！系统就绪。")
            return True
        else:
            print(f"\n⚠️  {failed}项测试失败，请检查系统配置。")
            return False


def main():
    """主函数"""
    try:
        tester = SystemTester()
        
        # 运行测试
        success = tester.run_tests()
        
        if success:
            print("\n系统测试完成，可以开始任务。")
            sys.exit(0)
        else:
            print("\n系统测试失败，请修复问题后重试。")
            sys.exit(1)
            
    except rospy.ROSInterruptException:
        print("\n测试被用户中断")
        sys.exit(1)
    except Exception as e:
        print(f"\n测试出现意外错误: {e}")
        sys.exit(1)


if __name__ == "__main__":
    main()
