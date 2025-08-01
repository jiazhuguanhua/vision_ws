#!/usr/bin/env python3
"""
 * File: 飞行控制核心节点
"""


import rospy
from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import State
from mavros_msgs.msg import ExtendedState
from mavros_msgs.srv import CommandBool, CommandBoolRequest, SetMode, SetModeRequest
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from waypoint_planner.msg import PointArray  # 添加航点数组消息类型
from std_msgs.msg import Empty, String
from animal_detect.msg import AnimalDetection  # 添加动物检测消息
import cv2
import os
import math
import datetime

id = "" # Only used in XTDrone
auto_offb_and_arm = False # True: Automatically switch to OFFBOARD and arm the drone | False: manually RC switch to OFFBOARD and arm the drone
# 定义预设的航路点数组（会被收到的wpt覆盖）
wpts = [
    [0, 0, 1.2],
    [0.5,0,1.2],
    [0.5, 0.5, 1.2],
    [0, 0.5, 1.2],
    [0, 0, 1.2]
]

current_state = State()
current_image = None
bridge = CvBridge()
visited_waypoints = set()  # 记录已访问过的航点
detection_active = False  # 动物检测状态
latest_detection = None  # 最新的检测结果

def state_cb(msg):
    global current_state
    current_state = msg

def image_cb(msg):
    global current_image
    current_image = msg

def animal_detection_callback(msg):
    """处理动物检测结果"""
    global latest_detection
    latest_detection = msg
    if msg.total_count > 0:
        animal_info = []
        for i, animal_type in enumerate(msg.animal_types):
            animal_info.append(f"{animal_type}({msg.counts[i]})")
        rospy.loginfo(f"🐾 检测到动物: {', '.join(animal_info)}, 总数: {msg.total_count}")
    else:
        rospy.loginfo("🔍 未检测到动物")



current_pose = PoseStamped()
current_extended_state = ExtendedState()


def pose_cb(msg):
    global current_pose
    current_pose = msg

def extended_state_cb(msg):
    global current_extended_state
    current_extended_state = msg

def waypoints_callback(msg):
    """处理来自航点规划的消息"""
    global wpts, wpt_idx
    # 清空当前航点列表
    wpts = []
    # 将PointArray消息转换为航点列表
    for point in msg.points:
        wpts.append([point.x, point.y, point.z])
    rospy.loginfo(f"收到新的航点数据: {len(wpts)}个航点")
    # 如果是首次接收航点，初始化索引
    if wpt_idx is None:
        wpt_idx = 0

def animal_detect_at_waypoint(wpt_idx, waypoint):
    """在航点执行动物检测"""
    global detection_active, latest_detection
    
    rospy.loginfo(f"🔍 在航点 {wpt_idx} 开始动物检测: [{waypoint[0]:.1f}, {waypoint[1]:.1f}, {waypoint[2]:.1f}]")
    
    # 启动动物检测
    detection_active = True
    start_detection_pub.publish(Empty())
    
    # 等待检测结果
    detection_timeout = 5.0  # 5秒超时
    start_time = rospy.Time.now()
    
    while (rospy.Time.now() - start_time).to_sec() < detection_timeout:
        if latest_detection is not None:
            # 检测完成，停止检测
            stop_detection_pub.publish(Empty())
            detection_active = False
            
            if latest_detection.total_count > 0:
                rospy.loginfo(f"✅ 航点 {wpt_idx} 动物检测完成，发现 {latest_detection.total_count} 只动物")
                # 激光控制由animal_detect包自动处理
            else:
                rospy.loginfo(f"✅ 航点 {wpt_idx} 动物检测完成，未发现动物")
            
            # 清空检测结果，准备下次检测
            latest_detection = None
            return True
        
        rospy.sleep(0.1)
    
    # 超时处理
    rospy.logwarn(f"⚠️ 航点 {wpt_idx} 动物检测超时")
    stop_detection_pub.publish(Empty())
    detection_active = False
    return False

if __name__ == "__main__":
    rospy.init_node("offb_node_py")
    
    # 初始化航点相关变量
    wpts = []
    wpt_idx = None


    state_sub = rospy.Subscriber(id + "/mavros/state", State, callback = state_cb)
    image_sub = rospy.Subscriber(id + "/usb_cam/image_raw", Image, callback=image_cb)
    pose_sub = rospy.Subscriber(id + "/mavros/local_position/pose", PoseStamped, callback=pose_cb)
    ext_state_sub = rospy.Subscriber(id + "/mavros/extended_state", ExtendedState, callback=extended_state_cb)
    
    # 订阅航点规划话题
    waypoints_sub = rospy.Subscriber("/waypoints", PointArray, callback=waypoints_callback)
    
    # 订阅动物检测结果
    animal_detection_sub = rospy.Subscriber("/animal_detection_result", AnimalDetection, callback=animal_detection_callback)
    
    # 发布动物检测控制命令
    start_detection_pub = rospy.Publisher("/start_detection", Empty, queue_size=1)
    stop_detection_pub = rospy.Publisher("/stop_detection", Empty, queue_size=1)
    
    rospy.loginfo("等待航点规划数据...")

    local_pos_pub = rospy.Publisher(id + "/mavros/setpoint_position/local", PoseStamped, queue_size=10)

    rospy.wait_for_service(id + "/mavros/cmd/arming")
    arming_client = rospy.ServiceProxy(id + "/mavros/cmd/arming", CommandBool)    

    rospy.wait_for_service(id + "/mavros/set_mode")
    set_mode_client = rospy.ServiceProxy(id + "/mavros/set_mode", SetMode)
    
    # Wait for camera capture service (comment out for now)
    # rospy.loginfo("Waiting for camera capture service...")
    # rospy.wait_for_service("capture_image")
    # capture_client = rospy.ServiceProxy("capture_image", CaptureImage)


    # Setpoint publishing MUST be faster than 2Hz
    rate = rospy.Rate(20)

    # Wait for Flight Controller connection
    while(not rospy.is_shutdown() and not current_state.connected):
        rate.sleep()

    # 阻塞等待航点数据到来
    while len(wpts) == 0 and not rospy.is_shutdown():
        rospy.loginfo_throttle(1.0, "等待航点规划数据...")
        rate.sleep()

    if not rospy.is_shutdown():
        rospy.loginfo(f"成功接收到{len(wpts)}个航点")
    wpt_idx = 0

    # 初始化第一个航点
    pose = PoseStamped()
    pose.pose.position.x = wpts[wpt_idx][0]
    pose.pose.position.y = wpts[wpt_idx][1]
    pose.pose.position.z = wpts[wpt_idx][2]


    # Send a few setpoints before starting
    for i in range(100):   
        if(rospy.is_shutdown()):
            break
        local_pos_pub.publish(pose)
        rate.sleep()

    set_mode_req = SetModeRequest()
    set_mode_req.custom_mode = 'OFFBOARD'

    arm_cmd = CommandBoolRequest()
    arm_cmd.value = True

    last_req = rospy.Time.now()

    def dist(p1, p2):
        return math.sqrt(
            (p1.x - p2[0])**2 +
            (p1.y - p2[1])**2 +
            (p1.z - p2[2])**2
        )

    def capture_image_at_waypoint(wpt_idx, waypoint):
        """在航点拍照并保存"""
        global current_image
        if current_image is not None:
            try:
                # 将ROS图像消息转换为OpenCV图像
                cv_image = bridge.imgmsg_to_cv2(current_image, "bgr8")
                
                # 创建保存目录
                save_dir = "/home/micoair/Desktop"
                if not os.path.exists(save_dir):
                    os.makedirs(save_dir)
                
                # 生成文件名（包含时间戳）
                timestamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
                filename = f"waypoint_{wpt_idx}_{waypoint[0]:.1f}_{waypoint[1]:.1f}_{waypoint[2]:.1f}_{timestamp}.jpg"
                filepath = os.path.join(save_dir, filename)
                
                # 保存图像
                cv2.imwrite(filepath, cv_image)
                rospy.loginfo(f"照片已保存: {filepath}")
                return True
            except Exception as e:
                rospy.logerr(f"拍照失败: {e}")
                return False
        else:
            rospy.logwarn("没有接收到摄像头图像")
            return False

    if(auto_offb_and_arm == True): #自动切offb和arm
        rospy.sleep(3.0)
        if(set_mode_client.call(set_mode_req).mode_sent == True):
            rospy.loginfo("OFFBOARD enabled")
        
        rospy.sleep(0.5)

        if(arming_client.call(arm_cmd).success == True):
            rospy.loginfo("Vehicle armed")

    while(not rospy.is_shutdown()):
        if current_state.armed == True:
            # Update wpt
            if dist(current_pose.pose.position, wpts[wpt_idx]) < 0.1:
                rospy.loginfo(f"到达航点: {wpts[wpt_idx]}")
                
                #当前航点任务载荷
                #====================
                # 在航点拍照
                capture_image_at_waypoint(wpt_idx, wpts[wpt_idx])
                
                # 检查是否为新航点，如果是则进行动物检测
                if wpt_idx not in visited_waypoints:
                    visited_waypoints.add(wpt_idx)
                    animal_detect_at_waypoint(wpt_idx, wpts[wpt_idx])
                
                #====================
                
                rospy.sleep(1.0)  # 等待1秒钟，确保到达
                if wpt_idx < len(wpts) - 1:
                    wpt_idx += 1
                    #更新next航路点
                    pose.pose.position.x = wpts[wpt_idx][0]
                    pose.pose.position.y = wpts[wpt_idx][1]
                    pose.pose.position.z = wpts[wpt_idx][2]
                    rospy.loginfo(f"前往下一个航点: {wpts[wpt_idx]}")
                else: #Switch to landing mode
                    rospy.loginfo("已到达最后一个航点")
                    set_mode_req.custom_mode = 'AUTO.LAND'
                    if(set_mode_client.call(set_mode_req).mode_sent == True):
                        rospy.loginfo("AUTO.LAND enabled")
        
        if current_extended_state.landed_state == 1 and current_state.mode == "AUTO.LAND":
            rospy.loginfo("Landed")
            arm_cmd.value = False
            if(arming_client.call(arm_cmd).success == True):
                rospy.loginfo("Vehicle disarmed by program")
            rospy.signal_shutdown("任务完成，程序退出")

        local_pos_pub.publish(pose)
        rate.sleep()
