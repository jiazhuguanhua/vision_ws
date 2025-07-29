"""
 * File: offb_node_multiwpt.py
"""

#! /usr/bin/env python


import rospy
from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import State
from mavros_msgs.msg import ExtendedState
from mavros_msgs.srv import CommandBool, CommandBoolRequest, SetMode, SetModeRequest
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import os
import math
import datetime

id = "" # Only used in XTDrone
auto_offb_and_arm = False # True: Automatically switch to OFFBOARD and arm the drone | False: manually RC switch to OFFBOARD and arm the drone
# 定义航路点数组（可根据需要修改）
wpts = [
    [0, 0, 0.5],
    [1, 0, 0.5]

]

current_state = State()
current_image = None
bridge = CvBridge()

def state_cb(msg):
    global current_state
    current_state = msg

def image_cb(msg):
    global current_image
    current_image = msg



current_pose = PoseStamped()
current_extended_state = ExtendedState()


def pose_cb(msg):
    global current_pose
    current_pose = msg

def extended_state_cb(msg):
    global current_extended_state
    current_extended_state = msg

if __name__ == "__main__":
    rospy.init_node("offb_node_py")


    state_sub = rospy.Subscriber(id + "/mavros/state", State, callback = state_cb)
    image_sub = rospy.Subscriber(id + "/usb_cam/image_raw", Image, callback=image_cb)

    pose_sub = rospy.Subscriber(id + "/mavros/local_position/pose", PoseStamped, callback=pose_cb)
    ext_state_sub = rospy.Subscriber(id + "/mavros/extended_state", ExtendedState, callback=extended_state_cb)

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

    wpt_idx = 0

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
                save_dir = "/home/winner/Desktop"
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

    if(auto_offb_and_arm == True):
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
                
                # 在航点拍照
                capture_image_at_waypoint(wpt_idx, wpts[wpt_idx])
                
                rospy.sleep(1.0)  # 等待1秒钟，确保到达
                if wpt_idx < len(wpts) - 1:
                    wpt_idx += 1
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
