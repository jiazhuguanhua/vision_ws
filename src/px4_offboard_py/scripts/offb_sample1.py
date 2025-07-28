#!/usr/bin/env python

import rospy
import mavros
from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandBool, SetMode
from geometry_msgs.msg import Point

current_state = State()

def state_callback(state):
    global current_state
    current_state = state

mavros.set_namespace()
des_pos = PoseStamped()
state_sub = rospy.Subscriber('mavros/state', State, callback = state_callback)
position_pub = rospy.Publisher('mavros/setpoint_position/local', PoseStamped, queue_size = 10)
arming_drone = rospy.ServiceProxy('/mavros/cmd/arming', CommandBool)
setting_mode = rospy.ServiceProxy('mavros/set_mode', SetMode)

def offb_control_sample():
    print("Init Node: Offboard_node")
    rospy.init_node('Offboard_node', anonymous=True)
    prev_state = current_state
    rate = rospy.Rate(20.0)

    des_pos.pose.position.x = 0
    des_pos.pose.position.y = 0
    des_pos.pose.position.z = 1

    # waiting for FCU connection
    while(not rospy.is_shutdown() and not current_state.connected):
        print("Waiting for FCU connection")
        rate.sleep()

    print("FCU connected")
    # Sending a few points before start
    for i in range(60):
        if(rospy.is_shutdown()):
            break

        position_pub.publish(des_pos)
        rate.sleep()
 
    print("Switching to offboard Mode")
    setting_mode(base_mode=0, custom_mode="OFFBOARD")

    last_request = rospy.get_rostime()
    while((rospy.get_rostime() - last_request) < rospy.Duration(2.0)):
        position_pub.publish(des_pos)
        rate.sleep()

    print("Arming the drone on offboard Mode")
    if current_state.mode == "OFFBOARD":
        arming_drone(True)

    print("Take off to Point 1")
    last_request = rospy.get_rostime()
    while((rospy.get_rostime() - last_request) < rospy.Duration(8.0)):
        position_pub.publish(des_pos)
        rate.sleep()

    print("Fly to Point 2")
    des_pos.pose.position.x = 0
    des_pos.pose.position.y = 0.5
    des_pos.pose.position.z = 1
    last_request = rospy.get_rostime()
    while((rospy.get_rostime() - last_request) < rospy.Duration(2.0)):
        position_pub.publish(des_pos)
        rate.sleep()

    print("Fly to Point 3")
    des_pos.pose.position.x = 0.5
    des_pos.pose.position.y = 0.5
    des_pos.pose.position.z = 1
    last_request = rospy.get_rostime()
    while((rospy.get_rostime() - last_request) < rospy.Duration(2.0)):
        position_pub.publish(des_pos)
        rate.sleep()

    print("Fly to Point 4")
    des_pos.pose.position.x = 0.5
    des_pos.pose.position.y = 0
    des_pos.pose.position.z = 1
    last_request = rospy.get_rostime()
    while((rospy.get_rostime() - last_request) < rospy.Duration(2.0)):
        position_pub.publish(des_pos)
        rate.sleep()

    print("Fly back to Point 1")
    des_pos.pose.position.x = 0
    des_pos.pose.position.y = 0
    des_pos.pose.position.z = 1
    last_request = rospy.get_rostime()
    while((rospy.get_rostime() - last_request) < rospy.Duration(3.0)):
        position_pub.publish(des_pos)
        rate.sleep()

    print("Vehicle Landing")
    print("Mission End")

if __name__ == '__main__':
    try:
        offb_control_sample()
    except rospy.ROSInterruptException:
        pass
