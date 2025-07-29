#!/usr/bin/env python


import rospy
import mavros
from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandBool, SetMode
from geometry_msgs.msg import Point

# Callback method for state subscriber
current_state = State()  # Reading the current state from mavros msgs


def state_callback(state):
    global current_state
    current_state = state


mavros.set_namespace()
local_position_publisher = rospy.Publisher(mavros.get_topic('setpoint_position', 'local'), PoseStamped, queue_size=10)
state_subscriber = rospy.Subscriber(mavros.get_topic('state'), State, state_callback)

arming_client = rospy.ServiceProxy(mavros.get_topic('cmd', 'arming'), CommandBool)
set_mode_client = rospy.ServiceProxy(mavros.get_topic('set_mode'), SetMode)

pose = PoseStamped()


def coordinates_xyz(data):
    print("Callback function")

    X = data.x
    Y = data.y
    Z = data.z
    print("X value: ", X)
    print("Y value: ", Y)
    print("Z value: ", Z)
    print("")
    # pose = PoseStamped()
    pose.pose.position.x = X
    pose.pose.position.y = Y
    pose.pose.position.z = Z
    # pose.header.stamp = rospy.Time.now()
    # local_position_publisher.publish(pose)


def position_control():
    print("Position control def")
    rospy.init_node('Offboard_node', anonymous=True)
    prev_state = current_state
    rate = rospy.Rate(20.0)

    # Sending a few points before start
    for i in range(100):
        if(rospy.is_shutdown()):
            break

        local_position_publisher.publish(pose)
        rate.sleep()

    # waiting for FCU connection
    while(not current_state.connected):
        if(rospy.is_shutdown()):
            break

        rate.sleep()

    last_request = rospy.get_rostime()

    while(not rospy.is_shutdown()):
        now = rospy.get_rostime()
        if(current_state.mode != "OFFBOARD" and (now - last_request) > rospy.Duration(5.0)):
            set_mode_client(base_mode=0, custom_mode="OFFBOARD")
            last_request = now

        else:
            if(not current_state.armed and (now - last_request > rospy.Duration(5.0))):
                arming_client(True)
                last_request = now

        # if current_state.armed:
            # rospy.loginfo("Drone ready to fly")
        if prev_state.armed != current_state.armed:
            rospy.loginfo("Vehicle armed: %r" % current_state.armed)
        if(prev_state.mode != current_state.mode and current_state.mode != "OFFBOARD"):
            rospy.loginfo("Current mode: %s" % current_state.mode)
            break

        prev_state = current_state
        # rospy.Subscriber("SOKA_DRONE", Point, coordinates_xyz)
        # pose.header.stamp = rospy.Time.now()
        local_position_publisher.publish(pose)
        rate.sleep()


if __name__ == '__main__':
    try:
        position_control()
    except rospy.ROSInterruptException:
        pass
