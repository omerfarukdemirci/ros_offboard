#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import CommandBool
from mavros_msgs.msg import State
from mavros_msgs.msg import SetMode

current_state.mavros_msgs.State
def state_cb(msg):
    global current_state
    current_state = msg

    pose.geometry_msgs.PoseStamped
    pose.pose.setpoint_position.x=0
    pose.pose.setpoint_position.y=0
    pose.pose.setpoint_position.z=0


if __name__=='__main__':

    rospy.init_node("offb_node")
    state_sub = rospy.Subscriber("mavros/state",State, state_cb)
    local_pos_pub = rospy.Publisher("mavros/setpoint_position/local", PoseStamped,queue_size=10)
    rospy.spin()
    rate = rospy.Rate(20)

    while not rospy.is_shutdown():
       
       
        local_pos_pub.publish(pose)
    