#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import JointState

topic_name = "/robot/joint_states"

pub = rospy.Publisher("/joint_states", JointState, queue_size=50)

def joint_state_callback(data):
    pub.publish(data)

rospy.init_node('sub_pub_joint_state')
print("[MOJE] created sub_pub_joint_state,, publishing '/robot/joint_states' to '/joint_states'")
rospy.Subscriber(topic_name, JointState, joint_state_callback, queue_size=1, buff_size=3*1024)
rospy.spin()
