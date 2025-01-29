#!/usr/bin/env python3


import rospy
from control_msgs.msg import FollowJointTrajectoryGoal, FollowJointTrajectoryAction
import actionlib
import sys
from typing import Literal

def server_not_up(action_name):
    rospy.logerr(f"Timed out waiting for '{action_name}' Joint Trajectory"
                         " Action Server to connect. Start the action server"
                         " before running example.")
    rospy.signal_shutdown("Timed out waiting for Action Server")
    sys.exit(1)
    

class BaxterActionClientBridge:
    def __init__(self):
        [rospy.Subscriber(f"/robot/limb/{limb}/follow_joint_trajectory_bridge", FollowJointTrajectoryGoal, self.arm_goal_callback, callback_args=limb) for limb in ["left", "right"]]
        self.left_arm_action_client = actionlib.SimpleActionClient(
            "robot/limb/left/follow_joint_trajectory",
            FollowJointTrajectoryAction,
        )
        self.right_arm_action_client = actionlib.SimpleActionClient(
            "robot/limb/right/follow_joint_trajectory",
            FollowJointTrajectoryAction,
        )
        server_up = self.left_arm_action_client.wait_for_server(timeout=rospy.Duration(5.0))
        if not server_up:
            server_not_up(action_name="robot/limb/left/follow_joint_trajectory")
        server_up = self.right_arm_action_client.wait_for_server(timeout=rospy.Duration(5.0))
        if not server_up:
            server_not_up(action_name="robot/limb/right/follow_joint_trajectory")
            
        self.clients = {
            "left": self.left_arm_action_client,
            "right": self.right_arm_action_client,
        }
            
        rospy.loginfo("BaxterActionClientBridge node is successfully created")
            
        
    def arm_goal_callback(self, goal_msg: FollowJointTrajectoryGoal, limb: Literal["left", "right"]):
        rospy.loginfo(f"[MOJE] BaxterActionClientBridge arm_goal_callback(), received goal action msg for limb: {limb}")
        # print("goal_msg:", goal_msg)
        goal_msg.trajectory.header.stamp = rospy.Time.now()
        client = self.clients[limb]
        client.send_goal(goal_msg)
        rospy.loginfo("[MOJE] BaxterActionClientBridge arm_goal_callback(), request successfully sent to the server") #request successfully sent, doesn't mean that server successfully received it

def main():
    rospy.init_node("baxter_action_client_bridge_node")
    baxter_action_client_bridge_node = BaxterActionClientBridge()
    rospy.spin()

if __name__ == "__main__":
    main()