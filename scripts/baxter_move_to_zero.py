#!/usr/bin/env python3

import rospy
from std_msgs.msg import (
    UInt16,
)
import baxter_interface

from baxter_interface import CHECK_VERSION

import threading

class MoveToZero:
    def __init__(self):
        self.rate_publisher = rospy.Publisher('robot/joint_state_publish_rate',
                                            UInt16, queue_size=10)
        self.left_arm = baxter_interface.limb.Limb("left")
        self.right_arm = baxter_interface.limb.Limb("right")

        self.PUB_RATE = 100

        print("Getting robot state... ")
        self.rs = baxter_interface.RobotEnable(CHECK_VERSION)
        self.init_state = self.rs.state().enabled
        print("Enabling robot... ")
        self.rs.enable()

        self.rate_publisher.publish(self.PUB_RATE)
        self.rate = rospy.Rate(self.PUB_RATE)
        # rospy.loginfo(f"Right arm current joint values: {self.right_arm.joint_angles()}")
        
        # self.move_to_neutral_pose()
        # self.move_to_zero_joint_values()
        self.move_to_zero_joint_values_multithreading() #with multithreading it successfully moves both arms simultaneously, without it - it goes one by one arm
        
        
    def move_arm(self, arm):
        angles = dict(zip(arm.joint_names(),
                            [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]))
        rospy.loginfo(f"Moving {arm.name} arm to all 0 angles...")
        # return self.right_arm.move_to_joint_positions(angles, timeout)
        arm.move_to_joint_positions(angles, timeout=15)
        rospy.loginfo(f"Done Moving {arm.name} arm to all 0 angles")
        
    def move_to_zero_joint_values_multithreading(self):
        left_arm_thread = threading.Thread(target=self.move_arm, args=(self.left_arm, ))
        right_arm_thread = threading.Thread(target=self.move_arm, args=(self.right_arm, ))
        
        left_arm_thread.start()
        right_arm_thread.start()
        rospy.loginfo("Started both threads")
        
        left_arm_thread.join()
        right_arm_thread.join()
        rospy.loginfo("Both threads finished")
        
        
    def move_to_zero_joint_values(self):
        def get_angels(arm):
            angles = dict(zip(arm.joint_names(),
                            [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]))
            return angles
        
        timeout = 15
        rospy.loginfo("Moving right arm to all 0 angles...")
        # return self.right_arm.move_to_joint_positions(angles, timeout)
        self.right_arm.move_to_joint_positions(get_angels(self.right_arm), timeout)
        rospy.loginfo("Done Moving right arm to all 0 angles")
        rospy.loginfo(f"Right arm current joint values: {self.right_arm.joint_angles()}")
        
        rospy.loginfo("Moving left arm to all 0 angles...")
        self.left_arm.move_to_joint_positions(get_angels(self.left_arm), timeout)
        rospy.loginfo("Done Moving left arm to all 0 angles")
        rospy.loginfo(f"Left arm current joint values: {self.left_arm.joint_angles()}")
    
    
    def move_to_neutral_pose(self):
        rospy.loginfo("Moving right limb to neutral pose...")
        self.right_arm.move_to_neutral()
        rospy.loginfo("Done moving right limb to neutral pose")
        
        rospy.loginfo("Moving left limb to neutral pose...")
        self.left_arm.move_to_neutral()
        rospy.loginfo("Done moving left limb to neutral pose")
        
    def run(self):
        while not rospy.is_shutdown():
            
            self.rate.sleep()


def main():
    rospy.init_node("baxter_move_to_zero_node")
    rospy.loginfo("Created baxter_move_to_zero_node")
    baxter_move_to_zero = MoveToZero()
    baxter_move_to_zero.run()
    
    
    
if __name__ == "__main__":
    main()





