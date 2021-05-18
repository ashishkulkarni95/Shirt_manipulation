#!/usr/bin/env python

from gen_utilities.srv import GripperMotion, GripperMotionResponse
import gripper_comm
import time
import sys
import rospy

global objgripper_BLUE

def gripper_motion(req):
    global objgripper_BLUE
    rospy.loginfo(rospy.get_caller_id() + "I received %s", req.action)
    rospy.loginfo("blue has received this")
    if req.action == "open":
        objgripper_BLUE.go_to(50)
        rospy.loginfo("Opened blue")
    elif req.action == "close":
        objgripper_BLUE.go_to(250)
        rospy.loginfo("Closed blue")
    elif req.action == "troubleshoot":
        objgripper_BLUE.activate()
    rospy.loginfo("None of the actions are performed")
    return GripperMotionResponse("Done!")


def initialize_grippers():
    rospy.init_node('gripper_blue_motion')
    s = rospy.Service('gripper_blue_service', GripperMotion, gripper_motion)
    rospy.spin()

if __name__ == "__main__":
    objgripper_BLUE = gripper_comm.GripperIO(1)
    objgripper_BLUE.set_speed(150)
    objgripper_BLUE.set_force(245)
    objgripper_BLUE.activate()
    initialize_grippers()
