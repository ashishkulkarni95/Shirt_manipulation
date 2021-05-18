#!/usr/bin/env python

from gen_utilities.srv import GripperMotion, GripperMotionResponse
import gripper_comm
import time
import sys
import rospy

global objgripper_GREEN

def gripper_motion(req):
    global objgripper_GREEN
    rospy.loginfo(rospy.get_caller_id() + "I received %s", req.action)
    rospy.loginfo("green has received this")
    if req.action == "open":
        objgripper_GREEN.go_to(50)
        rospy.loginfo("Opened green")
    elif req.action == "close":
        objgripper_GREEN.go_to(250)
        rospy.loginfo("Closed green")
    elif req.action == "troubleshoot":
        objgripper_GREEN.activate()
    rospy.loginfo("None of the actions are performed")
    return GripperMotionResponse("Done!")


def initialize_grippers():
    rospy.init_node('gripper_green_motion')
    s = rospy.Service('gripper_green_service', GripperMotion, gripper_motion)
    rospy.spin()

if __name__ == "__main__":
    objgripper_GREEN = gripper_comm.GripperIO(3)
    objgripper_GREEN.set_speed(150)
    objgripper_GREEN.set_force(245)
    objgripper_GREEN.activate()
    initialize_grippers()
