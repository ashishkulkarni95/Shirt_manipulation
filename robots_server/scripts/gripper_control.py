#****************************************************************************************
#
# Author : Aniruddha Shembekar, University of Southern California
#
#****************************************************************************************

#!/usr/bin/env python
import rospy
import os
import time
from std_msgs.msg import String
import gripper_state

try:
    gripper = gripper_state.GripperState()
except:
    print("gripper could not be connected")
    
gripper1_loc_data = "open"
gripper2_loc_data = "open"

def gripper_activation_signal(str):
    if (str.data=="START"):
        try:
            gripper.activate_grippers()
        except:
            print("gripper activation issue")

def gripper_state_signal(str):
    global gripper1_loc_data
    global gripper2_loc_data

    print("got callback for " + str.data)

    if (str.data=="gripper_1_open"):
        # print("opening the gripper one")
        gripper1_loc_data = "open"
        try:
            gripper.open(1)
        except:
            print("gripper 1 could not open")
        time.sleep(0.8)
    if (str.data=="gripper_1_close"):
        # print("closing the gripper one")
        gripper1_loc_data = "close"
        try:
            gripper.close(1)
        except:
            print("gripper 1 could not close")
        time.sleep(0.8)
    if (str.data=="gripper_2_open"):
        print("opening the gripper two")
        gripper2_loc_data = "open"
        try:
            gripper.open(2)
            print("gripper 2 opened")
        except:
            print("gripper 2 could not open")
        time.sleep(0.8)
    if (str.data=="gripper_2_close"):
        # print("closing the gripper two")
        gripper2_loc_data = "close"
        try:
            gripper.close(2)
        except:
            print("gripper 2 could not close")
        time.sleep(0.8)    
    
def gripper_talker_node():

    global gripper1_loc_data
    global gripper2_loc_data
    
    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('Gripper_Control')

    rospy.Subscriber("grippers_activation_data", String, gripper_activation_signal)
    rospy.Subscriber("gripper_state_val", String, gripper_state_signal)
    gripper1_loc_pub = rospy.Publisher('gripper_rob1_curr_loc', String, queue_size=100)
    gripper2_loc_pub = rospy.Publisher('gripper_rob2_curr_loc', String, queue_size=100)
    # gripper.activate_grippers()

    rate = rospy.Rate(10)
    # spin() simply keeps python from exiting until this node is stopped
    while not rospy.is_shutdown():
        # print(gripper1_loc_data)
        gripper1_loc_pub.publish(gripper1_loc_data)
        gripper2_loc_pub.publish(gripper2_loc_data)
        rate.sleep()

if __name__ == '__main__':
    # try:  
    #     os.system("sudo chmod 666 /dev/ttyACM0")
    #     os.system("sudo chmod 666 /dev/ttyUSB1")
    #     os.system("sudo chmod 666 /dev/ttyUSB3")
    # except:
    #     print("could not give admin access to the usb files...")
    gripper_talker_node()