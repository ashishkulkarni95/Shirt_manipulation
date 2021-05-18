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
from gen_utilities.srv import StringPasser, StringPasserRequest, StringPasserResponse
import epson_comm

global s5

try:
    s5 = epson_comm.epsonComm('192.168.10.11', 20811)
except:
    print('Can not find the class: epsonComm')
    

def getCommState(str):
    print(str)
    if ((str.data=="START") or (str.data=="START_DRAPE_ONLY")):
        try:
            s5.establishComm()
        except:
            print("Connection could not be established with Epson s5")
    
    if (str.data=='STOP'):
        s5.sendString('close')
        reply = s5.receiveString()

def motion_passer_callback(req):
    s5.sendString(req.data_in)
    reply = s5.receiveString()
    return StringPasserResponse(reply)

    
def epson_comm_node():
    
    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('Epson_Motion_Comm')

    rospy.Subscriber('rob_comm_state', String, getCommState)
    rospy.Service('epson_motion_loc_pub', StringPasser, motion_passer_callback)
    
    rate = rospy.Rate(100)
    # spin() simply keeps python from exiting until this node is stopped
    while not rospy.is_shutdown():
        rate.sleep()

if __name__ == '__main__':
    epson_comm_node()