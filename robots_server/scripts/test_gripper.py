#****************************************************************************************
#
# Author : Aniruddha Shembekar, University of Southern California
#
#****************************************************************************************

import gripper_state
import sys
import time

gripper = gripper_state.GripperState();

gripper.activate_grippers()
c=raw_input("Press key to continue\n")	
gripper.open(2)
time.sleep(1)
gripper.close(2)
time.sleep(1)
gripper.open(1)
time.sleep(1)
gripper.close(1)
time.sleep(1)

# while (True):
# 	gripper.open(2)
# 	time.sleep(1)
# 	gripper.close(2)
# 	time.sleep(1)
# 	gripper.open(1)
# 	time.sleep(1)
# 	gripper.close(1)
# 	time.sleep(1)


sys.exit()