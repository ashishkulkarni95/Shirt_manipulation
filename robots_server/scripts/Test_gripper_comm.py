#****************************************************************************************
#
# Author : Aniruddha Shembekar, University of Southern California
#
#****************************************************************************************

import gripperIO
import time
import sys

# gripper on iiwa116
objgripper=gripper_comm.gripperIO(3)

# gripper on iiwa118
# objgripper=gripper_comm.gripperIO(1)

objgripper.set_speed(255)
objgripper.set_force(255)
objgripper.activate()

c=raw_input("Press key to continue\n")	


for i in range(1):
	objgripper.go_to(255)

	# time.sleep(1)
	
	objgripper.go_to(125)

	# time.sleep(1)

	objgripper.go_to(0)

	

sys.exit() 