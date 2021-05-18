#****************************************************************************************
#
# Author : Aniruddha Shembekar, University of Southern California
#
#****************************************************************************************

import gripper_comm
import arduino_comm
import time
import sys

time_gap_valve_and_gripper = 0.1

# arduino object
objarduino=arduino_comm.CommModule(0)
objarduino.arduinocomm()

# gripper object
objgripper=gripper_comm.GripperIO(3)
objgripper.set_speed(255)
objgripper.set_force(0)
objgripper.activate()

def gripper_1_open_with_valves():
    objarduino.valve_open(1)
    time.sleep(2*time_gap_valve_and_gripper)
    objarduino.valve_open(2)
    time.sleep(time_gap_valve_and_gripper)    
    objgripper.go_to(170)

def gripper_1_valves_close():
	objarduino.valve_close(1)
	objarduino.valve_close(2)
	time.sleep(time_gap_valve_and_gripper)

def gripper_1_close():
    objgripper.go_to(250)

c=raw_input("Press key to continue\n")	

try:
	for i in range(10):
		time.sleep(1)
		gripper_1_close()
		time.sleep(1)
		gripper_1_open_with_valves()
		time.sleep(1)
		gripper_1_valves_close()
		time.sleep(1)
		gripper_1_close()
except:
	time.sleep(0.5)
	gripper_1_valves_close()
	
sys.exit() 