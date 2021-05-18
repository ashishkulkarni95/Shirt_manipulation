#****************************************************************************************
#
# Author : Aniruddha Shembekar, University of Southern California
#
#****************************************************************************************

import arduino_comm
import time
import sys

objarduino=arduino_comm.CommModule(0)

objarduino.arduinocomm()

c=raw_input("Press key to continue\n")	

# time.sleep(0.2)

# objarduino.valve3_open()
# time.sleep(3)
# objarduino.valve3_close()

for i in range(1):
	time.sleep(1)

	objarduino.valve_open(1)
	time.sleep(2)
	objarduino.valve_close(1)
	time.sleep(2)
	objarduino.valve_open(2)
	time.sleep(2)
	objarduino.valve_close(2)
	time.sleep(2)
	objarduino.valve_open(3)
	time.sleep(2)
	objarduino.valve_close(3)
	time.sleep(2)
	objarduino.valve_open(4)
	time.sleep(2)
	objarduino.valve_close(4)
	time.sleep(2)
	objarduino.valve_open(1)
	objarduino.valve_open(2)
	objarduino.valve_open(3)
	objarduino.valve_open(4)
	time.sleep(2)
	objarduino.valve_close(1)
	objarduino.valve_close(2)
	objarduino.valve_close(3)
	objarduino.valve_close(4)

sys.exit() 