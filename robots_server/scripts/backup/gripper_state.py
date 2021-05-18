#****************************************************************************************
#
# Author : Aniruddha Shembekar, University of Southern California
#
#****************************************************************************************

import serial 
import time
import sys
import gripper_comm
import arduino_comm

# to give admin access to usb ports - 
# sudo chmod 666 /dev/ttyACM0
# sudo chmod 666 /dev/ttyUSB1
# sudo chmod 666 /dev/ttyUSB3

class GripperState():
	def __init__(self,arduino_comm_num=0,gripper1_comm_num=3,gripper2_comm_num=1):
		self.time_gap_valve_and_gripper = 0.1
		# arduino object
		self.objarduino=arduino_comm.CommModule(arduino_comm_num)
		self.objarduino.arduinocomm()
		# gripper 1 object
		print("gripper1_comm_num :" + str(gripper1_comm_num))
		self.objgripper1=gripper_comm.GripperIO(gripper1_comm_num)
		self.objgripper1.set_speed(255)
		self.objgripper1.set_force(0)
		# gripper 2 object
		print("gripper2_comm_num :"+str(gripper2_comm_num))
		self.objgripper2=gripper_comm.GripperIO(gripper2_comm_num)
		self.objgripper2.set_speed(255)
		self.objgripper2.set_force(0)
		self.open_pos_g1 = 170
		self.open_pos_g2 = 170
		self.close_pos_g1 = 255
		self.close_pos_g2 = 255

	def activate_grippers(self):
		try:
			self.objgripper1.activate()
		except:
			print("gripper 1 could not be activated")
		try:
			self.objgripper2.activate()
		except:
			print("gripper 2 could not be activated")

	def set_open_pos(self,gripper_num,val):
		if (gripper_num==1):
			self.open_pos_g1 = val
		if (gripper_num==2):
			self.open_pos_g2 = val

	def set_close_pos(self,gripper_num,val):
		if (gripper_num==1):
			self.close_pos_g1 = val	
		if (gripper_num==2):
			self.close_pos_g2 = val	

	def valve_open(self,valve1,valve2):
		self.objarduino.valve_open(valve1)
		time.sleep(2*self.time_gap_valve_and_gripper)
		self.objarduino.valve_open(valve2)
		time.sleep(self.time_gap_valve_and_gripper)    
			
	def valve_close(self,valve1,valve2):
		time.sleep(0.1)
		self.objarduino.valve_close(valve1)
		self.objarduino.valve_close(valve2)
		time.sleep(self.time_gap_valve_and_gripper)

	def open(self,gripper_num):
		if (gripper_num==1):
			self.valve_open(1,2)
			self.objgripper1.go_to(self.open_pos_g1)
			self.valve_close(1,2)
		if (gripper_num==2):
			# print("reached here from gripper state code")
			self.valve_open(3,4)
			self.objgripper2.go_to(self.open_pos_g2)
			self.valve_close(3,4)
			
	def close(self,gripper_num):
		if (gripper_num==1):
		    self.objgripper1.go_to(self.close_pos_g1)
		if (gripper_num==2):
			self.objgripper2.go_to(self.close_pos_g2)
		    
