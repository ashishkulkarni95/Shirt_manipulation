#****************************************************************************************
#
# Author : Aniruddha Shembekar, University of Southern California
#
#****************************************************************************************

import serial 
import time
import sys

class CommModule():
    def __init__(self, _port):
        
        # for windows
        # comm_port = "COM" + str(_port)
        
        # for linux
        comm_port = "/dev/ttyACM" + str(_port)
        
        # connection to the arduino serial port
        self.ser = serial.Serial(comm_port, 19200, timeout=0.1)
        return

    def arduinocomm(self):
        # establishing connection
        print ('Establishing conenction to Arduino...')
        connect = "0"
        # print (connect.encode())
        # self.ser.write('1'.encode())
        while connect != '2':
            self.ser.write('1'.encode())
            connect = self.ser.readline()[:-2]
            # print (connect.decode())
            connect = connect.decode()
        print ('connection established!')
        return True

    def valve_open(self,num):
        try:
            if (num==1):
                # print ('Opening valve 1...')
                self.ser.write('a'.encode())
            elif (num==2):
                # print ('Opening valve 2...')
                self.ser.write('c'.encode())
            elif (num==3):
                # print ('Opening valve 3...')
                self.ser.write('e'.encode())
                # print ("reached here e from arduino comm")
            elif (num==4):
                # print ('Opening valve 4...')
                self.ser.write('g'.encode())
                # print ("reached here g from arduino comm")
        except:
            print ('Valve did not open...')
            # sys.exit()

    def valve_close(self,num):
        try:
            if (num==1):
                # print ('Closing valve 1...')
                self.ser.write('b'.encode())
            elif (num==2):
                # print ('Closing valve 2...')
                self.ser.write('d'.encode())
            elif (num==3):
                # print ('Closing valve 3...')
                self.ser.write('f'.encode())
            elif (num==4):
                # print ('Closing valve 4...')
                self.ser.write('h'.encode())
        except:
            print ('valve did not close...')
