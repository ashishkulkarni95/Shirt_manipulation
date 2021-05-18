#****************************************************************************************
#
# Author : Aniruddha Shembekar, University of Southern California
#
#****************************************************************************************

import socket
import time
import sys

class epsonComm():

    def __init__(self, ip_addr, port):
        self.IP_addr = ip_addr
        self.Port = port
        return

    def establishComm(self):
        # global HOST
        HOST = self.IP_addr
        # global PORT1
        PORT1=self.Port
        # global s1
        s1=socket.socket() 
        s1 = socket.socket( socket.AF_INET, socket.SOCK_STREAM )
        s1.settimeout(30)
        s1.bind( (HOST, PORT1) )
        s1.listen(1)
        # global comm, addr1
        self.comm, self.addr1 = s1.accept()
        print ("Communication Established to S5")
        return True

    def sendString(self, msg):
        print("sending :  " + msg)
        try:
            msg=msg+"\r\n"
            self.comm.sendall(str.encode(msg))
        except:
            print ("Sending Failed")
            sys.exit()

        time.sleep(0.05)

    def receiveString(self):
        try:
            msg=""
            while msg== "":
                msg=self.comm.recv(1024)            
        except:
            print ("Receiving Failed")
            sys.exit()
        return msg.rstrip()
        time.sleep(0.05)
