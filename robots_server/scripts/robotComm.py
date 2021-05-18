#****************************************************************************************
#
# Author : Aniruddha Shembekar, University of Southern California
#
#****************************************************************************************

import socket

class RobotCommunication:

	# Initialize robot class with host address string and port integer
    def __init__(self, Host_Address, Port):
        self.host_address = Host_Address
        self.port = Port
        return

    # Instantiate the communication
    def EstablishComm(self):
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        conn_socket = self.sock.connect((self.host_address, self.port))
        self.sock.sendall(str.encode("Establish Communication\r\n"))
        data = self.sock.recv(1024)
        if "Okay" in data:
            print("communication_established to the robot!")
        
    # Send data to the server, input is msg string
    def SendData(self,str_data):
        print("Sending data to the server at " + self.host_address)
        msg = str_data + "\r\n"
        self.sock.sendall(str.encode(msg)) 

    # Returns the received data from the server 
    def ReceiveData(self):
        print("Receiving data from the server at " + self.host_address)
        data = self.sock.recv(1024)
        data = data.decode();
        return data

    # Closes communication socket
    def CloseComm(self):
        print("closing socket")
        self.sock.close();