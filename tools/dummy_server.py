import socket
from ctypes import *
 

localIP     = "0.0.0.0"
localPort   = 3000
bufferSize  = 1024

 

msgFromServer       = "-"
bytesToSend         = str.encode(msgFromServer)

 

# Create a datagram socket
UDPServerSocket = socket.socket(family=socket.AF_INET, type=socket.SOCK_DGRAM)

 

# Bind to address and ip
UDPServerSocket.bind((localIP, localPort))

 

print("UDP server up and listening")

class MSG(Structure):
    _pack_ = 1
    _fields_ = [
        ("forward", c_int32),
        ("steering", c_int32),
        ("turret", c_int32),
        ("boom", c_int32),
        ("hook", c_int32),
    ]

# Listen for incoming datagrams

while True:
    bytesAddressPair = UDPServerSocket.recvfrom(bufferSize)
    message = bytesAddressPair[0]
    address = bytesAddressPair[1]

    # clientMsg = "Message from Client:{}".format(message)
    # clientIP  = "Client IP Address:{}".format(address)
    
    msg = MSG.from_buffer_copy(message)
    print(msg.forward, msg.steering, msg.turret, msg.boom, msg.hook)
    # print(clientIP)
  

    # Sending a reply to client

    UDPServerSocket.sendto(bytesToSend, address)