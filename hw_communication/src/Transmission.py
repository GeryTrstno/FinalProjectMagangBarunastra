#!/usr/bin/env python

import rospy
from hw_communication.msg import Motor
import socket

IP = "192.168.1.12"
Port = 8080
password = "ITS"

def Send_Data(message):

    output = password + message
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.sendto(output.encode(), (IP, Port))
    print("Data Sent")
    sock.close()

def Receive_Data():
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind(IP, Port)

    while True:
        data, address = sock.recvfrom(4096)
        message = data.encode()
        verifPass = message[:3]
        if verifPass == password:
            messageData = message[3:]
            print("Pesannya adalah: %s" %messageData)
        else:
            pass

def Kapal_Maju():
    return "2000"

def Kapal_Mundur():
    return "-1500"

def Kapal_Kanan():
    return "2000, 1000"

def Kapal_Kiri():
    return "1000, 2000"

def Kapal_Tembak():
    return "1000"

if __name__ == '__main__':
    Send_Data(Kapal_Maju)
    Send_Data(Kapal_Mundur)
    Send_Data(Kapal_Kanan)
    Send_Data(Kapal_Kiri)
    Send_Data(Kapal_Tembak)

    Receive_Data()