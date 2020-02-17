#!/usr/bin/env python

from socket import *
import rospy
from ik_solver.msg import PoseStateArray

HOST = '172.31.1.147'
PORT = 30003
ADDR = (HOST,PORT)

UdpSocket = socket(AF_INET, SOCK_DGRAM)

def callback(data):
   global UdpSocket
   msg = []
   for dt in data.poses:
      s = "{} {} {} {} {} {} {}".format(dt.position.x,dt.position.y,dt.position.z,
                                        dt.orientation.w,dt.orientation.x,dt.orientation.y,dt.orientation.z)
      msg.append(s)
      print(s)
   pocket = str.encode(";".join(msg))
   UdpSocket.sendto(pocket, ADDR)
   print("Send", len(pocket))

rospy.init_node('iiwa_retranslator')
rospy.Subscriber('/cmd/Trajectory', PoseStateArray, callback)
rospy.spin()

UdpSocket.close()



