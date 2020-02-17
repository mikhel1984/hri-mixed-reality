#!/usr/bin/env python
# Read robot positions

from socket import *
import rospy
from iiwa_msgs.msg import JointPosition

PORT = 30002

UdpSocket = socket(AF_INET, SOCK_DGRAM)
UdpSocket.bind(('172.31.1.2',PORT))

rospy.init_node('iiwa_state_publisher')
pub = rospy.Publisher('/iiwa/state/JointPosition',JointPosition,queue_size=1)
# receive and retranslate
msg = JointPosition()
print("Waiting for data")
while not rospy.is_shutdown():   
   data, address = UdpSocket.recvfrom(256)
   if not data:
      print("wtf?")
      continue
   print("Joints",data)
   lst = data.split(' ')
   if len(lst) != 7:
      print("WRONG NUMBER OF JOINTS")
      continue
   msg.position.a1 = float(lst[0])
   msg.position.a2 = float(lst[1])
   msg.position.a3 = float(lst[2])
   msg.position.a4 = float(lst[3])
   msg.position.a5 = float(lst[4])
   msg.position.a6 = float(lst[5])
   msg.position.a7 = float(lst[6])
   pub.publish(msg)

