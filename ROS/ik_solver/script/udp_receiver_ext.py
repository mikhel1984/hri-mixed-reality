#!/usr/bin/env python
# Read robot positions and torques

from socket import *
import rospy
from iiwa_msgs.msg import JointPosition
from iiwa_msgs.msg import JointTorque

PORT = 30002

UdpSocket = socket(AF_INET, SOCK_DGRAM)
UdpSocket.bind(('172.31.1.2',PORT))

rospy.init_node('iiwa_state_publisher')
pub = rospy.Publisher('/iiwa/state/JointPosition',JointPosition,queue_size=1)
pub_ext = rospy.Publisher('/iiwa/state/JointExtTorque',JointTorque,queue_size=1)
# receive and retranslate
msg = JointPosition()
tau = JointTorque()
print("Waiting for data")
while not rospy.is_shutdown():   
   data, address = UdpSocket.recvfrom(512)
   if not data:
      print("WTF?")
      continue
   #print("Joints",data)
   lst = data.split(' ')
   if len(lst) != 14:
      print("WRONG NUMBER OF JOINTS")
      continue
   # positions
   msg.position.a1 = float(lst[0])
   msg.position.a2 = float(lst[1])
   msg.position.a3 = float(lst[2])
   msg.position.a4 = float(lst[3])
   msg.position.a5 = float(lst[4])
   msg.position.a6 = float(lst[5])
   msg.position.a7 = float(lst[6])
   pub.publish(msg)
   # external torques
   tau.torque.a1 = float(lst[7])
   tau.torque.a2 = float(lst[8])
   tau.torque.a3 = float(lst[9])
   tau.torque.a4 = float(lst[10])
   tau.torque.a5 = float(lst[11])
   tau.torque.a6 = float(lst[12])
   tau.torque.a7 = float(lst[13])
   pub_ext.publish(tau)

