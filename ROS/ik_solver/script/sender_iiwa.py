#!/usr/bin/env python

import rospy
from ik_solver.msg import PoseStateArray
from ik_solver.msg import PoseState


pub = rospy.Publisher('/cmd/Trajectory', PoseStateArray, queue_size=1)
rospy.init_node('publish_positions',anonymous=True)
#rate = rospy.Rate(5)

msg = PoseStateArray()
msg.robot = "Iiwa14"
#
m1 = PoseState()
m1.position.x = 582
m1.position.y = 100
m1.position.z = 473
m1.orientation.x = 0
m1.orientation.y = 1
m1.orientation.z = 0
m1.orientation.w = 0
m1.gripperOpen = True
msg.poses.append(m1)
#
m2 = PoseState()
m2.position.x = 682
m2.position.y = 100
m2.position.z = 473
m2.orientation.x = 0
m2.orientation.y = 1
m2.orientation.z = 0
m2.orientation.w = 0
m2.gripperOpen = False
msg.poses.append(m2)
#
m3 = PoseState()
m3.position.x = 682
m3.position.y = 0
m3.position.z = 473
m3.orientation.x = 0
m3.orientation.y = 1
m3.orientation.z = 0
m3.orientation.w = 0
m3.gripperOpen = True
msg.poses.append(m3)
#
m4 = PoseState()
m4.position.x = 582
m4.position.y = 0
m4.position.z = 473
m4.orientation.x = 0
m4.orientation.y = 1
m4.orientation.z = 0
m4.orientation.w = 0
m4.gripperOpen = False
msg.poses.append(m4)

pub.publish(msg)
