#!/usr/bin/env python

import sys, time
import rospy
import moveit_commander

from ik_solver.msg import PoseState, PoseStateArray
from trajectory_msgs.msg import JointTrajectory
from geometry_msgs.msg import Pose
from sensor_msgs.msg import JointState
from math import pi, sqrt

class MoveItIiwaInterface(object):
  
  def __init__(self):
    "IIWA node constructor"
    super(MoveItIiwaInterface, self).__init__()
    # initialize
    moveit_commander.roscpp_initialize(sys.argv)
    # init node
    rospy.init_node('FollowPoints',anonymous=True)
    self.pub = rospy.Publisher('/calc/Trajectory', JointTrajectory, queue_size=1)
    rospy.Subscriber('/cmd/Trajectory', PoseStateArray, self.callback)
    # read robot state    
    self.currentState = ()
    rospy.Subscriber('/iiwa/state/JointsPosition', JointState, self.stateCallback)
    # robot interface
    self.robot = moveit_commander.RobotCommander()
    # joint group
    self.group = moveit_commander.MoveGroupCommander("manipulator")
    # trajectory points
    self.trajectory = []
    self.gripper = []
    self.gripperLast = 1.0
    # new trajectory
    self.isNew = False
    # name
    self.robotName = "Iiwa14"
    # control message
    self.msg = JointTrajectory()
    self.msg.joint_names = ["j1","j2","j3","j4","j5","j6","j7","gripper"]
    self.msg.header.frame_id = self.robotName    
    # wait for current state
    #while not self.currentState:
    #  print "Waiting for connection..."
    #  time.sleep(2)
    #self.group.go(self.currentState, wait=True)
    initial = [0,0.5,0,-1.4,0,1.22,0]
    self.group.go(initial, wait=True)
    #print self.group.get_current_pose()
    print "MoveIt! Iiwa interface is prepared"
  
  def evalAndPublish(self):
    "Define trajectory points and publish result"
    if not self.isNew:
      return True
    self.isNew = False
    # find way points
    STEP = 0.02    # m, cartesian distance
    (plan,fraction) = self.group.compute_cartesian_path(self.trajectory,STEP,0.0)
    print "Solve:", fraction*100.0, "%"
    # modify commands
    traj = plan.joint_trajectory
    # save last joint state for publishing
    goal = traj.points[-1].positions[:]
    i = 0
    for t in traj.points:
      # add gripper state
      t.positions += (self.gripper[i],)      
      if self.isZero(t) and i+1 < len(self.gripper):
      #if self.isZero(t):
        # new point
        i += 1        
    self.gripperLast = self.gripper[-1]
    self.msg.points = traj.points  
    self.msg.header.stamp = rospy.get_rostime()  
    # send result
    self.pub.publish(self.msg)
    # "move" model
    self.group.go(goal, wait=False)
    self.group.stop() # ?
    
  def callback(self,data):
    "Listen commands"
    if data.robot != self.robotName:
      return
    # clear
    self.isNew = False
    self.trajectory = []
    self.gripper = [self.gripperLast]    
    # convert formats
    for pose in data.poses:
      # copy daya
      p = Pose()
      #p.position = pose.position
      p.position.x = pose.position.x / 1000
      p.position.y = pose.position.y / 1000
      p.position.z = pose.position.z / 1000
      #p.orientation = pose.orientation
      #q = self.rotate(pose.orientation)
      q = pose.orientation
      p.orientation.w = q.w
      p.orientation.x = q.x
      p.orientation.y = q.y
      p.orientation.z = q.z
      gr = 1.0 if pose.gripperOpen else 0.0
      # duplicate points when gripper change state
      self.trajectory.append(p)
      if gr != self.gripper[-1]:        
        self.gripper.append(self.gripper[-1]) 
      else:
        self.gripper.append(gr) 
      # duplicate to get zero value   
      self.trajectory.append(p)
      self.gripper.append(gr)
    # allow execution
    self.isNew = True    

  def stateCallback(self,data):
    "Read current robot state"
    self.currentState = data.position
    
  def isZero(self,point):
    "Check if velocity is zero"
    s = 0.0
    for v in point.velocities:
      s += abs(v)
    return s == 0.0

  def rotate(self,q):
    "Rotate quaternion"
    w0 = sqrt(2)/2
    #q_w = (0,0,-w0,w0)    # "normal" zero position
    q_iw = (0,0,w0,-w0)    # inverted normal world position
    q_m = (w0,0,0,w0)     # moveit zero position
    # get transformation in "normal" world
    dq = self.prod(q_iw, (q.w,q.x,q.y,q.z))
    # translate into moveit point of view
    dq2 = (dq[0],dq[3],-dq[1],-dq[2])
    # find moveit orientation
    return self.prod(q_m, dq2)

  def prod(self,q1,q2):
    "Quaternion product"
    q3 = (q1[0]*q2[0]-q1[1]*q2[1]-q1[2]*q2[2]-q1[3]*q2[3], 
          q1[0]*q2[1]+q1[1]*q2[0]+q1[2]*q2[3]-q1[3]*q2[2], 
          q1[0]*q2[2]-q1[1]*q2[3]+q1[2]*q2[0]+q1[3]*q2[1], 
          q1[0]*q2[3]+q1[1]*q2[2]-q1[2]*q2[1]+q1[3]*q2[0])
    return q3
  
  #def inv(self,q):
  #  "Unit quaternion inversion"
  #  return (q[0],-q[1],-q[2],-q[3])

print "Waiting for MoveIt!..."
time.sleep(4)
      
robot = MoveItIiwaInterface()
rate = rospy.Rate(10)

while not rospy.is_shutdown():
  robot.evalAndPublish()
  rate.sleep()
