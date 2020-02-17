#ifndef IIWA14_ROBOT_H
#define IIWA14_ROBOT_H

#include <ros/ros.h>
//#include <fstream>

#include "iiwa_msgs/JointPosition.h"
#include "geometry_msgs/PoseStamped.h"
#include "ik_solver/PoseStateArray.h"
//#include "geometry_msgs/PoseArray.h"
//#include "iiwa_msgs/JointVelocity.h"
#include "trajectory_msgs/JointTrajectory.h"

#include "../kinematics/saturation_in_null_space.h"

#include "kuka_state.h"

/// Communication with ROS, control the robot
class RosIiwaLink {
public:
   RosIiwaLink(ros::NodeHandle& nh);
   ~RosIiwaLink();
   
   bool evalAndPublish();   

   void jointPositionCallback(const iiwa_msgs::JointPosition& jp);
   
   void cartArrayCallback(const ik_solver::PoseStateArray& pa);

private:   

   void updateTrajectory();
 
   KukaState& rs;
   
   ros::Subscriber subJointPosition;
   ros::Subscriber subTrajPositions;
   
   ros::Publisher  pubJointTraj;
   
   trajectory_msgs::JointTrajectory msgJointTraj;
      
   SNSCalculator<JOINT_NO,CART_NO>* sns;
   ros::AsyncSpinner *spinner;
   
   bool isStopped;
   
}; // IiwaRobot

#endif // IIWA14_ROBOT_H
