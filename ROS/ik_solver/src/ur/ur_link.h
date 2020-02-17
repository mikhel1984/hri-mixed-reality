#ifndef UR10_ROBOT_H
#define UR10_ROBOT_H

#include <ros/ros.h>
//#include <fstream>

#include "iiwa_msgs/JointPosition.h"
#include "geometry_msgs/PoseStamped.h"
//#include "geometry_msgs/PoseArray.h"
//#include "iiwa_msgs/JointVelocity.h"
#include "ik_solver/PoseStateArray.h"
#include "trajectory_msgs/JointTrajectory.h"

#include "../kinematics/saturation_in_null_space.h"

#include "ur_state.h"

/// Communication with ROS, control the robot
class UrLink {
public:
   UrLink(ros::NodeHandle& nh);
   ~UrLink();
   
   bool evalAndPublish();   

   void jointPositionCallback(const iiwa_msgs::JointPosition& jp);
   
   void cartArrayCallback(const ik_solver::PoseStateArray& pa);

private:   

   void updateTrajectory();
 
   UrState& rs;
   
   ros::Subscriber subJointPosition;
   ros::Subscriber subTrajPositions;
   
   ros::Publisher  pubJointTraj;
   
   trajectory_msgs::JointTrajectory msgJointTraj;

   //ros::Publisher  pubJointPosition;   
   //ros::Time begin; 
   
   //iiwa_msgs::JointPosition cmdJointPosition;
      
   SNSCalculator<JOINT_NO,CART_NO>* sns;
   ros::AsyncSpinner *spinner;
   
   bool isStopped;
   
}; // UrLink

#endif // UR10_ROBOT_H
