#include "ros_iiwa_link.h"
#include "iiwa_kinematics.h"
#include "../debug_settings.h"
#include "../topics.h"

#include <iostream>

using namespace Eigen;

Matrix<double,CART_NO,1> findVelocity(Matrix<double,POSE_QUATERN,1>& goal, Matrix<double,POSE_QUATERN,1>& curr, double vMax)
{
   Matrix<double,CART_NO,1> cartV;
   cartV.setZero();
   // translation
   cartV(0) = goal(0)-curr(0);
   cartV(1) = goal(1)-curr(1);
   cartV(2) = goal(2)-curr(2);
   //cartV /= rs.T;
   double vel= cartV.norm();        
   if(vel > vMax) cartV *= (vMax / vel); 
   
   // orientaion
   Quaterniond qGoal(goal(6),goal(3),goal(4),goal(5));
   Quaterniond qCurrent(curr(6),curr(3),curr(4),curr(5));
   
   Matrix3d mGoal = qGoal.toRotationMatrix();
   Matrix3d mCurrent = qCurrent.toRotationMatrix();
   Matrix3d sRot = (mGoal-mCurrent)*mCurrent.transpose(); 
   Vector3d vRot(sRot(2,1), sRot(0,2), sRot(1,0));
   //double w = vRot.norm();
   cartV(3) = vRot(0); cartV(4) = vRot(1); cartV(5) = vRot(2);

   return cartV;
}

RosIiwaLink::RosIiwaLink(ros::NodeHandle& nh)          
	 : isStopped(true)
         , rs(KukaState::getInstance())
{    
   // subscribe
   subJointPosition = nh.subscribe(SUB_KUKA_JOINT_STATE, 1, &RosIiwaLink::jointPositionCallback, this);
   subTrajPositions = nh.subscribe(SUB_TRAJECTORY, 3, &RosIiwaLink::cartArrayCallback, this);
   // publish
   pubJointTraj = nh.advertise<trajectory_msgs::JointTrajectory>(PUB_TRAJECTORY, 1);
      
   msgJointTraj.header.frame_id = KUKA_NAME;
   msgJointTraj.joint_names.resize(JOINT_NO+1);
   msgJointTraj.joint_names[0] = "j1";
   msgJointTraj.joint_names[1] = "j2";
   msgJointTraj.joint_names[2] = "j3";
   msgJointTraj.joint_names[3] = "j4";
   msgJointTraj.joint_names[4] = "j5";
   msgJointTraj.joint_names[5] = "j6";
   msgJointTraj.joint_names[6] = "j7";
   msgJointTraj.joint_names[7] = "gripper";

   // calculator
   sns = new SNSCalculator<JOINT_NO,CART_NO>(iiwa14::qmax, iiwa14::qmin, iiwa14::vmax);
   // start time
   //begin = ros::Time::now();
   //lastSaveTime = ros::Time::now().toSec();
   
   spinner = new ros::AsyncSpinner(1);
   spinner->start();
}

RosIiwaLink::~RosIiwaLink()
{
   spinner->stop();

   delete spinner;
   delete sns;
}

void RosIiwaLink::cartArrayCallback(const ik_solver::PoseStateArray& pa)
{
   if(pa.robot != KUKA_NAME) return;
      
   rs.isNewTrajectory = false;
   int rows = pa.poses.size();
   rs.trajectory.resize(2*rows, POSE_QUATERN);
   rs.gripper.resize(2*rows,1);
   rs.trajectory.setZero();
   int ind = 0;
   for(int r = 0; r < rows; r++) {
      rs.trajectory(ind,0) = pa.poses[r].position.x;      
      rs.trajectory(ind,1) = pa.poses[r].position.y;      
      rs.trajectory(ind,2) = pa.poses[r].position.z;      
      rs.trajectory(ind,3) = pa.poses[r].orientation.x;
      rs.trajectory(ind,4) = pa.poses[r].orientation.y;
      rs.trajectory(ind,5) = pa.poses[r].orientation.z;
      rs.trajectory(ind,6) = pa.poses[r].orientation.w;
      rs.gripper(ind)      = pa.poses[r].gripperOpen ? 1.0 : 0.0;
      
      // duplicate point with different gripper state
      if(r > 0 && pa.poses[r-1].gripperOpen != pa.poses[r].gripperOpen) {
         rs.gripper(ind) = pa.poses[r-1].gripperOpen;
         
         ind++;
         rs.trajectory(ind,0) = pa.poses[r].position.x;      
         rs.trajectory(ind,1) = pa.poses[r].position.y;      
         rs.trajectory(ind,2) = pa.poses[r].position.z;      
         rs.trajectory(ind,3) = pa.poses[r].orientation.x;
         rs.trajectory(ind,4) = pa.poses[r].orientation.y;
         rs.trajectory(ind,5) = pa.poses[r].orientation.z;
         rs.trajectory(ind,6) = pa.poses[r].orientation.w;
         rs.gripper(ind)      = pa.poses[r].gripperOpen ? 1.0 : 0.0;         
      }
      ind++;
   }
    
   rs.isNewTrajectory = true;   
}

void RosIiwaLink::jointPositionCallback(const iiwa_msgs::JointPosition& jp)
{
   rs.isRobotConnected = true;
   rs.jointPosition(0) = jp.position.a1;
   rs.jointPosition(1) = jp.position.a2;
   rs.jointPosition(2) = jp.position.a3;
   rs.jointPosition(3) = jp.position.a4;
   rs.jointPosition(4) = jp.position.a5;
   rs.jointPosition(5) = jp.position.a6;
   rs.jointPosition(6) = jp.position.a7;
   
   if(isStopped) {
      rs.cmdJointPosition = rs.jointPosition;
      isStopped = false;
   }
}

bool RosIiwaLink::evalAndPublish() 
{
   if(!rs.isNewTrajectory) return true;
   rs.isNewTrajectory = false;

   if(!rs.isRobotConnected) rs.jointPosition.setZero();
   
   Matrix<double,JOINT_NO,1> nextState = rs.jointPosition; 
   Matrix<double,POSE_QUATERN,1> nextPose = iiwa14::cartesianState(nextState);
   Matrix<double,JOINT_NO,1> nsv;
   nsv.setZero();
   
   int rows = rs.trajectory.rows();
   
   for(int i = 0; i < rows; i++) {
      Matrix<double,POSE_QUATERN,1> goalPosition = rs.trajectory.row(i);
      if(goalPosition.squaredNorm() < 1E-6) {
         rows = i;
         break;
      }
   }
   
   msgJointTraj.header.stamp = ros::Time::now();
   msgJointTraj.points.resize(rows);
   
   //std::cout << "Initial " << nextPose.transpose() << std::endl;

   for(int i = 0; i < rows; i++) {
      Matrix<double,POSE_QUATERN,1> goalPosition = rs.trajectory.row(i);
      //if(goalPosition.squaredNorm() < 1E-6) break;    // "marker" for end of rows
      int counter = 0;
      while(true) {
         Matrix<double,CART_NO,1> velocity = findVelocity(goalPosition, nextPose, rs.cartVMax);         
         
         Matrix<double,JOINT_NO,1> diff = sns->jointVelocity(velocity, nsv, nextState, 
                                                             iiwa14::Jacobian(nextState), rs.T);         
         if(!sns->successfull()) {
            ROS_ERROR("Can't find solution for:");
            std::cout << goalPosition.transpose() << std::endl;
            //std::cout << "Can't find solution for "  << goalPosition.transpose() << std::endl;
	    return false;
         }
         
         nextState += diff;
         nextPose = iiwa14::cartesianState(nextState);

      // check difference
         if((nextPose-goalPosition).squaredNorm() < CART_SQ_ERR) {
            break;
         }
         if(++counter > 300) {
            //std::cout << "Too much iterations!" << std::endl;
            ROS_ERROR("Too much iterations!");
            return false;
         }
         
      }
      // process result
      msgJointTraj.points[i].positions.resize(JOINT_NO+1);
      msgJointTraj.points[i].velocities.resize(JOINT_NO+1);
      msgJointTraj.points[i].accelerations.resize(JOINT_NO+1);
      
      for(int j = 0; j < JOINT_NO; j++) {
         msgJointTraj.points[i].positions[j] = nextState(j);
      }  
      msgJointTraj.points[i].positions[JOINT_NO] = rs.gripper(i);    
      
      std::cout << nextState.transpose() << std::endl;            
   }
   msgJointTraj.header.stamp = ros::Time::now();
   // publish result
   pubJointTraj.publish(msgJointTraj);
   
   return true;
}


