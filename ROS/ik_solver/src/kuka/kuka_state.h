#ifndef ROBOT_CURRENT_STATE_H
#define ROBOT_CURRENT_STATE_H

#include <eigen3/Eigen/Dense>
#include <string>

#include "kuka_settings.h"

/**
@brief Current robot state.
Singleton object which stores information about robot state
and share it to other objects.
*/
struct KukaState {
   // current position
   Eigen::Matrix<double,JOINT_NO,1> jointPosition;
   // command position
   Eigen::Matrix<double,JOINT_NO,1> cmdJointPosition;
   // current joint torque
   //Eigen::Matrix<double,JOINT_NO,1> externalTorque;
   // cartesian 
   //Eigen::Matrix<double,POSE_QUATERN,1> goalCartesianPos;  // X, Y, Z, QX, QY, QZ, QW
   Eigen::Matrix<double,POSE_QUATERN,1> cartesianPosition; // X, Y, Z, QX, QY, QZ, QW
   // trajectory
   Eigen::MatrixXd trajectory;
   Eigen::MatrixXd gripper;
   // log file name
   //std::string log;    // ?
   // period of commands
   double T;
   // maximal cartesian velocity
   double cartVMax;
   // torque memory
   //double memTorques[NN_MEMORY * JOINT_NO];  // initialize - ??
   // current event(s)
   //int events;
   // collision description (?)
   //int collision;
   // flag for connection
   bool isRobotConnected;
   // flag for exit
   bool isContinue;
   // flag for new trajectory
   bool isNewTrajectory;

   /**
   Get robot instance.
   @return Reference to RobotState instance.
   */
   static KukaState& getInstance() {
      static KukaState state;
      return state;
   }

private:
   KukaState() 
    : isRobotConnected(false)
    , isContinue(true)
    , isNewTrajectory(false)
    , cartVMax(50)
    , trajectory(Eigen::MatrixXd(1,1))
    , gripper(Eigen::MatrixXd(1,1))
    , T(0) { } 
};  // RobotState


#endif // ROBOT_CURRENT_STATE_H
