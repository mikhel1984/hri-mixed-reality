	// Basic robot reactions

//	UNITS
// Length:       mm
// Angle:        rad
// Velocity:     mm/s
// Rad.velocity: rad/s

#include "kuka/ros_iiwa_link.h"
#include <iostream>

#define FREQ  10.0              // message frequency

int main(int argc, char **argv) 
{
     
   // initialise ROS objects
   ros::init(argc, argv, "FollowPoints");
   ros::NodeHandle nh("~");
   ros::Rate rate(FREQ);

   // interaction with ROS
   RosIiwaLink robot(nh);
      
   // robot state exchange
   KukaState& rs = KukaState::getInstance();
   //rs.cartVMax = 50;
   
   // debug
   //rs.jointPosition << 0, 0.5, 0, -1.4, 0, 1.22, 0;
   //rs.isRobotConnected = true;

   // run
   while(ros::ok() && rs.isContinue) {       
       robot.evalAndPublish();
       rate.sleep();       
   }

   // stopping
   ROS_INFO("Bye!");

   return 0;
}
