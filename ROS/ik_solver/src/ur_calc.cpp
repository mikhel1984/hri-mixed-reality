	// Basic robot reactions

//	UNITS
// Length:       mm
// Angle:        rad
// Velocity:     mm/s
// Rad.velocity: rad/s

#include "ur/ur_link.h"
#include <iostream>

#define FREQ  10.0              // message frequency

int main(int argc, char **argv) 
{
     
   // initialise ROS objects
   ros::init(argc, argv, "FollowPoints");
   ros::NodeHandle nh("~");
   ros::Rate rate(FREQ);

   // interaction with ROS
   UrLink robot(nh);
      
   // robot state exchange
   UrState& rs = UrState::getInstance();
   //rs.cartVMax = 50;
   
   // debug
   rs.jointPosition << 0, -1.4, 1.92, -2.15, -1.41, 0;
   rs.isRobotConnected = true;

   // run
   while(ros::ok() && rs.isContinue) {
       robot.evalAndPublish();
       rate.sleep();       
   }

   // stopping
   ROS_INFO("Bye!");

   return 0;
}
