#ifndef UR_PROJECT_SETTINGS_H
#define UP_PROJECT_SETTINGS_H

//
//	KUKA PARAMETERS
//

// number of coordinates
#define JOINT_NO 6         // joint space
#define CART_NO  6         // cartesian space
#define POSE_QUATERN 7     // ROS parameters for joint space

// link length
#define LEN_D1    180.7
#define LEN_A2    (-612.7)
#define LEN_A3    (-571.55)
#define LEN_D4    174.15
#define LEN_D5    119.85
#define LEN_D6    116.55

//

// gap between real and program limits
#define GAP  5             // position
#define VGAP 3             // velocity

// square of position of error
#define CART_SQ_ERR            5   // in cartesian space
#define JOINT_SQ_ERR           0.0001   // in joint space

//
//	OTHER
//

#define PI   3.14159265358979323846
#define SEP  ","                    // CSV separator


#endif // UR_PROJECT_SETTINGS_H
