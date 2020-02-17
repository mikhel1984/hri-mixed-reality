#ifndef UR_KINEMATICS
#define UR_KINEMATICS

#include "../kinematics/homogenous.h"
#include "ur_settings.h"

namespace ur10 {

// angle range
extern double qmax[JOINT_NO], qmin[JOINT_NO];
// velocity range
extern double vmax[JOINT_NO];
// torque threshold approximation
//extern double noise[JOINT_NO];


// Find Jacobian using precalculated formulas
Eigen::MatrixXd Jacobian(Eigen::Matrix<double,JOINT_NO,1>& q);

Eigen::Matrix<double,POSE_QUATERN,1> cartesianState(Eigen::Matrix<double,JOINT_NO,1>& q);

} // namespace ur10

#endif // UR_KINEMATICS
