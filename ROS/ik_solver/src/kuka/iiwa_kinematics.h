#ifndef IIWA_KINEMATICS
#define IIWA_KINEMATICS

#include "../kinematics/homogenous.h"
#include "kuka_settings.h"

namespace iiwa14 {

// angle range
extern double qmax[JOINT_NO], qmin[JOINT_NO];
// velocity range
extern double vmax[JOINT_NO];
// torque threshold approximation
extern double noise[JOINT_NO];


// Find Jacobian using precalculated formulas
Eigen::MatrixXd Jacobian(Eigen::Matrix<double,JOINT_NO,1>& q);

Eigen::Matrix<double,POSE_QUATERN,1> cartesianState(Eigen::Matrix<double,JOINT_NO,1>& q);

} // namespace iiwa14

#endif // IIWA_KINEMATICS
