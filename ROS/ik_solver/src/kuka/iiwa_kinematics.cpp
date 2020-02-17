#include <cmath>
#include <eigen3/Eigen/Geometry>

#include "iiwa_kinematics.h"
// temporary
#include <iostream>

using namespace Eigen;

// angle range
double iiwa14::qmax[] = {RAD(170-GAP),RAD(120-GAP),RAD(170-GAP),RAD(120-GAP),RAD(170-GAP),RAD(120-GAP),RAD(175-GAP)};
double iiwa14::qmin[] = {-RAD(170-GAP),-RAD(120-GAP),-RAD(170-GAP),-RAD(120-GAP),-RAD(170-GAP),-RAD(120-GAP),-RAD(175-GAP)};
// velocity range
double iiwa14::vmax[] = {RAD(85-VGAP),RAD(85-VGAP),RAD(100-VGAP),RAD(75-VGAP),RAD(130-VGAP),RAD(135-VGAP),RAD(135-VGAP)};
// external torque thresholds (approximation)
double iiwa14::noise[] = {2.5, 4, 1.5, 3.5, 1.5, 1.5, 1};

Matrix4d FK(Matrix<double,JOINT_NO,1>& q) {
   // angles
   double c0 = cos(q(0)), s0 = sin(q(0));
   double c1 = cos(q(1)), s1 = sin(q(1));
   double c2 = -cos(q(2)), s2 = -sin(q(2));       // q(2)+pi
   double c3 = cos(-q(3)), s3 = sin(-q(3));
   double c4 = -cos(q(4)), s4 = -sin(q(4));       // q(4)-pi
   double c5 = cos(q(5)), s5 = sin(q(5));
   double c6 = cos(q(6)), s6 = sin(q(6));
   Matrix4d m;
   double _grp0 = (-c0*c1*s2-s0*c2), _grp1 = (c0*s1*s3+(c0*c1*c2-s0*s2)*c3);
   double _grp2 = (c0*s1*c3-(c0*c1*c2-s0*s2)*s3);
   m(0,0) = (_grp0*c4-_grp1*s4)*s6+((_grp0*s4+_grp1*c4)*c5-_grp2*s5)*c6; 
   m(0,1) = (_grp0*c4-_grp1*s4)*c6-((_grp0*s4+_grp1*c4)*c5-_grp2*s5)*s6; 
   m(0,2) = (_grp0*s4+_grp1*c4)*s5+_grp2*c5; 
   m(0,3) = LINK67*((_grp0*s4+_grp1*c4)*s5+_grp2*c5)+LINK45*_grp2+LINK23*c0*s1;
   double _grp3 = (c0*c2-s0*c1*s2), _grp4 = (s0*s1*s3+(c0*s2+s0*c1*c2)*c3);
   double _grp5 = (s0*s1*c3-(c0*s2+s0*c1*c2)*s3);
   m(1,0) = (_grp3*c4-_grp4*s4)*s6+((_grp3*s4+_grp4*c4)*c5-_grp5*s5)*c6; 
   m(1,1) = (_grp3*c4-_grp4*s4)*c6-((_grp3*s4+_grp4*c4)*c5-_grp5*s5)*s6; 
   m(1,2) = (_grp3*s4+_grp4*c4)*s5+_grp5*c5; 
   m(1,3) = LINK67*((_grp3*s4+_grp4*c4)*s5+_grp5*c5)+LINK45*_grp5+LINK23*s0*s1;
   double _grp6 = (c1*s3-s1*c2*c3), _grp7 = (s1*c2*s3+c1*c3);
   m(2,0) = (s1*s2*c4-_grp6*s4)*s6+((s1*s2*s4+_grp6*c4)*c5-_grp7*s5)*c6; 
   m(2,1) = (s1*s2*c4-_grp6*s4)*c6-((s1*s2*s4+_grp6*c4)*c5-_grp7*s5)*s6; 
   m(2,2) = (s1*s2*s4+_grp6*c4)*s5+_grp7*c5; 
   m(2,3) = LINK67*((s1*s2*s4+_grp6*c4)*s5+_grp7*c5)+LINK45*_grp7+LINK23*c1+LINK01; 
   m(3,0) = 0.0; 
   m(3,1) = 0.0; 
   m(3,2) = 0.0; 
   m(3,3) = 1.0; 
   
   return m;
}

// Find Jacobian using precalculated formulas
MatrixXd iiwa14::Jacobian(Matrix<double,JOINT_NO,1>& q) {
   // angles
   double c0 = cos(q(0)), s0 = sin(q(0));
   double c1 = cos(q(1)), s1 = sin(q(1));
   double c2 = -cos(q(2)), s2 = -sin(q(2));       // cos(q(2)+pi) = -cos(q(2)), the same for sin
   double c3 = cos(-q(3)), s3 = sin(-q(3));       // initially was calculated for negative argument
   double c4 = -cos(q(4)), s4 = -sin(q(4));       // cos(q(4)-pi) = -cos(q(4)), the same for sin
   double c5 = cos(q(5)), s5 = sin(q(5));
   double c6 = cos(q(6)), s6 = sin(q(6));
   // initialization
   MatrixXd m(CART_NO,JOINT_NO);
   //Matrix67d m;
   double _grp0 = (c0*s1*s3+(c0*c1*c2-s0*s2)*c3), _grp1 = (c0*s1*c3-(c0*c1*c2-s0*s2)*s3);
   double _grp2 = (c0*c2-s0*c1*s2), _grp3 = (s0*s1*s3+(c0*s2+s0*c1*c2)*c3);
   double _grp4 = (s0*s1*c3-(c0*s2+s0*c1*c2)*s3), _grp5 = (c1*s3-s1*c2*c3);
   double _grp6 = (s1*s2*s4+_grp5*c4)*s5, _grp7 = (s1*c2*s3+c1*c3);
   double _grp8 = (-c0*c1*s2-s0*c2);
   m(0,0) = (-LINK67*((_grp2*s4+_grp3*c4)*s5+_grp4*c5))-LINK45*_grp4-LINK23*s0*s1; 
   m(0,1) = c0*(LINK67*(_grp6+_grp7*c5)+LINK45*_grp7+LINK23*c1); 
   m(0,2) = s0*s1*(LINK67*(_grp6+_grp7*c5)+LINK45*_grp7+LINK23*c1)-c1*(LINK67*((_grp2*s4+_grp3*c4)*s5+_grp4*c5)+LINK45*_grp4+LINK23*s0*s1); 
   m(0,3) = _grp2*(LINK67*(_grp6+_grp7*c5)+LINK45*_grp7)-s1*s2*(LINK67*((_grp2*s4+_grp3*c4)*s5+_grp4*c5)+LINK45*_grp4); 
   m(0,4) = ((-s1*c2*s3)-c1*c3)*(LINK67*((_grp2*s4+_grp3*c4)*s5+_grp4*c5)+LINK45*_grp4)+_grp4*(LINK67*(_grp6+_grp7*c5)+LINK45*_grp7); 
   m(0,5) = LINK67*(_grp5*s4-s1*s2*c4)*((_grp2*s4+_grp3*c4)*s5+_grp4*c5)+LINK67*(_grp2*c4-_grp3*s4)*(_grp6+_grp7*c5); 
   m(0,6) = LINK67*(_grp6+_grp7*c5)*((_grp2*s4+_grp3*c4)*s5+_grp4*c5)+LINK67*((-_grp6)-_grp7*c5)*((_grp2*s4+_grp3*c4)*s5+_grp4*c5); 
   m(1,0) = LINK67*((_grp8*s4+_grp0*c4)*s5+_grp1*c5)+LINK45*_grp1+LINK23*c0*s1; 
   m(1,1) = s0*(LINK67*(_grp6+_grp7*c5)+LINK45*_grp7+LINK23*c1); 
   m(1,2) = c1*(LINK67*((_grp8*s4+_grp0*c4)*s5+_grp1*c5)+LINK45*_grp1+LINK23*c0*s1)-c0*s1*(LINK67*(_grp6+_grp7*c5)+LINK45*_grp7+LINK23*c1); 
   m(1,3) = s1*s2*(LINK67*((_grp8*s4+_grp0*c4)*s5+_grp1*c5)+LINK45*_grp1)+(c0*c1*s2+s0*c2)*(LINK67*(_grp6+_grp7*c5)+LINK45*_grp7); 
   m(1,4) = _grp7*(LINK67*((_grp8*s4+_grp0*c4)*s5+_grp1*c5)+LINK45*_grp1)+((c0*c1*c2-s0*s2)*s3-c0*s1*c3)*(LINK67*(_grp6+_grp7*c5)+LINK45*_grp7); 
   m(1,5) = LINK67*(s1*s2*c4-_grp5*s4)*((_grp8*s4+_grp0*c4)*s5+_grp1*c5)+LINK67*(_grp0*s4-_grp8*c4)*(_grp6+_grp7*c5); 
   m(1,6) = LINK67*(_grp6+_grp7*c5)*((_grp8*s4+_grp0*c4)*s5+_grp1*c5)+LINK67*(_grp6+_grp7*c5)*((-(_grp8*s4+_grp0*c4)*s5)-_grp1*c5); 
   m(2,0) = 0.0; 
   m(2,1) = (-s0*(LINK67*((_grp2*s4+_grp3*c4)*s5+_grp4*c5)+LINK45*_grp4+LINK23*s0*s1))-c0*(LINK67*((_grp8*s4+_grp0*c4)*s5+_grp1*c5)+LINK45*_grp1+LINK23*c0*s1); 
   m(2,2) = c0*s1*(LINK67*((_grp2*s4+_grp3*c4)*s5+_grp4*c5)+LINK45*_grp4+LINK23*s0*s1)-s0*s1*(LINK67*((_grp8*s4+_grp0*c4)*s5+_grp1*c5)+LINK45*_grp1+LINK23*c0*s1); 
   m(2,3) = _grp8*(LINK67*((_grp2*s4+_grp3*c4)*s5+_grp4*c5)+LINK45*_grp4)+(s0*c1*s2-c0*c2)*(LINK67*((_grp8*s4+_grp0*c4)*s5+_grp1*c5)+LINK45*_grp1); 
   m(2,4) = _grp1*(LINK67*((_grp2*s4+_grp3*c4)*s5+_grp4*c5)+LINK45*_grp4)+((c0*s2+s0*c1*c2)*s3-s0*s1*c3)*(LINK67*((_grp8*s4+_grp0*c4)*s5+_grp1*c5)+LINK45*_grp1); 
   m(2,5) = LINK67*(_grp8*c4-_grp0*s4)*((_grp2*s4+_grp3*c4)*s5+_grp4*c5)+LINK67*(_grp3*s4-_grp2*c4)*((_grp8*s4+_grp0*c4)*s5+_grp1*c5); 
   m(2,6) = LINK67*((_grp8*s4+_grp0*c4)*s5+_grp1*c5)*((_grp2*s4+_grp3*c4)*s5+_grp4*c5)+LINK67*((_grp8*s4+_grp0*c4)*s5+_grp1*c5)*((-(_grp2*s4+_grp3*c4)*s5)-_grp4*c5); 
   m(3,0) = 0.0; 
   m(3,1) = -s0; 
   m(3,2) = c0*s1; 
   m(3,3) = (-c0*c1*s2)-s0*c2; 
   m(3,4) = c0*s1*c3-(c0*c1*c2-s0*s2)*s3; 
   m(3,5) = _grp8*c4-_grp0*s4; 
   m(3,6) = (_grp8*s4+_grp0*c4)*s5+_grp1*c5; 
   m(4,0) = 0.0; 
   m(4,1) = c0; 
   m(4,2) = s0*s1; 
   m(4,3) = c0*c2-s0*c1*s2; 
   m(4,4) = s0*s1*c3-(c0*s2+s0*c1*c2)*s3; 
   m(4,5) = _grp2*c4-_grp3*s4; 
   m(4,6) = (_grp2*s4+_grp3*c4)*s5+_grp4*c5; 
   m(5,0) = 1.0; 
   m(5,1) = 0.0; 
   m(5,2) = c1; 
   m(5,3) = s1*s2; 
   m(5,4) = s1*c2*s3+c1*c3; 
   m(5,5) = s1*s2*c4-_grp5*s4; 
   m(5,6) = _grp6+_grp7*c5; 
   
   return m;
}

Matrix<double,POSE_QUATERN,1> iiwa14::cartesianState(Matrix<double,JOINT_NO,1>& q) 
{
   Matrix4d fk = FK(q);
   Matrix3d rot = fk.block(0,0,3,3);
   Quaterniond v(rot);

   Matrix<double,POSE_QUATERN,1> res;
   res(0) = fk(0,3); res(1) = fk(1,3); res(2) = fk(2,3);

   res(3) = v.x(); res(4) = v.y(); res(5) = v.z(); res(6) = v.w();

   return res;
}

