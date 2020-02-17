#include <cmath>
#include <eigen3/Eigen/Geometry>

#include "ur_kinematics.h"
// temporary
#include <iostream>

using namespace Eigen;

// angle range
double ur10::qmax[] = {RAD(360-GAP),RAD(10-GAP),RAD(180-GAP),RAD(60-GAP),RAD(180-GAP),RAD(180-GAP)};
double ur10::qmin[] = {-RAD(360-GAP),-RAD(190-GAP),-RAD(180-GAP),-RAD(250-GAP),-RAD(180-GAP),-RAD(180-GAP)};
// velocity range
double ur10::vmax[] = {RAD(131-VGAP),RAD(131-VGAP),RAD(191-VGAP),RAD(191-VGAP),RAD(191-VGAP),RAD(191-VGAP)};
// external torque thresholds (approximation)
//double ur10::noise[] = {2.5, 4, 1.5, 3.5, 1.5, 1.5};

Matrix4d FK(Matrix<double,JOINT_NO,1>& q) {
   // angles
   double c1 = cos(q(0)) , s1 = sin(q(0));
   double c2 = cos(q(1)) , s2 = sin(q(1));
   double c3 = cos(q(2)) , s3 = sin(q(2));
   double c4 = cos(q(3)) , s4 = sin(q(3));
   double c5 = cos(q(4)) , s5 = sin(q(4));
   double c6 = cos(q(5)) , s6 = sin(q(5));
   
   Matrix4d m;
   m(0,0) = (((-c1*c2*s3)-c1*s2*c3)*c4-(c1*c2*c3-c1*s2*s3)*s4)*s6+(s1*s5+(((-c1*c2*s3)-c1*s2*c3)*s4+(c1*c2*c3-c1*s2*s3)*c4)*c5)*c6; 
   m(0,1) = (((-c1*c2*s3)-c1*s2*c3)*c4-(c1*c2*c3-c1*s2*s3)*s4)*c6-(s1*s5+(((-c1*c2*s3)-c1*s2*c3)*s4+(c1*c2*c3-c1*s2*s3)*c4)*c5)*s6; 
   m(0,2) = s1*c5-(((-c1*c2*s3)-c1*s2*c3)*s4+(c1*c2*c3-c1*s2*s3)*c4)*s5; 
   m(0,3) = LEN_D6*(s1*c5-(((-c1*c2*s3)-c1*s2*c3)*s4+(c1*c2*c3-c1*s2*s3)*c4)*s5)+LEN_D5*((c1*c2*c3-c1*s2*s3)*s4-((-c1*c2*s3)-c1*s2*c3)*c4)-LEN_A3*c1*s2*s3+LEN_A3*c1*c2*c3+LEN_A2*c1*c2+LEN_D4*s1; 
   m(1,0) = (((-s1*c2*s3)-s1*s2*c3)*c4-(s1*c2*c3-s1*s2*s3)*s4)*s6+((((-s1*c2*s3)-s1*s2*c3)*s4+(s1*c2*c3-s1*s2*s3)*c4)*c5-c1*s5)*c6; 
   m(1,1) = (((-s1*c2*s3)-s1*s2*c3)*c4-(s1*c2*c3-s1*s2*s3)*s4)*c6-((((-s1*c2*s3)-s1*s2*c3)*s4+(s1*c2*c3-s1*s2*s3)*c4)*c5-c1*s5)*s6; 
   m(1,2) = (-(((-s1*c2*s3)-s1*s2*c3)*s4+(s1*c2*c3-s1*s2*s3)*c4)*s5)-c1*c5; 
   m(1,3) = LEN_D6*((-(((-s1*c2*s3)-s1*s2*c3)*s4+(s1*c2*c3-s1*s2*s3)*c4)*s5)-c1*c5)+LEN_D5*((s1*c2*c3-s1*s2*s3)*s4-((-s1*c2*s3)-s1*s2*c3)*c4)-LEN_A3*s1*s2*s3+LEN_A3*s1*c2*c3+LEN_A2*s1*c2-LEN_D4*c1; 
   m(2,0) = ((c2*c3-s2*s3)*c4-(c2*s3+s2*c3)*s4)*s6+((c2*c3-s2*s3)*s4+(c2*s3+s2*c3)*c4)*c5*c6; 
   m(2,1) = ((c2*c3-s2*s3)*c4-(c2*s3+s2*c3)*s4)*c6-((c2*c3-s2*s3)*s4+(c2*s3+s2*c3)*c4)*c5*s6; 
   m(2,2) = -((c2*c3-s2*s3)*s4+(c2*s3+s2*c3)*c4)*s5; 
   m(2,3) = (-LEN_D6*((c2*c3-s2*s3)*s4+(c2*s3+s2*c3)*c4)*s5)+LEN_D5*((c2*s3+s2*c3)*s4-(c2*c3-s2*s3)*c4)+LEN_A3*c2*s3+LEN_A3*s2*c3+LEN_A2*s2+LEN_D1; 
   m(3,0) = 0.0; 
   m(3,1) = 0.0; 
   m(3,2) = 0.0; 
   m(3,3) = 1.0;    
    
   return m;
}

// Find Jacobian using precalculated formulas
MatrixXd ur10::Jacobian(Matrix<double,JOINT_NO,1>& q) {
   // angles
   double c1 = cos(q(0)) , s1 = sin(q(0));
   double c2 = cos(q(1)) , s2 = sin(q(1));
   double c3 = cos(q(2)) , s3 = sin(q(2));
   double c4 = cos(q(3)) , s4 = sin(q(3));
   double c5 = cos(q(4)) , s5 = sin(q(4));
   double c6 = cos(q(5)) , s6 = sin(q(5));
   
   // initialization
   MatrixXd m(CART_NO,JOINT_NO);
   m(0,0) = (-LEN_D6*((-(((-s1*c2*s3)-s1*s2*c3)*s4+(s1*c2*c3-s1*s2*s3)*c4)*s5)-c1*c5))-LEN_D5*((s1*c2*c3-s1*s2*s3)*s4-((-s1*c2*s3)-s1*s2*c3)*c4)+LEN_A3*s1*s2*s3-LEN_A3*s1*c2*c3-LEN_A2*s1*c2+LEN_D4*c1; 
   m(0,1) = -c1*((-LEN_D6*((c2*c3-s2*s3)*s4+(c2*s3+s2*c3)*c4)*s5)+LEN_D5*((c2*s3+s2*c3)*s4-(c2*c3-s2*s3)*c4)+LEN_A3*c2*s3+LEN_A3*s2*c3+LEN_A2*s2); 
   m(0,2) = -c1*((-LEN_D6*((c2*c3-s2*s3)*s4+(c2*s3+s2*c3)*c4)*s5)+LEN_D5*((c2*s3+s2*c3)*s4-(c2*c3-s2*s3)*c4)+LEN_A3*c2*s3+LEN_A3*s2*c3); 
   m(0,3) = -c1*(LEN_D5*((c2*s3+s2*c3)*s4-(c2*c3-s2*s3)*c4)-LEN_D6*((c2*c3-s2*s3)*s4+(c2*s3+s2*c3)*c4)*s5); 
   m(0,4) = ((c2*c3-s2*s3)*c4-(c2*s3+s2*c3)*s4)*(LEN_D6*((-(((-s1*c2*s3)-s1*s2*c3)*s4+(s1*c2*c3-s1*s2*s3)*c4)*s5)-c1*c5)+LEN_D5*((s1*c2*c3-s1*s2*s3)*s4-((-s1*c2*s3)-s1*s2*c3)*c4))+((s1*c2*c3-s1*s2*s3)*s4-((-s1*c2*s3)-s1*s2*c3)*c4)*(LEN_D5*((c2*s3+s2*c3)*s4-(c2*c3-s2*s3)*c4)-LEN_D6*((c2*c3-s2*s3)*s4+(c2*s3+s2*c3)*c4)*s5); 
   m(0,5) = 0.0; 
   m(1,0) = LEN_D6*(s1*c5-(((-c1*c2*s3)-c1*s2*c3)*s4+(c1*c2*c3-c1*s2*s3)*c4)*s5)+LEN_D5*((c1*c2*c3-c1*s2*s3)*s4-((-c1*c2*s3)-c1*s2*c3)*c4)-LEN_A3*c1*s2*s3+LEN_A3*c1*c2*c3+LEN_A2*c1*c2+LEN_D4*s1; 
   m(1,1) = -s1*((-LEN_D6*((c2*c3-s2*s3)*s4+(c2*s3+s2*c3)*c4)*s5)+LEN_D5*((c2*s3+s2*c3)*s4-(c2*c3-s2*s3)*c4)+LEN_A3*c2*s3+LEN_A3*s2*c3+LEN_A2*s2); 
   m(1,2) = -s1*((-LEN_D6*((c2*c3-s2*s3)*s4+(c2*s3+s2*c3)*c4)*s5)+LEN_D5*((c2*s3+s2*c3)*s4-(c2*c3-s2*s3)*c4)+LEN_A3*c2*s3+LEN_A3*s2*c3); 
   m(1,3) = -s1*(LEN_D5*((c2*s3+s2*c3)*s4-(c2*c3-s2*s3)*c4)-LEN_D6*((c2*c3-s2*s3)*s4+(c2*s3+s2*c3)*c4)*s5); 
   m(1,4) = ((c2*s3+s2*c3)*s4-(c2*c3-s2*s3)*c4)*(LEN_D6*(s1*c5-(((-c1*c2*s3)-c1*s2*c3)*s4+(c1*c2*c3-c1*s2*s3)*c4)*s5)+LEN_D5*((c1*c2*c3-c1*s2*s3)*s4-((-c1*c2*s3)-c1*s2*c3)*c4))+(((-c1*c2*s3)-c1*s2*c3)*c4-(c1*c2*c3-c1*s2*s3)*s4)*(LEN_D5*((c2*s3+s2*c3)*s4-(c2*c3-s2*s3)*c4)-LEN_D6*((c2*c3-s2*s3)*s4+(c2*s3+s2*c3)*c4)*s5); 
   m(1,5) = (-LEN_D6*((c2*c3-s2*s3)*s4+(c2*s3+s2*c3)*c4)*s5*((((-c1*c2*s3)-c1*s2*c3)*s4+(c1*c2*c3-c1*s2*s3)*c4)*s5-s1*c5))-LEN_D6*((c2*c3-s2*s3)*s4+(c2*s3+s2*c3)*c4)*s5*(s1*c5-(((-c1*c2*s3)-c1*s2*c3)*s4+(c1*c2*c3-c1*s2*s3)*c4)*s5); 
   m(2,0) = 0.0; 
   m(2,1) = s1*(LEN_D6*((-(((-s1*c2*s3)-s1*s2*c3)*s4+(s1*c2*c3-s1*s2*s3)*c4)*s5)-c1*c5)+LEN_D5*((s1*c2*c3-s1*s2*s3)*s4-((-s1*c2*s3)-s1*s2*c3)*c4)-LEN_A3*s1*s2*s3+LEN_A3*s1*c2*c3+LEN_A2*s1*c2-LEN_D4*c1)+c1*(LEN_D6*(s1*c5-(((-c1*c2*s3)-c1*s2*c3)*s4+(c1*c2*c3-c1*s2*s3)*c4)*s5)+LEN_D5*((c1*c2*c3-c1*s2*s3)*s4-((-c1*c2*s3)-c1*s2*c3)*c4)-LEN_A3*c1*s2*s3+LEN_A3*c1*c2*c3+LEN_A2*c1*c2+LEN_D4*s1); 
   m(2,2) = s1*(LEN_D6*((-(((-s1*c2*s3)-s1*s2*c3)*s4+(s1*c2*c3-s1*s2*s3)*c4)*s5)-c1*c5)+LEN_D5*((s1*c2*c3-s1*s2*s3)*s4-((-s1*c2*s3)-s1*s2*c3)*c4)-LEN_A3*s1*s2*s3+LEN_A3*s1*c2*c3-LEN_D4*c1)+c1*(LEN_D6*(s1*c5-(((-c1*c2*s3)-c1*s2*c3)*s4+(c1*c2*c3-c1*s2*s3)*c4)*s5)+LEN_D5*((c1*c2*c3-c1*s2*s3)*s4-((-c1*c2*s3)-c1*s2*c3)*c4)-LEN_A3*c1*s2*s3+LEN_A3*c1*c2*c3+LEN_D4*s1); 
   m(2,3) = s1*(LEN_D6*((-(((-s1*c2*s3)-s1*s2*c3)*s4+(s1*c2*c3-s1*s2*s3)*c4)*s5)-c1*c5)+LEN_D5*((s1*c2*c3-s1*s2*s3)*s4-((-s1*c2*s3)-s1*s2*c3)*c4)-LEN_D4*c1)+c1*(LEN_D6*(s1*c5-(((-c1*c2*s3)-c1*s2*c3)*s4+(c1*c2*c3-c1*s2*s3)*c4)*s5)+LEN_D5*((c1*c2*c3-c1*s2*s3)*s4-((-c1*c2*s3)-c1*s2*c3)*c4)+LEN_D4*s1); 
   m(2,4) = ((c1*c2*c3-c1*s2*s3)*s4-((-c1*c2*s3)-c1*s2*c3)*c4)*(LEN_D6*((-(((-s1*c2*s3)-s1*s2*c3)*s4+(s1*c2*c3-s1*s2*s3)*c4)*s5)-c1*c5)+LEN_D5*((s1*c2*c3-s1*s2*s3)*s4-((-s1*c2*s3)-s1*s2*c3)*c4))+(((-s1*c2*s3)-s1*s2*c3)*c4-(s1*c2*c3-s1*s2*s3)*s4)*(LEN_D6*(s1*c5-(((-c1*c2*s3)-c1*s2*c3)*s4+(c1*c2*c3-c1*s2*s3)*c4)*s5)+LEN_D5*((c1*c2*c3-c1*s2*s3)*s4-((-c1*c2*s3)-c1*s2*c3)*c4)); 
   m(2,5) = LEN_D6*(s1*c5-(((-c1*c2*s3)-c1*s2*c3)*s4+(c1*c2*c3-c1*s2*s3)*c4)*s5)*((((-s1*c2*s3)-s1*s2*c3)*s4+(s1*c2*c3-s1*s2*s3)*c4)*s5+c1*c5)+LEN_D6*(s1*c5-(((-c1*c2*s3)-c1*s2*c3)*s4+(c1*c2*c3-c1*s2*s3)*c4)*s5)*((-(((-s1*c2*s3)-s1*s2*c3)*s4+(s1*c2*c3-s1*s2*s3)*c4)*s5)-c1*c5); 
   m(3,0) = 0.0; 
   m(3,1) = s1; 
   m(3,2) = s1; 
   m(3,3) = s1; 
   m(3,4) = (c1*c2*c3-c1*s2*s3)*s4-((-c1*c2*s3)-c1*s2*c3)*c4; 
   m(3,5) = s1*c5-(((-c1*c2*s3)-c1*s2*c3)*s4+(c1*c2*c3-c1*s2*s3)*c4)*s5; 
   m(4,0) = 0.0; 
   m(4,1) = -c1; 
   m(4,2) = -c1; 
   m(4,3) = -c1; 
   m(4,4) = (s1*c2*c3-s1*s2*s3)*s4-((-s1*c2*s3)-s1*s2*c3)*c4; 
   m(4,5) = (-(((-s1*c2*s3)-s1*s2*c3)*s4+(s1*c2*c3-s1*s2*s3)*c4)*s5)-c1*c5; 
   m(5,0) = 1.0; 
   m(5,1) = 0.0; 
   m(5,2) = 0.0; 
   m(5,3) = 0.0; 
   m(5,4) = (c2*s3+s2*c3)*s4-(c2*c3-s2*s3)*c4; 
   m(5,5) = -((c2*c3-s2*s3)*s4+(c2*s3+s2*c3)*c4)*s5; 
   return m;
}

Matrix<double,POSE_QUATERN,1> ur10::cartesianState(Matrix<double,JOINT_NO,1>& q) 
{
   Matrix4d fk = FK(q);
   Matrix3d rot = fk.block(0,0,3,3);
   Quaterniond v(rot);

   Matrix<double,POSE_QUATERN,1> res;
   res(0) = fk(0,3); res(1) = fk(1,3); res(2) = fk(2,3);

   res(3) = v.x(); res(4) = v.y(); res(5) = v.z(); res(6) = v.w();

   return res;
}

