#include <string>
#include <iostream>
#include <stdio.h>

#include "kinematic_chain.h"



void KinematicPair::GenerateDHMatrices() {
  dh_mat << cos(theta), -sin(theta)*cos(alpha), sin(theta)*sin(alpha), r*cos(theta),
            sin(theta), cos(theta)*cos(alpha), -cos(theta)*sin(alpha), r*sin(theta),
            0, sin(alpha), cos(alpha), d,
            0, 0, 0, 1;
  
  // Set the rotation part of the inverse matrix to the tranpose from the forward matrix
  inv_dh_mat.block<3,3>(0, 0) = dh_mat.block<3,3>(0,0).transpose();

  // Set the translation part.
  inv_dh_mat.block<3, 1>(0, 3) = - inv_dh_mat.block<3, 3>(0, 0) * dh_mat.block<3, 1>(0, 3);
  inv_dh_mat.block<1, 4>(3, 0) << 0, 0, 0, 1;
}


void RevoluteJoint::SetTheta(double _theta) {
  theta = _theta;
  params_changed = true;
}


void Leg::SetState(double coxa_angle, double femur_angle, double tibia_angle) {
  coxa.SetTheta(coxa_angle);
  femur.SetTheta(femur_angle);
  tibia.SetTheta(tibia_angle);
}

Eigen::Vector4d Leg::ToGlobal(Eigen::Vector4d in) {
  Eigen::Matrix<double, 4, 4> compound = coxa.DHMat() * femur.DHMat() * tibia.DHMat();
  return compound * in;
}

void Leg::AllSegments(Eigen::Vector4d points[4]) {
  Eigen::Vector4d origin(0.0, 0.0, 0.0, 1.0);
  Eigen::Matrix<double, 4, 4> compound = coxa.DHMat();

  points[0] = origin;
  points[1] = compound * origin;
  compound *= femur.DHMat();
  points[2] = compound * origin;
  compound *= tibia.DHMat();
  points[3] = compound * origin;
}

/*
int main() {
  RevoluteJoint coxa(M_PI/2.0, .5, 0.0);
  RevoluteJoint femur(0.0, 1.5, 0.0);
  RevoluteJoint tibia(0.0, 2.0, 0.0);

  Leg front_right(coxa, femur, tibia);
  Eigen::Vector4d segs[4];
  front_right.AllSegments(segs);
  front_right.SetState(0.0, M_PI/4.0, -M_PI/4.0);

  return 0;
}
*/
