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


/*
void LegController::SetCommand() {
}

void LegController::GetJointCommands() {
}

void LegController::UpdateState(time_passed) {
}
*/
