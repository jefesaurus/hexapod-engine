#include "kinematic_pair.h"

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
  params_changed = params_changed || (_theta != theta);
  theta = _theta;
}

void RevoluteJoint::SetCommand(double _dest, double _vel) {
  destination = _dest;
  velocity = _vel;
  is_moving = (destination != theta);
}

void RevoluteJoint::UpdateState(double time_elapsed) {
  if (is_active) {
    double diff = destination - theta;
    if (fabs(diff) > 0.0) {
      double delta = std::copysign(velocity*time_elapsed, diff);
      if (fabs(delta) < fabs(diff)) {
        SetTheta(theta + delta);
      } else {
        SetTheta(destination);
      }
    } else {
      is_moving = false;
    }
  }
}
