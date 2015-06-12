#include <string>
#include <iostream>
#include <stdio.h>

#include "leg.h"


template<int n_joints> 
void LegController<n_joints>::SetCommand(PathGen<Eigen::Vector3d>* _path, double _deadline) {
  path = _path;
  deadline = _deadline;
  current_time = 0; // Reset time.
}

template<int n_joints> 
int LegController<n_joints>::GetJointCommands(Eigen::Vector3d point, double joint_angles[n_joints], double joint_speeds[n_joints]) {
  int solved = ik_solver->Solve(point[0], point[1], point[2], joint_angles, n_joints);
  if (solved == 0) {
    double max_eta = fabs(this->joints[0].Theta() - joint_angles[0])/this->joints[0].MaxAngularVelocity();
    for (int i = 1; i < n_joints; i++) {
      max_eta = std::max(max_eta, fabs(this->joints[i].Theta() - joint_angles[i])/this->joints[i].MaxAngularVelocity());
    }

    if (max_eta < deadline) {
      max_eta = deadline - current_time;
    }

    // TODO do something clever with the Velocity IK so it goes in a line.
    for (int i = 0; i < n_joints; i++) {
      joint_speeds[i] = fabs(this->joints[i].Theta() - joint_angles[i]) / max_eta;
    }
  }
  return solved;
}

// Doesn't bother with joint speeds. Just an accessor for the ik solver.
template<int n_joints> 
int LegController<n_joints>::GetJointCommands(Eigen::Vector3d point, double joint_angles[n_joints]) {
  int solved = ik_solver->Solve(point[0], point[1], point[2], joint_angles, n_joints);
  return solved;
}

template<int n_joints> 
void LegController<n_joints>::UpdateState(double time_elapsed) {
  // Update the state of the simulation
  this->Leg<n_joints>::UpdateState(time_elapsed);
  current_time += time_elapsed;

  double joint_angles[n_joints];
  double joint_speeds[n_joints];

  if (deadline > 0 && path != NULL) {
    double progress = (current_time + time_elapsed)/deadline;
    // Finished
    if (progress > 1.0) {
      progress = 1.0;
      deadline = -1;
    }
    Eigen::Vector3d next_interpoint = path->Value(progress);

    int solved = this->GetJointCommands(next_interpoint, joint_angles, joint_speeds);
    if (solved == 0) {
      this->SetJointCommands(joint_angles, joint_speeds);
    } else {
      printf("Infeasible\n");
    }
  }

  /*
  int solved = this->GetJointCommands(dest, joint_angles, joint_speeds);
  if (solved != 0) {
    printf("Infeasible\n");
  }
  this->SetJointCommands(joint_angles, joint_speeds);
  */
}

// Explicit instantiation to help out the confused compiler.
template class LegController<3>;
