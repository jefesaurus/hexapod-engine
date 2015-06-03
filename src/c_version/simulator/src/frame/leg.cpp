#include <string>
#include <iostream>
#include <stdio.h>

#include "leg.h"


template<int n_joints> 
void LegController<n_joints>::SetCommand(double x, double y, double z, double _deadline) {
  goal_x = x;
  goal_y = y;
  goal_z = z;
  deadline = _deadline;
  current_time = 0; // Reset time.
}

template<int n_joints> 
void LegController<n_joints>::GetJointCommands(double x, double y, double z, double joint_angles[n_joints], double joint_speeds[n_joints]) {
  // TODO Pass two points instead of just one of it can keep moving if it is too close to the destination.
  // Get IK solution
  // Estimate completion times and scale everything by whatever will take longest.
  

}

template<int n_joints> 
void LegController<n_joints>::UpdateState(double time_elapsed) {
  // Update the state of the simulation
  current_time += time_elapsed;


  // Get the new joint commands to stay on schedule.


  //double progress = (current_time + interval_estimate)/deadline;
}

// Explicit instantiation to help out the confused compiler.
template class LegController<3>;
