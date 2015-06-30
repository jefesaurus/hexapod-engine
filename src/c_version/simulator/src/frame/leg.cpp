#include <string>
#include <iostream>
#include <stdio.h>

#include "leg.h"
#include "utils.h"

template<int n_joints> 
void LegController<n_joints>::SetState(double angles[n_joints]) {
  model->SetState(angles);
}

template<int n_joints> 
void LegController<n_joints>::SetControl(PathGen* _path, double _deadline) {
  path = _path;
  deadline = _deadline;
  current_time = 0; // Reset time.
  motion_complete = false;
}

template<int n_joints> 
int LegController<n_joints>::GetJointCommands(Eigen::Vector3d point, double current_deadline, LegCommand<n_joints>* command) {
  double joint_angles[n_joints];
  point = model->local_pose.ToFrame(point);
  int solved = ik_solver->Solve(point[0], point[1], point[2], joint_angles, n_joints);

  if (solved == 0) {
    double max_eta = fabs(model->joints[0].Theta() - joint_angles[0])/model->joints[0].MaxAngularVelocity();
    for (int i = 1; i < n_joints; i++) {
      max_eta = std::max(max_eta, fabs(model->joints[i].Theta() - joint_angles[i])/model->joints[i].MaxAngularVelocity());
    }

    if (max_eta < current_deadline) {
      max_eta = current_deadline;
    }

    // TODO do something clever with the Velocity IK so it goes in a line.
    for (int i = 0; i < n_joints; i++) {
      command->joint_commands[i].angle = joint_angles[i];
      command->joint_commands[i].velocity = fabs(model->joints[i].Theta() - joint_angles[i]) / max_eta;
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
void LegController<n_joints>::UpdateState(double time_elapsed, LegCommand<n_joints>* out_command) {
  // Update the state of the simulation
  current_time += time_elapsed;

  // Hold position if there is no controlled path
  if (path == NULL) {
    motion_complete = true;
    for (int i = 0; i < n_joints; i++) {
      out_command->joint_commands[i].angle = model->joints[i].Theta();
    }
    return;
  }

  LegCommand<n_joints> command;

  // Clamp progress to 1.0
  double progress = (current_time + time_elapsed)/deadline;
  if (progress > 1.0) {
    progress = 1.0;
  }

  // Get the next point in the path and attempt to head towards it.
  Eigen::Vector3d next_interpoint = path->Value(progress);
  int solved = GetJointCommands(next_interpoint, time_elapsed, &command);
  if (solved == 0) {
    *out_command = command;
    infeasible = false;
  } else {
    infeasible = true;
    // Hold position
    for (int i = 0; i < n_joints; i++) {
      out_command->joint_commands[i].angle = model->joints[i].Theta();
    }
  }

  if (!motion_complete &&
      current_time >= deadline && 
      (path->Value(1.0) - GetEndpoint()).squaredNorm() < destination_epsilon_squared) {
    motion_complete = true;
  }
}

template<int n_joints> 
Eigen::Vector3d LegController<n_joints>::GetEndpoint() {
  return Vector4dTo3d(model->ToGlobal());
}

static const int path_draw_points = 100;
template<int n_joints> 
void LegController<n_joints>::Draw(Eigen::Matrix4d to_global) {
  // Draw the simulated leg model.
  if (infeasible) {
    model->Draw(to_global, 1.0, 0.0, 0.0);
  } else {
    model->Draw(to_global);
  }

  // Draw the commanded path, if there is a commanded path.
  Eigen::Vector4d local_point;
  if (path != NULL) {
    path->Draw(to_global);

    // Draw the point in time along the path where the leg should aim for
    double progress = (float)(current_time)/deadline;
    local_point << path->Value(progress), 1;
    Eigen::Vector4d global_point = to_global * local_point;
    Point(global_point, 0.0, 1.0, 0.0);
  }
}

// Explicit instantiation to help out the confused compiler.
template class LegController<3>;
