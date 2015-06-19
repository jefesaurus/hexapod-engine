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
}

template<int n_joints> 
int LegController<n_joints>::GetJointCommands(Eigen::Vector3d point, double current_deadline, LegCommand<n_joints>* command) {
  double joint_angles[n_joints];
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
      command->joint_commands[i].velocity =fabs(model->joints[i].Theta() - joint_angles[i]) / max_eta;
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
  model->UpdateState(time_elapsed);
  current_time += time_elapsed;

  LegCommand<n_joints> command;

  if (deadline > 0 && path != NULL) {
    double progress = (current_time + time_elapsed)/deadline;
    // Finished
    if (progress > 1.0) {
      progress = 1.0;
      deadline = -1;
    }
    Eigen::Vector3d next_interpoint = path->Value(progress);
    //printf("%f, %f, %f\n", next_interpoint[0], next_interpoint[1], next_interpoint[2]);

    int solved = this->GetJointCommands(next_interpoint, time_elapsed, &command);
    if (solved == 0) {
      model->SetCommand(command);
    } else {
      printf("Infeasible\n");
    }
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
  model->Draw(to_global);

  // Draw the commanded path, if there is a commanded path.
  Eigen::Vector4d path_segs[path_draw_points];
  double r[path_draw_points];
  double g[path_draw_points];
  double b[path_draw_points];
  Eigen::Vector4d local_point;
  if (deadline > 0 && path != NULL) {
    double progress;
    for (int i = 0; i < path_draw_points; i++) {
      progress = (float)i/(path_draw_points - 1);
      local_point << path->Value(progress), 1;
      path_segs[i] = to_global * local_point;
      GetHeatMapColor(progress, &r[i], &g[i], &b[i]);
    }

    LineStrip(path_draw_points, path_segs, r, g, b);

    // Draw the point in time along the path where the leg should aim for
    progress = (float)(current_time)/deadline;
    local_point << path->Value(progress), 1;
    Eigen::Vector4d global_point = to_global * local_point;
    Point(global_point, 0.0, 1.0, 0.0);
  }
}

// Explicit instantiation to help out the confused compiler.
template class LegController<3>;
