#ifndef CHASSIS_H_
#define CHASSIS_H_

#include <cstdlib>
#include <vector>
#include <Eigen/Core>
#include <Eigen/LU>
#include <cmath>

#include "leg.h"
#include "drawing_primitives.h"
#include "interpolators.h"
#include "pose.h"

template <int n_legs, int n_joints>
class Chassis : public Drawable {
protected:
  Leg<n_joints> legs[n_legs];
  Pose local;

public:
  Chassis(Leg<n_joints> _legs[n_legs]) : local(0, 0, 0, 0, 0, 0) {
    for (int i = 0; i < n_legs; i++) {
      legs[i] = _legs[i];
    }
  }

  inline void SetState(int leg_index, double angles[n_joints]) {
    legs[leg_index].SetState(angles);
  }

  inline Eigen::Vector4d LocalToGlobal(Eigen::Vector4d in) const {
    return local.FromFrame(in);
  }

  inline const Pose& GetPose() const {
    return local;
  }
  inline void SetPose(Pose pose) {
    local = pose;
  }

  void SetCommand(LegCommand<n_joints> leg_commands) {
    for (int i = 0; i < n_legs; i++) {
      legs[i].SetCommand(leg_commands[i]);
    }
  }

  void UpdateState(double time_elapsed) {
    for (int i = 0; i < n_legs; i++) {
      legs[i].UpdateState(time_elapsed);
    }
  }

  void Draw(Eigen::Matrix4d to_global) {
    to_global *= local.FromFrameMat();
    for (int i = 0; i < n_legs; i++) {
      legs[i].Draw(to_global);
    }
  }
};


template <int n_legs, int n_joints>
class ChassisController : public Drawable {
  // The simulated model
  Chassis<n_legs, n_joints> model;

  // Individual leg controllers.
  LegController<n_joints> leg_controllers[n_legs];

  // The pose path for the origin of the chassis.
  std::unique_ptr<PoseGen> chassis_path;
  std::unique_ptr<PathGen> leg_paths[n_legs];

public:
  ChassisController(Leg<n_joints> legs[n_legs], std::unique_ptr<IKSolver> solvers[n_legs]) : model(legs) {
    // Create the controllers, giving them references to the internal simulation model and the provided solvers.
    for (int i = 0; i < n_legs; i++) {
      leg_controllers[i] = LegController<n_joints>(&legs[i], std::move(solvers[i])); 
    }
  }

  void SetPose(Pose pose) {
    model.SetPose(pose);
  }

  void SetControl(std::unique_ptr<PoseGen> path) {
    // TODO reparametrize accounting for arc length to get uniform speed
    chassis_path = std::move(path);
  }

  void UpdateState(double time_elapsed) {
    model.UpdateState(time_elapsed);

    /*
    LegCommand<n_joints> leg_commands;
    for (int i = 0; i < n_legs; i++) {
      leg_controllers[i].UpdateState(time_elapsed, &leg_commands[i]);
    }

    model.SetCommand(leg_commands);
    */
  }

  void Draw(Eigen::Matrix4d to_global) {
    model.Draw(to_global);
    to_global *= model.GetPose().FromFrameMat();
    if (chassis_path != nullptr) {
      chassis_path->Draw(to_global);
    }
  }
};

#endif // CHASSIS_H_
