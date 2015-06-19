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
  Pose leg_poses[n_legs];
  Pose local;

public:
  Chassis(Leg<n_joints> _legs[n_legs], Pose _leg_poses[n_legs]) : local(0, 0, 0, 0, 0, 0) {
    for (int i = 0; i < n_legs; i++) {
      legs[i] = _legs[i];
      leg_poses[i] = _leg_poses[i];
    }
  }

  inline void SetState(int leg_index, double angles[n_joints]) {
    legs[leg_index].SetState(angles);
  }

  inline Eigen::Vector4d LocalToGlobal(Eigen::Vector4d in) const {
    return local.FromFrame(in);
  }

  inline Eigen::Vector4d LegToGlobal(int leg_index, Eigen::Vector4d in) const {
    return local.FromFrame(leg_poses[leg_index].FromFrame(in));
  }

  inline Eigen::Vector4d GlobalToLeg(int leg_index, Eigen::Vector4d in) const {
    return leg_poses[leg_index].ToFrame(local.ToFrame(in));
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
    Eigen::Matrix4d leg_to_global;
    for (int i = 0; i < n_legs; i++) {
      leg_to_global = to_global * leg_poses[i].FromFrameMat();
      legs[i].Draw(leg_to_global);
    }
  }
};


template <int n_legs, int n_joints>
class ChassisController {
  // The simulated model
  Chassis<n_legs, n_joints> model;

  // Individual leg controllers.
  LegController<n_joints> leg_controllers[n_legs];
public:
  ChassisController(Leg<n_joints> legs[n_legs], Pose leg_poses[n_legs], std::unique_ptr<IKSolver> solvers[n_legs]) : model(legs, leg_poses) {
    for (int i = 0; i < n_legs; i++) {
      leg_controllers[i](legs[i], std::move(solvers[i])); 
    }
  };
};

#endif // CHASSIS_H_
