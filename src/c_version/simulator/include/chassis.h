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



template <int n_legs, int n_joints> class Chassis : public Drawable {
protected:
  Leg<n_joints> legs[n_legs];
  Pose leg_poses[n_legs];
  Pose local;

  Chassis(Leg<n_joints> legs, Pose leg_poses) : legs(legs), leg_poses(leg_poses), local(0, 0, 0, 0, 0, 0) {};

  inline Eigen::Vector4d LocalToGlobal(Eigen::Vector4d in) const {
    return local.FromFrame(in);
  }

  inline Eigen::Vector4d LegToGlobal(int leg_index, Eigen::Vector4d in) const {
    return local.FromFrame(leg_poses[leg_index].FromFrame(in));
  }

  inline Eigen::Vector4d GlobalToLeg(int leg_index, Eigen::Vector4d in) const {
    return leg_poses[leg_index].ToFrame(local.ToFrame(in));
  }

  // Commands are in Leg-major order. (joints in the same leg are adjacent)
  void SetCommand(LegCommand<n_joints> leg_commands) {
    for (int i = 0; i < n_legs; i++) {
      legs[i].SetCommand(leg_commands[i]);
    }
  }
  void UpdateState(double time_elapsed);

};

#endif // CHASSIS_H_
