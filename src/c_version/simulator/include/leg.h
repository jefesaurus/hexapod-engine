#ifndef LEG_H
#define LEG_H

#include <cstdlib>
#include <vector>
#include <Eigen/Core>
#include <Eigen/LU>
#include <cmath>

#include "kinematic_pair.h"
#include "drawing_primitives.h"
#include "ik_solver.h"



// Leg representation made up of only revolute joints.
template <int n_joints> class Leg : public Drawable {
protected:
  RevoluteJoint joints[n_joints];

public:

  Leg(RevoluteJoint _joints[n_joints]) {
    DrawLock();
    for (int i = 0; i < n_joints; i++) {
      joints[i] = _joints[i];
    }
    DrawUnlock();
  }

  void SetState(double angles[n_joints]) {
    DrawLock();
    for (int i = 0; i < n_joints; i++) {
      joints[i].SetTheta(angles[i]);
    }
    DrawUnlock();
  }

  inline const RevoluteJoint* Joint(int i) const { return &joints[i]; };

  // Play the leg through in time toward its commanded destination.
  void UpdateState(double time_elapsed) {
    DrawLock();
    for (int i = 0; i < n_joints; i++) {
      joints[i].UpdateState(time_elapsed);
    }
    DrawUnlock();
  }

  // Propagate a set of commands to each of the joints.
  void SetJointCommands(double angles[n_joints], double velocities[n_joints]) {
    for (int i = 0; i < n_joints; i++) {
      joints[i].SetCommand(angles[i], velocities[i]);
    }
  }

  bool IsMoving() {
    for (int i = 0; i < n_joints; i++) {
      if (joints[i].IsMoving()) {
        return true;
      }
    }
    return false;
  }

  Eigen::Vector4d ToGlobal(Eigen::Vector4d in) {
    Eigen::Matrix<double, 4, 4> compound = joints[0].DHMat();
    for (int i = 1; i < n_joints; i++) {
      compound *= joints[i].DHMat();
    }
    return compound * in;
  }

  // If you don't specify a point, it defaults to the origin.
  inline Eigen::Vector4d ToGlobal() {
    return ToGlobal(Eigen::Vector4d(0.0, 0.0, 0.0, 1.0));
  }; 

  // Returns the origins of each of the segments in the global coordinate system
  void AllOrigins(Eigen::Vector4d points[n_joints + 1]) {
    DrawLock();
    Eigen::Vector4d origin(0.0, 0.0, 0.0, 1.0);
    points[0] = origin;

    Eigen::Matrix4d compound(joints[0].DHMat());
    for (int i = 1; i < n_joints; i++) {
      points[i] = compound * origin;
      compound *= joints[i].DHMat();
    }
    points[n_joints] = compound * origin;
    DrawUnlock();
  }

  // Returns the basic structure of each of the joints.
  // SLightly More detailed version of AllOrigins.
  void AllJoints(Eigen::Vector4d points[2*n_joints]) {
    DrawLock();
    Eigen::Vector4d origin(0.0, 0.0, 0.0, 1.0);
    Eigen::Vector4d offset(0.0, 0.0, 0.0, 1.0);
    Eigen::Matrix4d compound(Eigen::Matrix4d::Identity(4, 4));

    for (int i = 0; i < n_joints; i++) {
      compound *= joints[i].DHMat();
      offset[0] = -joints[i].R();
      points[2*i] = compound * offset;
      points[2*i + 1] = compound * origin;
    }
    DrawUnlock();
  }

  // Implement the drawable interface.
  void Draw() {
    int strip_size = 2*n_joints;
    Eigen::Vector4d segs[strip_size];
    AllJoints(segs);
    LineStrip(strip_size, segs, 1.0, 0.0, 0.0);
  }
};


// Class to have more fine tuned control over the exact paths taken to a destination.
template <int n_joints> class LegController : public Leg<n_joints> {
protected:
  IKSolver* ik_solver;
  Eigen::Vector3d dest;
  double deadline, current_time;

public:
  LegController(RevoluteJoint _joints[n_joints], IKSolver* ik_solver) : Leg<n_joints>(_joints), ik_solver(ik_solver) {}; 

  void SetCommand(Eigen::Vector3d goal, double deadline);
  int GetJointCommands(Eigen::Vector3d point, double joint_angles[n_joints], double joint_speeds[n_joints]);
  int GetJointCommands(Eigen::Vector3d point, double joint_angles[n_joints]);
  void UpdateState(double time_elapsed);
};

#endif // LEG_H
