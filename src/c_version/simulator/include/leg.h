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
private:
  RevoluteJoint joints[n_joints];

public:

  Leg(RevoluteJoint _joints[n_joints]) {
    for (int i = 0; i < n_joints; i++) {
      joints[i] = _joints[i];
    }
  }

  void SetState(double angles[n_joints]) {
    for (int i = 0; i < n_joints; i++) {
      joints[i].SetTheta(angles[i]);
    }
  }

  // Play the leg through in time toward its commanded destination.
  void UpdateState(double time_elapsed) {
    for (int i = 0; i < n_joints; i++) {
      joints[i].UpdateState(time_elapsed);
    }
  }

  // Propagate a set of commands to each of the joints.
  void SetJointCommands(double angles[n_joints], double velocities[n_joints]) {
    for (int i = 0; i < n_joints; i++) {
      joints[i].SetCommand(angles[i], velocities[i]);
    }
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
    Eigen::Vector4d origin(0.0, 0.0, 0.0, 1.0);
    points[0] = origin;

    Eigen::Matrix<double, 4, 4> compound = joints[0].DHMat();
    for (int i = 1; i < n_joints; i++) {
      points[i] = compound * origin;
      compound *= joints[i].DHMat();
    }
    points[n_joints] = compound * origin;
  }

  // Returns the basic structure of each of the joints.
  // SLightly More detailed version of AllOrigins.
  void AllJoints(Eigen::Vector4d points[2*n_joints]) {
    Eigen::Vector4d origin(0.0, 0.0, 0.0, 1.0);
    Eigen::Vector4d offset(0.0, 0.0, 0.0, 1.0);

    Eigen::Matrix4d compound = Eigen::Matrix4d::Identity(4, 4);

    for (int i = 0; i < n_joints; i++) {
      compound *= joints[i].DHMat();
      offset[0] = -joints[i].R();
      points[2*i] = compound * offset;
      points[2*i + 1] = compound * origin;
    }
  }

  // Implement the drawable interface.
  void Draw() {
    int strip_size = 2*n_joints;
    Eigen::Vector4d segs[strip_size];
    AllJoints(segs);
    LineStrip(strip_size, segs, 1.0, 0.0, 0.0);
  }
};

template <int n_joints> class LegController {
  Leg<n_joints> leg_model;
  IKSolver* ik_solver;
  double goal_x, goal_y, goal_z;
  double deadline, current_time;
public:
  LegController(Leg<n_joints> model, IKSolver* ik_solver) : leg_model(model), ik_solver(ik_solver) {}; 

  void SetCommand(double x, double y, double z, double deadline);
  void GetJointCommands(double x, double y, double z, double joint_angles[n_joints], double joint_speeds[n_joints]);
  void UpdateState(double time_elapsed);
};

#endif // LEG_H
