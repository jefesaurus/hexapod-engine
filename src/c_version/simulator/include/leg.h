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

  // Must have number of segments + 1 points
  void AllSegments(Eigen::Vector4d points[n_joints + 1]) {
    Eigen::Vector4d origin(0.0, 0.0, 0.0, 1.0);
    points[0] = origin;

    Eigen::Matrix<double, 4, 4> compound = joints[0].DHMat();
    for (int i = 1; i < n_joints; i++) {
      points[i] = compound * origin;
      compound *= joints[i].DHMat();
    }
    points[n_joints] = compound * origin;
  }

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
    //int strip_size = 1*n_joints + 1;
    int strip_size = 2*n_joints;
    Eigen::Vector4d segs[strip_size];
    //AllSegments(segs);
    AllJoints(segs);
    LineStrip(strip_size, segs, 1.0, 0.0, 0.0);
  }
};

template <int n_joints> class LegController {
  Leg<n_joints> leg_model;
  IKSolver* ik_solver;
public:
  LegController(Leg<n_joints> model, IKSolver* ik_solver) : leg_model(model), ik_solver(ik_solver) {}; 
};

#endif // LEG_H
