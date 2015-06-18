#ifndef IK_SOLVER_H_
#define IK_SOLVER_H_

#include "kinematic_pair.h"

class IKSolver {
public:
  // Returns 0 if solved, 1 if couldn't find a solution.
  virtual int Solve(double x, double y, double z, double angles[], int num_angles)=0;
};

class IK3DoF : public IKSolver {
  double coxa_r, coxa_d, tibia_r, tibia_d, femur_r, femur_d;
  double total_d;
  double max_range_squared;
  double min_range_squared;

public:
  IK3DoF();
  IK3DoF(RevoluteJoint coxa, RevoluteJoint femur, RevoluteJoint tibia) {
    coxa_r = coxa.R();
    coxa_d = coxa.D();
    tibia_r = tibia.R();
    tibia_d = tibia.D();
    femur_r = femur.R();
    femur_d = femur.D();
    total_d = femur_d + tibia_d;
    max_range_squared = pow(femur_r + tibia_r, 2);
    min_range_squared = pow(femur_r - tibia_r, 2);
  };

  // Returns 0 if it found a solution, something else if it couldn't.
  int Solve(double x, double y, double z, double angles[], int num_angles);
};

#endif // IK_SOLVER_H_
