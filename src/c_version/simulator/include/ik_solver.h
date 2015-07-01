#ifndef IK_SOLVER_H_
#define IK_SOLVER_H_

#include "kinematic_pair.h"

class IKSolver {
public:
  // Returns positive "score" if a solution was found, or 0 or a negative score if no solution was found.
  // More positive means further within the bounds of reachability.
  virtual double Solve(double x, double y, double z, double angles[], int num_angles)=0;
};

class IK3DoF : public IKSolver {
  double coxa_r, coxa_d, tibia_r, tibia_d, femur_r, femur_d;
  double total_d;
  double max_range_squared;
  double min_range_squared;

  // Angular extremes.
  double coxa_min, coxa_max;
  double femur_min, femur_max;
  double tibia_min, tibia_max;

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
    max_range_squared = (femur_r + tibia_r)*(femur_r + tibia_r);
    min_range_squared = (femur_r - tibia_r)*(femur_r - tibia_r);

    coxa_min = coxa.MinTheta();
    coxa_max = coxa.MaxTheta();
    femur_min = femur.MinTheta();
    femur_max = femur.MaxTheta();
    tibia_min = tibia.MinTheta();
    tibia_max = tibia.MaxTheta();
  };

  // Returns positive "score" if a solution was found, or 0 or a negative score if no solution was found.
  double Solve(double x, double y, double z, double angles[], int num_angles);
};

#endif // IK_SOLVER_H_
