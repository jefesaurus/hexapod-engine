#include <algorithm>
#include "ik_solver.h"


// 0: Coxa, 1: Femur, 2: Tibia
// TODO: Get check feasibility more exactly, and check all possible solutions.
// Negative is infeasible. 0 is the edge of feasibility. Positive is feasible.
double IK3DoF::Solve(double x, double y, double z, double angles[], int num_angles) {
  double coxa_angle = 0.0;
  // Start by angling directly to the point with the coxa
  if (!(x == 0 && y == 0)) {
    coxa_angle = atan2(y, x);

    // If there is some combined d offset, adjust for it.
    coxa_angle += asin(total_d/sqrt(x*x + y*y));
  }


  // Transform the points to the coordinates at the end of the coxa.
  double xn = x*cos(coxa_angle) + y*sin(coxa_angle) - coxa_r*(pow(sin(coxa_angle), 2) + pow(cos(coxa_angle), 2));
  double yn = z - coxa_d;
  //double zn = x*sin(coxa_angle) - y*cos(coxa_angle);
  double dist = xn*xn + yn*yn;

  if (coxa_angle < coxa_min) {
    return (coxa_angle - coxa_min)*dist;
  } else if (coxa_angle > coxa_max) {
    return (coxa_max - coxa_angle)*dist;
  }

  double target_dir = atan2(yn, xn);

  if (dist > max_range_squared) { // Too far
    return sqrt(max_range_squared) - sqrt(dist);
  } else if (dist < min_range_squared) {// Too close
    return sqrt(dist) - sqrt(min_range_squared);
  } 

  double theta_a = acos((pow(femur_r,2) + pow(tibia_r,2) - dist)/(2*femur_r * tibia_r));
  double theta_b = acos((pow(femur_r,2) + dist - pow(tibia_r,2))/(2*femur_r * sqrt(dist)));

  double femur_angle = target_dir + theta_b;
  double tibia_angle = theta_a - M_PI;
  if ((femur_angle < femur_min || femur_angle > femur_max) ||
      (tibia_angle < tibia_min || tibia_angle > tibia_max)) {
    femur_angle = target_dir - theta_b;
    tibia_angle = M_PI - theta_a;
    if ((femur_angle < femur_min || femur_angle > femur_max) ||
        (tibia_angle < tibia_min || tibia_angle > tibia_max)) {
      return -dist;
    }
  }
  // TODO try alternate solution: femur(target_dir - theta_b), tibia(M_PI - theta_a)
  angles[0] = coxa_angle;
  angles[1] = femur_angle;
  angles[2] = tibia_angle;
  return std::min(sqrt(dist) - sqrt(min_range_squared), sqrt(max_range_squared) - sqrt(dist));
}

