#include "ik_solver.h"


// 0: Coxa, 1: Femur, 2: Tibia
// TODO: Get check feasibility more exactly, and check all possible solutions.
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

  double target_dir = atan2(yn, xn);

  if (dist > max_range_squared) { // Too far
    return 1;
  } else if (dist < min_range_squared) {// Too close
    return 1;
  } 

  double theta_a = acos((pow(femur_r,2) + pow(tibia_r,2) - dist)/(2*femur_r * tibia_r));
  double theta_b = acos((pow(femur_r,2) + dist - pow(tibia_r,2))/(2*femur_r * sqrt(dist)));

  angles[0] = coxa_angle;
  angles[1] = target_dir + theta_b;
  angles[2] = theta_a - M_PI;
  return 0;
}

