#include <algorithm>
#include "ik_solver.h"


// 0: Coxa, 1: Femur, 2: Tibia
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

  double coxa_score = std::min((coxa_angle - coxa_min), (coxa_max - coxa_angle))*sqrt(x*x + y*y + z*z);
  double range_score = std::min(sqrt(max_range_squared) - sqrt(dist), sqrt(dist) - sqrt(min_range_squared));
  if (range_score < 0) {
    return std::min(coxa_score, range_score);
  }

  double theta_a = acos((femur_r*femur_r + tibia_r*tibia_r - dist)/(2*femur_r * tibia_r));
  double theta_b = acos((femur_r*femur_r + dist - tibia_r*tibia_r)/(2*femur_r * sqrt(dist)));

  double target_dir = atan2(yn, xn);

  double femur_angle_a = target_dir + theta_b;
  double tibia_angle_a = theta_a - M_PI;
  double femur_score_a = std::min(femur_angle_a - femur_min, femur_max - femur_angle_a)*sqrt(dist);
  double tibia_score_a = std::min(tibia_angle_a - tibia_min, tibia_max - tibia_angle_a)*sqrt(dist);
  double femur_angle_b = target_dir - theta_b;
  double tibia_angle_b = M_PI - theta_a;
  double femur_score_b = std::min(femur_angle_b - femur_min, femur_max - femur_angle_b)*sqrt(dist);
  double tibia_score_b = std::min(tibia_angle_b - tibia_min, tibia_max - tibia_angle_b)*sqrt(dist);

  double final_score = std::min(range_score, coxa_score);
  if (std::min(femur_score_a, tibia_score_a) > std::min(femur_score_b, tibia_score_b)) {
    final_score = std::min(std::min(final_score, femur_score_a), tibia_score_a);
    if (final_score > 0) {
      angles[0] = coxa_angle;
      angles[1] = femur_angle_a;
      angles[2] = tibia_angle_a;
    }
  } else {
    final_score = std::min(std::min(final_score, femur_score_b), tibia_score_b);
    if (final_score > 0) {
      angles[0] = coxa_angle;
      angles[1] = femur_angle_b;
      angles[2] = tibia_angle_b;
    }
  }
  return final_score;

  // TODO Checkout the other two solutions on the opposite coxa angle :)
}

