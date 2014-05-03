#include <stdio.h>
#include <math.h>
#include <stdlib.h>

typedef struct point point;
struct point {
  double x;
  double y;
  double z;
};

extern point fk_math(double x, double y, double z, double, double, double);
extern point fk_math_cache(double x, double y, double z, double, double, double);
extern void trial();

point fk_math_cache(double x, double y, double z, double theta_coxa, double theta_femur, double theta_tibia) {
  double sin_coxa = sin(theta_coxa);
  double cos_coxa = cos(theta_coxa);
  double sin_femur = sin(theta_femur);
  double cos_femur = cos(theta_femur);
  double sin_tibia = sin(theta_tibia);
  double cos_tibia = cos(theta_tibia);
  return (point){x*(-sin_femur*sin_tibia*cos_coxa + cos_coxa*cos_femur*cos_tibia) + y*(-sin_femur*cos_coxa*cos_tibia - sin_tibia*cos_coxa*cos_femur) + 1.0*z*sin_coxa - 2.0*sin_femur*sin_tibia*cos_coxa + 2.0*cos_coxa*cos_femur*cos_tibia + 1.5*cos_coxa*cos_femur + 0.5*cos_coxa,
  x*(-sin_coxa*sin_femur*sin_tibia + sin_coxa*cos_femur*cos_tibia) + y*(-sin_coxa*sin_femur*cos_tibia - sin_coxa*sin_tibia*cos_femur) - 1.0*z*cos_coxa - 2.0*sin_coxa*sin_femur*sin_tibia + 2.0*sin_coxa*cos_femur*cos_tibia + 1.5*sin_coxa*cos_femur + 0.5*sin_coxa,
  x*(1.0*sin_femur*cos_tibia + 1.0*sin_tibia*cos_femur) + y*(-1.0*sin_femur*sin_tibia + 1.0*cos_femur*cos_tibia) + 2.0*sin_femur*cos_tibia + 1.5*sin_femur + 2.0*sin_tibia*cos_femur};
}

void trial() {
  int i;
  for (i = 0; i < 1000000; i ++) {
    fk_math_cache(1.0, 3.0, 4.0, -2.09439510239, 1.0471975512, 0.78539816339);
  }
}

point fk_math(double x, double y, double z, double theta_coxa, double theta_femur, double theta_tibia) {
  return (point){x*(-sin(theta_femur)*sin(theta_tibia)*cos(theta_coxa) + cos(theta_coxa)*cos(theta_femur)*cos(theta_tibia)) + y*(-sin(theta_femur)*cos(theta_coxa)*cos(theta_tibia) - sin(theta_tibia)*cos(theta_coxa)*cos(theta_femur)) + 1.0*z*sin(theta_coxa) - 2.0*sin(theta_femur)*sin(theta_tibia)*cos(theta_coxa) + 2.0*cos(theta_coxa)*cos(theta_femur)*cos(theta_tibia) + 1.5*cos(theta_coxa)*cos(theta_femur) + 0.5*cos(theta_coxa),
  x*(-sin(theta_coxa)*sin(theta_femur)*sin(theta_tibia) + sin(theta_coxa)*cos(theta_femur)*cos(theta_tibia)) + y*(-sin(theta_coxa)*sin(theta_femur)*cos(theta_tibia) - sin(theta_coxa)*sin(theta_tibia)*cos(theta_femur)) - 1.0*z*cos(theta_coxa) - 2.0*sin(theta_coxa)*sin(theta_femur)*sin(theta_tibia) + 2.0*sin(theta_coxa)*cos(theta_femur)*cos(theta_tibia) + 1.5*sin(theta_coxa)*cos(theta_femur) + 0.5*sin(theta_coxa),
  x*(1.0*sin(theta_femur)*cos(theta_tibia) + 1.0*sin(theta_tibia)*cos(theta_femur)) + y*(-1.0*sin(theta_femur)*sin(theta_tibia) + 1.0*cos(theta_femur)*cos(theta_tibia)) + 2.0*sin(theta_femur)*cos(theta_tibia) + 1.5*sin(theta_femur) + 2.0*sin(theta_tibia)*cos(theta_femur)};
}
