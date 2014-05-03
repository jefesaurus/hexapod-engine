#include <stdio.h>
#include <math.h>
#include <stdlib.h>

typedef struct point point;
struct point {
  double x;
  double y;
  double z;
};


extern point func(point in);
extern point fk_math(double x, double y, double z, double, double, double);

point func(point in) {
  return (point){in.x + 1, in.y + 1, in.z + 1};
}

/*
[x*(-sin(theta_femur)*sin(theta_tibia)*cos(theta_coxa) + cos(theta_coxa)*cos(theta_femur)*cos(theta_tibia)) + y*(-sin(theta_femur)*cos(theta_coxa)*cos(theta_tibia) - sin(theta_tibia)*cos(theta_coxa)*cos(theta_femur)) + 1.0*z*sin(theta_coxa) - 2.0*sin(theta_femur)*sin(theta_tibia)*cos(theta_coxa) + 2.0*cos(theta_coxa)*cos(theta_femur)*cos(theta_tibia) + 1.5*cos(theta_coxa)*cos(theta_femur) + 0.5*cos(theta_coxa)]

[x*(-sin(theta_coxa)*sin(theta_femur)*sin(theta_tibia) + sin(theta_coxa)*cos(theta_femur)*cos(theta_tibia)) + y*(-sin(theta_coxa)*sin(theta_femur)*cos(theta_tibia) - sin(theta_coxa)*sin(theta_tibia)*cos(theta_femur)) - 1.0*z*cos(theta_coxa) - 2.0*sin(theta_coxa)*sin(theta_femur)*sin(theta_tibia) + 2.0*sin(theta_coxa)*cos(theta_femur)*cos(theta_tibia) + 1.5*sin(theta_coxa)*cos(theta_femur) + 0.5*sin(theta_coxa)]

[x*(1.0*sin(theta_femur)*cos(theta_tibia) + 1.0*sin(theta_tibia)*cos(theta_femur)) + y*(-1.0*sin(theta_femur)*sin(theta_tibia) + 1.0*cos(theta_femur)*cos(theta_tibia)) + 2.0*sin(theta_femur)*cos(theta_tibia) + 1.5*sin(theta_femur) + 2.0*sin(theta_tibia)*cos(theta_femur)]
*/

point fk_math_cache(double x, double y, double z, double theta_coxa, double theta_femur, double theta_tibia) {
  double sin_coxa
  double cos_coxa
  double sin_femur
  double cos_femur
  double sin_tibia
  double cos_tibia
  return (point){x*(-sin(theta_femur)*sin(theta_tibia)*cos(theta_coxa) + cos(theta_coxa)*cos(theta_femur)*cos(theta_tibia)) + y*(-sin(theta_femur)*cos(theta_coxa)*cos(theta_tibia) - sin(theta_tibia)*cos(theta_coxa)*cos(theta_femur)) + 1.0*z*sin(theta_coxa) - 2.0*sin(theta_femur)*sin(theta_tibia)*cos(theta_coxa) + 2.0*cos(theta_coxa)*cos(theta_femur)*cos(theta_tibia) + 1.5*cos(theta_coxa)*cos(theta_femur) + 0.5*cos(theta_coxa),
  x*(-sin(theta_coxa)*sin(theta_femur)*sin(theta_tibia) + sin(theta_coxa)*cos(theta_femur)*cos(theta_tibia)) + y*(-sin(theta_coxa)*sin(theta_femur)*cos(theta_tibia) - sin(theta_coxa)*sin(theta_tibia)*cos(theta_femur)) - 1.0*z*cos(theta_coxa) - 2.0*sin(theta_coxa)*sin(theta_femur)*sin(theta_tibia) + 2.0*sin(theta_coxa)*cos(theta_femur)*cos(theta_tibia) + 1.5*sin(theta_coxa)*cos(theta_femur) + 0.5*sin(theta_coxa),
  x*(1.0*sin(theta_femur)*cos(theta_tibia) + 1.0*sin(theta_tibia)*cos(theta_femur)) + y*(-1.0*sin(theta_femur)*sin(theta_tibia) + 1.0*cos(theta_femur)*cos(theta_tibia)) + 2.0*sin(theta_femur)*cos(theta_tibia) + 1.5*sin(theta_femur) + 2.0*sin(theta_tibia)*cos(theta_femur)};
}


point fk_math(double x, double y, double z, double theta_coxa, double theta_femur, double theta_tibia) {
  return (point){x*(-sin(theta_femur)*sin(theta_tibia)*cos(theta_coxa) + cos(theta_coxa)*cos(theta_femur)*cos(theta_tibia)) + y*(-sin(theta_femur)*cos(theta_coxa)*cos(theta_tibia) - sin(theta_tibia)*cos(theta_coxa)*cos(theta_femur)) + 1.0*z*sin(theta_coxa) - 2.0*sin(theta_femur)*sin(theta_tibia)*cos(theta_coxa) + 2.0*cos(theta_coxa)*cos(theta_femur)*cos(theta_tibia) + 1.5*cos(theta_coxa)*cos(theta_femur) + 0.5*cos(theta_coxa),
  x*(-sin(theta_coxa)*sin(theta_femur)*sin(theta_tibia) + sin(theta_coxa)*cos(theta_femur)*cos(theta_tibia)) + y*(-sin(theta_coxa)*sin(theta_femur)*cos(theta_tibia) - sin(theta_coxa)*sin(theta_tibia)*cos(theta_femur)) - 1.0*z*cos(theta_coxa) - 2.0*sin(theta_coxa)*sin(theta_femur)*sin(theta_tibia) + 2.0*sin(theta_coxa)*cos(theta_femur)*cos(theta_tibia) + 1.5*sin(theta_coxa)*cos(theta_femur) + 0.5*sin(theta_coxa),
  x*(1.0*sin(theta_femur)*cos(theta_tibia) + 1.0*sin(theta_tibia)*cos(theta_femur)) + y*(-1.0*sin(theta_femur)*sin(theta_tibia) + 1.0*cos(theta_femur)*cos(theta_tibia)) + 2.0*sin(theta_femur)*cos(theta_tibia) + 1.5*sin(theta_femur) + 2.0*sin(theta_tibia)*cos(theta_femur)};
}
