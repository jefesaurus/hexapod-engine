#include <stdio.h>
#include <stdlib.h>
#include <time.h>   
#include <math.h>


typedef struct point point;
struct point {
  double x;
  double y;
  double z;
};

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

point fk_math_cache_fixed(double x, double y, double z, double theta_coxa, double theta_femur, double theta_tibia) {
  double sin_coxa = sin(theta_coxa);
  double cos_coxa = cos(theta_coxa);
  double sin_femur = sin(theta_femur);
  double cos_femur = cos(theta_femur);
  double sin_tibia = sin(theta_tibia);
  double cos_tibia = cos(theta_tibia);
  return (point){-2.0*sin_femur*sin_tibia*cos_coxa + 2.0*cos_coxa*cos_femur*cos_tibia + 1.5*cos_coxa*cos_femur + 0.5*cos_coxa,
  - 2.0*sin_coxa*sin_femur*sin_tibia + 2.0*sin_coxa*cos_femur*cos_tibia + 1.5*sin_coxa*cos_femur + 0.5*sin_coxa,
  2.0*sin_femur*cos_tibia + 1.5*sin_femur + 2.0*sin_tibia*cos_femur};
}

point fk_math(double x, double y, double z, double theta_coxa, double theta_femur, double theta_tibia) {
  return (point){x*(-sin(theta_femur)*sin(theta_tibia)*cos(theta_coxa) + cos(theta_coxa)*cos(theta_femur)*cos(theta_tibia)) + y*(-sin(theta_femur)*cos(theta_coxa)*cos(theta_tibia) - sin(theta_tibia)*cos(theta_coxa)*cos(theta_femur)) + 1.0*z*sin(theta_coxa) - 2.0*sin(theta_femur)*sin(theta_tibia)*cos(theta_coxa) + 2.0*cos(theta_coxa)*cos(theta_femur)*cos(theta_tibia) + 1.5*cos(theta_coxa)*cos(theta_femur) + 0.5*cos(theta_coxa),
  x*(-sin(theta_coxa)*sin(theta_femur)*sin(theta_tibia) + sin(theta_coxa)*cos(theta_femur)*cos(theta_tibia)) + y*(-sin(theta_coxa)*sin(theta_femur)*cos(theta_tibia) - sin(theta_coxa)*sin(theta_tibia)*cos(theta_femur)) - 1.0*z*cos(theta_coxa) - 2.0*sin(theta_coxa)*sin(theta_femur)*sin(theta_tibia) + 2.0*sin(theta_coxa)*cos(theta_femur)*cos(theta_tibia) + 1.5*sin(theta_coxa)*cos(theta_femur) + 0.5*sin(theta_coxa),
  x*(1.0*sin(theta_femur)*cos(theta_tibia) + 1.0*sin(theta_tibia)*cos(theta_femur)) + y*(-1.0*sin(theta_femur)*sin(theta_tibia) + 1.0*cos(theta_femur)*cos(theta_tibia)) + 2.0*sin(theta_femur)*cos(theta_tibia) + 1.5*sin(theta_femur) + 2.0*sin(theta_tibia)*cos(theta_femur)};
}



double trialA() {
  clock_t t1, t2;

  point punto;
  double x = 0;
  double y = 0;
  double z = 0;

  t1 = clock();  

  int i;
  for (i = 0; i < 1400000000; i ++) {
    punto = fk_math_cache_fixed(0., 0., 0., -2.09439510239, 1.0471975512, 0.78539816339);
    x += punto.x;
    y += punto.y;
    z += punto.z;
  }
  t2 = clock();   

  float diff = (((float)t2 - (float)t1) / 1000000.0F ) * 1000;   
  printf("Milliseconds: %f\n",diff);  
  return (x+y+z);
}

double trialB() {
  clock_t t1, t2;

  point punto;
  double x = 0;
  double y = 0;
  double z = 0;



  t1 = clock();  

  int i;
  for (i = 0; i < 1400000000; i ++) {
    punto = fk_math(0., 0., 0., -2.09439510239, 1.0471975512, 0.78539816339);
    x += punto.x;
    y += punto.y;
    z += punto.z;
  }
  t2 = clock();   

  float diff = (((float)t2 - (float)t1) / 1000000.0F ) * 1000;   
  printf("Milliseconds: %f\n",diff);  
  return (x+y+z);
}

/*
Compiled with:
gcc demo.c -lm -Wall -O3

Experimental Output:
Milliseconds: 1211.336914
Result: 3122650734.759146
Milliseconds: 1209.632935
Result: 3122650734.759146

Conclusion: compiler is 2smart
*/
int main( int argc, const char* argv[] ) {
  double y = trialB();
  printf("Result: %f\n", y);
  double x = trialA();
  printf("Result: %f\n", x);
}
