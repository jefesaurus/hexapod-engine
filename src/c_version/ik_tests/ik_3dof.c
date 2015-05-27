#include <stdlib.h>
#include <stdio.h>
#include <math.h>
#include <time.h>   

#include "ik_3dof.h"
#include "./gen/basic_leg_transforms.h"

static const double tibia_r = 1.5;
static const double tibia_d = 0;
static const double femur_r = 2.5;
static const double femur_d = 0;

void IK_3DoF(double x, double y, double z, double* theta_coxa, double* theta_femur, double* theta_tibia) {
  double d = femur_d + tibia_d;
  double det = pow(x, 2) + pow(y, 2) - pow(d, 2);
  if (det <= 0.) {
    *theta_coxa = -1;
    *theta_femur = -1;
    *theta_tibia = -1;
    return;
  }
  double coxa_angle;
  if (d == y) {
    coxa_angle = 2.*atan(y/x); 
  } else {
    coxa_angle = 2.*atan(y/x); 
    coxa_angle = 2.*atan((x - sqrt(det))/(d - y));
  }

  double nx = x_to_leg_coxa(x, y, coxa_angle);
  double ny = y_to_leg_coxa(z);

  double target = pow(nx,2) + pow(ny,2);
  double range = pow(femur_r + tibia_r, 2);
  double target_dir = atan2(ny, nx);

  if (target > range) { // Too far
    *theta_coxa = -1;
    *theta_femur = -1;
    *theta_tibia = -1;
    return;
  } else if (target < pow((femur_r - tibia_r), 2)) {// Too close
    *theta_coxa = -1;
    *theta_femur = -1;
    *theta_tibia = -1;
    return;
  } 

  double theta_a = acos((pow(femur_r,2) + pow(tibia_r,2) - target)/(2*femur_r * tibia_r));
  double theta_b = acos((pow(femur_r,2) + target - pow(tibia_r,2))/(2*femur_r * sqrt(target)));

  *theta_coxa = coxa_angle;
  *theta_femur = target_dir + theta_b;
  *theta_tibia = theta_a - M_PI;
}


void trial(int count) {
  printf("Trial with %d iterations:\n", count);
  clock_t t1, t2;
  double domain = 1.5;

  double x, y, z;
  double coxa, tibia, femur;
  double scoxa = 0;
  double stibia = 0;
  double sfemur = 0;

  srand(time(NULL));
  t1 = clock();  
  for (int i = 0; i < count; i ++) {
    x = ((double)rand()/(double)RAND_MAX)*domain + 2;
    y = ((double)rand()/(double)RAND_MAX)*domain - domain/2;
    z = ((double)rand()/(double)RAND_MAX)*domain - 1;
    IK_3DoF(x, y, z, &coxa, &femur, &tibia);
    scoxa += coxa;
    sfemur += femur;
    stibia += tibia;
  }
  t2 = clock();   

  float diff = (((float)t2 - (float)t1) / 1000000.0F ) * 1000;   
  printf("Milliseconds: %f\n",diff);  
  printf("Results: %f, %f, %f\n",scoxa/count, sfemur/count, stibia/count);  
}

void test() {
  double x = 3;
  double y = 0;
  double z = 0;
  double coxa, tibia, femur;
  IK_3DoF(x, y, z, &coxa, &femur, &tibia);
  printf("Results: %f, %f, %f\n",coxa, femur, tibia);  
}

int main(int argc, char** argv) {
  if (argc < 2) {
    printf("argument 1 must be the number of iterations.\n");
  } else {
    int count = atoi(argv[1]);
    trial(count);
  }
  return 0;
}
