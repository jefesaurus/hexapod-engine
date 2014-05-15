/******************************************************************************
 *                      Code generated with sympy 0.7.5                       *
 *                                                                            *
 *              See http://www.sympy.org/ for more information.               *
 *                                                                            *
 *                       This file is part of 'leg_fk'                        *
 ******************************************************************************/
#include "basic_leg_transforms.h"
#include <math.h>

double x_from_leg_coxa(double theta_coxa) {

   return 0.5*cos(theta_coxa);

}

double y_from_leg_coxa(double theta_coxa) {

   return 0.5*sin(theta_coxa);

}

double z_from_leg_coxa() {

   return 0.0;

}

double x_to_leg_coxa(double base_x, double base_y, double theta_coxa) {

   return base_x*cos(theta_coxa) + base_y*sin(theta_coxa) - 0.5*pow(sin(theta_coxa), 2) - 0.5*pow(cos(theta_coxa), 2);

}

double y_to_leg_coxa(double base_z) {

   return 1.0*base_z;

}

double z_to_leg_coxa(double base_x, double base_y, double theta_coxa) {

   return 1.0*base_x*sin(theta_coxa) - 1.0*base_y*cos(theta_coxa);

}

double x_from_leg_femur(double theta_coxa, double theta_femur) {

   return 1.5*cos(theta_coxa)*cos(theta_femur) + 0.5*cos(theta_coxa);

}

double y_from_leg_femur(double theta_coxa, double theta_femur) {

   return 1.5*sin(theta_coxa)*cos(theta_femur) + 0.5*sin(theta_coxa);

}

double z_from_leg_femur(double theta_femur) {

   return 1.5*sin(theta_femur);

}

double x_to_leg_femur(double base_x, double base_y, double base_z, double theta_coxa, double theta_femur) {

   return base_x*cos(theta_coxa)*cos(theta_femur) + base_y*sin(theta_coxa)*cos(theta_femur) + 1.0*base_z*sin(theta_femur) + (-0.5*pow(sin(theta_coxa), 2) - 0.5*pow(cos(theta_coxa), 2))*cos(theta_femur) - 1.5*pow(sin(theta_femur), 2) - 1.5*pow(cos(theta_femur), 2);

}

double y_to_leg_femur(double base_x, double base_y, double base_z, double theta_coxa, double theta_femur) {

   return -base_x*sin(theta_femur)*cos(theta_coxa) - base_y*sin(theta_coxa)*sin(theta_femur) + 1.0*base_z*cos(theta_femur) - (-0.5*pow(sin(theta_coxa), 2) - 0.5*pow(cos(theta_coxa), 2))*sin(theta_femur);

}

double z_to_leg_femur(double base_x, double base_y, double theta_coxa) {

   return 1.0*base_x*sin(theta_coxa) - 1.0*base_y*cos(theta_coxa);

}

double x_from_leg_tibia(double theta_coxa, double theta_femur, double theta_tibia) {

   return -2.0*sin(theta_femur)*sin(theta_tibia)*cos(theta_coxa) + 2.0*cos(theta_coxa)*cos(theta_femur)*cos(theta_tibia) + 1.5*cos(theta_coxa)*cos(theta_femur) + 0.5*cos(theta_coxa);

}

double y_from_leg_tibia(double theta_coxa, double theta_femur, double theta_tibia) {

   return -2.0*sin(theta_coxa)*sin(theta_femur)*sin(theta_tibia) + 2.0*sin(theta_coxa)*cos(theta_femur)*cos(theta_tibia) + 1.5*sin(theta_coxa)*cos(theta_femur) + 0.5*sin(theta_coxa);

}

double z_from_leg_tibia(double theta_femur, double theta_tibia) {

   return 2.0*sin(theta_femur)*cos(theta_tibia) + 1.5*sin(theta_femur) + 2.0*sin(theta_tibia)*cos(theta_femur);

}

double x_to_leg_tibia(double base_x, double base_y, double base_z, double theta_coxa, double theta_femur, double theta_tibia) {

   return base_x*(-sin(theta_femur)*sin(theta_tibia) + cos(theta_femur)*cos(theta_tibia))*cos(theta_coxa) + base_y*(-sin(theta_femur)*sin(theta_tibia) + cos(theta_femur)*cos(theta_tibia))*sin(theta_coxa) + base_z*(1.0*sin(theta_femur)*cos(theta_tibia) + 1.0*sin(theta_tibia)*cos(theta_femur)) + (-sin(theta_femur)*sin(theta_tibia) + cos(theta_femur)*cos(theta_tibia))*(-0.5*pow(sin(theta_coxa), 2) - 0.5*pow(cos(theta_coxa), 2)) + (-1.5*pow(sin(theta_femur), 2) - 1.5*pow(cos(theta_femur), 2))*cos(theta_tibia) - 2.0*pow(sin(theta_tibia), 2) - 2.0*pow(cos(theta_tibia), 2);

}

double y_to_leg_tibia(double base_x, double base_y, double base_z, double theta_coxa, double theta_femur, double theta_tibia) {

   return base_x*(-sin(theta_femur)*cos(theta_tibia) - sin(theta_tibia)*cos(theta_femur))*cos(theta_coxa) + base_y*(-sin(theta_femur)*cos(theta_tibia) - sin(theta_tibia)*cos(theta_femur))*sin(theta_coxa) + base_z*(-1.0*sin(theta_femur)*sin(theta_tibia) + 1.0*cos(theta_femur)*cos(theta_tibia)) + (-sin(theta_femur)*cos(theta_tibia) - sin(theta_tibia)*cos(theta_femur))*(-0.5*pow(sin(theta_coxa), 2) - 0.5*pow(cos(theta_coxa), 2)) - (-1.5*pow(sin(theta_femur), 2) - 1.5*pow(cos(theta_femur), 2))*sin(theta_tibia);

}

double z_to_leg_tibia(double base_x, double base_y, double theta_coxa) {

   return 1.0*base_x*sin(theta_coxa) - 1.0*base_y*cos(theta_coxa);

}
