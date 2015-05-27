/******************************************************************************
 *                      Code generated with sympy 0.7.5                       *
 *                                                                            *
 *              See http://www.sympy.org/ for more information.               *
 *                                                                            *
 *                       This file is part of 'leg_fk'                        *
 ******************************************************************************/


#ifndef LEG_FK__.._GEN_BASIC_LEG_TRANSFORMS__H
#define LEG_FK__.._GEN_BASIC_LEG_TRANSFORMS__H

double x_from_leg_coxa(double theta_coxa);
double y_from_leg_coxa(double theta_coxa);
double z_from_leg_coxa();
double x_to_leg_coxa(double base_x, double base_y, double theta_coxa);
double y_to_leg_coxa(double base_z);
double z_to_leg_coxa(double base_x, double base_y, double theta_coxa);
double x_from_leg_femur(double theta_coxa, double theta_femur);
double y_from_leg_femur(double theta_coxa, double theta_femur);
double z_from_leg_femur(double theta_femur);
double x_to_leg_femur(double base_x, double base_y, double base_z, double theta_coxa, double theta_femur);
double y_to_leg_femur(double base_x, double base_y, double base_z, double theta_coxa, double theta_femur);
double z_to_leg_femur(double base_x, double base_y, double theta_coxa);
double x_from_leg_tibia(double theta_coxa, double theta_femur, double theta_tibia);
double y_from_leg_tibia(double theta_coxa, double theta_femur, double theta_tibia);
double z_from_leg_tibia(double theta_femur, double theta_tibia);
double x_to_leg_tibia(double base_x, double base_y, double base_z, double theta_coxa, double theta_femur, double theta_tibia);
double y_to_leg_tibia(double base_x, double base_y, double base_z, double theta_coxa, double theta_femur, double theta_tibia);
double z_to_leg_tibia(double base_x, double base_y, double theta_coxa);

#endif

