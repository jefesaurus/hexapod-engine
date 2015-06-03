#ifndef KINEMATIC_PAIR_H
#define KINEMATIC_PAIR_H

#include <cstdlib>
#include <vector>
#include <Eigen/Core>
#include <Eigen/LU>
#include <cmath>
#include <cassert>
#include <cfloat>

#include "drawing_primitives.h"

class KinematicPair {
protected:
  double alpha, r, d, theta;
  bool params_changed;
  Eigen::Matrix4d dh_mat;
  Eigen::Matrix4d inv_dh_mat;
  void GenerateDHMatrices();

public:
  KinematicPair() {};
  KinematicPair(double _alpha, double _r, double _d, double _theta) : alpha(_alpha), r(_r), d(_d), theta(_theta), params_changed(true) {};

  // Accessors
  inline double Alpha() const { return alpha; };
  inline double R() const { return r; };
  inline double D() const { return d; };
  inline double Theta() const { return theta; };
  inline bool ParamsChanged() const { return params_changed; };

  // Recalculates the DH matrices if necessary as they are queried.
  inline const Eigen::Matrix4d& DHMat() {
    if (params_changed) {
      GenerateDHMatrices();
      params_changed = false;
    }
    return dh_mat;
  };
  inline const Eigen::Matrix<double, 4, 4>& InvDHMat() {
    if (params_changed) {
      GenerateDHMatrices();
      params_changed = false;
    }
    return inv_dh_mat;
  };
};

class RevoluteJoint : public KinematicPair {
  // Extremes
  double min_theta, max_theta;
  double max_angular_velocity;

  // Current command state
  bool is_active;
  double destination, velocity;

public:
  RevoluteJoint() {};
  RevoluteJoint(double _min_theta, double _max_theta, double _max_vel, double _alpha, double _r, double _d) : KinematicPair(_alpha, _r, _d, (_min_theta + _max_theta)/2.0), max_angular_velocity(_max_vel) {};
  RevoluteJoint(double _alpha, double _r, double _d) : RevoluteJoint(-M_PI, M_PI, DBL_MAX, _alpha, _r, _d) {};

  void SetTheta(double theta);
  void SetCommand(double destination, double velocity);
  void UpdateState(double time_elapsed);
};

#endif // KINEMATIC_PAIR_H
