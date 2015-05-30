#ifndef KINEMATIC_CHAIN_H
#define KINEMATIC_CHAIN_H

#include <cstdlib>
#include <vector>
#include <Eigen/Core>
#include <Eigen/LU>
#include <cmath>

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
  double min_theta, max_theta;

public:
  RevoluteJoint() {};
  RevoluteJoint(double _min_theta, double _max_theta, double _alpha, double _r, double _d) : KinematicPair(_alpha, _r, _d, (_min_theta + _max_theta)/2.0) {};
  RevoluteJoint(double _alpha, double _r, double _d) : RevoluteJoint(-M_PI, M_PI, _alpha, _r, _d) {};
  void SetTheta(double theta);
};

// 
template <int n_joints> class Leg {
private:
  RevoluteJoint joints[n_joints];

public:
  Leg(RevoluteJoint _joints[n_joints]) {
    for (int i = 0; i < n_joints; i++) {
      joints[i] = _joints[i];
    }
  }

  void SetState(double angles[n_joints]) {
    for (int i = 0; i < n_joints; i++) {
      joints[i].SetTheta(angles[i]);
    }
  }

  Eigen::Vector4d ToGlobal(Eigen::Vector4d in) {
    Eigen::Matrix<double, 4, 4> compound = joints[0].DHMat();
    for (int i = 1; i < n_joints; i++) {
      compound *= joints[i].DHMat();
    }
    return compound * in;
  }

  // If you don't specify a point, it defaults to the origin.
  inline Eigen::Vector4d ToGlobal() {
    return ToGlobal(Eigen::Vector4d(0.0, 0.0, 0.0, 1.0));
  }; 

  // Must have number of segments + 1 points
  void AllSegments(Eigen::Vector4d points[4]) {
    Eigen::Vector4d origin(0.0, 0.0, 0.0, 1.0);
    points[0] = origin;

    Eigen::Matrix<double, 4, 4> compound = joints[0].DHMat();
    for (int i = 1; i < n_joints; i++) {
      points[i] = compound * origin;
      compound *= joints[i].DHMat();
    }
    points[n_joints] = compound * origin;
  }
};

/*
class LegController {

};
*/


/*
struct LegState {
  double theta_coxa, theta_femur, theta_tibia;
  LegState(double _theta_coxa, double _theta_femur, double _theta_tibia);
}

class HexLeg {
public:
  double (*x_to_leg_coxa)(double base_x, double base_y, double theta_coxa);
  double (*y_to_leg_coxa)(double theta_coxa);
  double (*z_to_leg_coxa)(double theta_coxa);
  double (*x_from_leg_coxa)(double theta_coxa);
  double (*y_from_leg_coxa)(double theta_coxa);
  double (*z_from_leg_coxa)(double theta_coxa);

  HexLeg() = delete;
  HexLeg(double (*_x_to_leg_coxa)(double, double, double));
};
*/

#endif // KINEMATIC_CHAIN_H
