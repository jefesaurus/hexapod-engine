#ifndef INTERPOLATORS_H_
#define INTERPOLATORS_H_

#include <Eigen/Core>
#include <cmath>

#include "pose.h"

template <class T>
class Interpolator {
public:
  virtual T Value(double progress)=0;
};

class PathGen : public Interpolator<Eigen::Vector3d> {
public:
  virtual Eigen::Vector3d Value(double progress)=0;
};

class PoseGen : public Interpolator<Pose> {
public:
  virtual Pose Value(double progress)=0;
};

template <class T> 
inline static T LinearInterpolate(T start, T end, double progress) {
  return (1.0 - progress) * start + progress * end;
};

class LinearPath : public PathGen {
protected:
  Eigen::Vector3d start, end;
public:
  LinearPath() {};
  LinearPath(Eigen::Vector3d start, Eigen::Vector3d end) : start(start), end(end) {};
  Eigen::Vector3d Value(double progress) {
    return LinearInterpolate<Eigen::Vector3d>(start, end, progress);
  }

  void SetPoints(Eigen::Vector3d _start, Eigen::Vector3d _end) {
    start = _start;
    end = _end;
  }
};

class CubicBezierPath : public PathGen {
protected:
  Eigen::Vector3d p0, p1, p2, p3;
public:
  CubicBezierPath() {};
  CubicBezierPath(Eigen::Vector3d p0, Eigen::Vector3d p1, Eigen::Vector3d p2, Eigen::Vector3d p3) : p0(p0), p1(p1), p2(p2), p3(p3) {};
  Eigen::Vector3d Value(double progress) {
    return pow((1 - progress), 3)*p0 + 3*pow((1 - progress), 2)*progress*p1 + 3*(1 - progress)*pow(progress, 2)*p2 + pow(progress, 3)*p3;
  }

  void SetPoints(Eigen::Vector3d _p0, Eigen::Vector3d _p1, Eigen::Vector3d _p2, Eigen::Vector3d _p3) {
    p0 = _p0;
    p1 = _p1;
    p2 = _p2;
    p3 = _p3;
  }
};

class StepPath : public CubicBezierPath {
public:
  StepPath(Eigen::Vector3d start, Eigen::Vector3d end, double step_height) {
    this->p0 = start;
    this->p1 = Eigen::Vector3d(start[0], start[1], start[2] + step_height);
    this->p2 = Eigen::Vector3d(end[0], end[1], end[2] + step_height);
    this->p3 = end;
  };

  void SetPoints(Eigen::Vector3d start, Eigen::Vector3d end, double step_height) {
    this->p0 = start;
    this->p1 = Eigen::Vector3d(start[0], start[1], start[2] + step_height);
    this->p2 = Eigen::Vector3d(end[0], end[1], end[2] + step_height);
    this->p3 = end;
  }
};

/*

// Generate the cubic hermite spline path between two poses.
class PoseSpline : public PoseGen {
Pose p1, p2;
public:
  PoseSpline(Pose p1, Pose p2) : p1(p1), p2(p2) {}

  Pose Value(double progress) {
    // Linear interpolation of the orientation
    double yaw, pitch, roll;
    yaw = LinearInterpolate
    // Hermite spline of position
    double x, y, z;
    return Pose(x, y, z, yaw, pitch, roll);
  }

}
*/

#endif // INTERPOLATORS_H_
