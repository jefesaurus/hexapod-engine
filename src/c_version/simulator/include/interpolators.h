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

class PoseGen : public Interpolator<Pose>, public Drawable {
public:
  virtual Pose Value(double progress)=0;
  void Draw(Eigen::Matrix4d to_global) {
    static int n_points = 50;
    for (int i = 0; i <= n_points; i++) {
      Pose inter = Value(((float)i/n_points));
      inter.Draw(to_global);
    }
  }
};

template <class T> 
inline static T LinearInterpolate(T start, T end, double progress) {
  return (1.0 - progress) * start + progress * end;
};

inline static double LinearInterpolateRadians(double start, double end, double progress) {
  return start + progress*WrapRadians(end - start);
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
    // TODO replace with hermite, or write out power explicitly.
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

template <class T> 
inline static T CubicHermiteInterpolate(T p1, T p2, T m1, T m2, double p) {
  return (2*p*p*p - 3*p*p + 1)*p1 + (p*p*p - 2*p*p + p)*m1 + (-2*p*p*p + 3*p*p)*p2 + (p*p*p - p*p)*m2;
};

// Generate the cubic hermite spline path between two poses.
class PoseSpline : public PoseGen {
Pose p1, p2;
// The slopes at the respective poses.
Eigen::Vector3d m1, m2;
public:
  PoseSpline(Pose p1, Pose p2, Eigen::Vector3d m1, Eigen::Vector3d m2) : p1(p1), p2(p2), m1(m1), m2(m2) {}

  Pose Value(double progress) {
    // Linear interpolation of the orientation
    double yaw = LinearInterpolateRadians(p1.yaw, p2.yaw, progress);
    double pitch = LinearInterpolateRadians(p1.pitch, p2.pitch, progress);
    double roll = LinearInterpolateRadians(p1.roll, p2.roll, progress);

    // Hermite spline of position
    Eigen::Vector3d p1_pos(p1.x, p1.y, p1.z);
    Eigen::Vector3d p2_pos(p2.x, p2.y, p2.z);

    Eigen::Vector3d position = CubicHermiteInterpolate<Eigen::Vector3d>(p1_pos, p2_pos, m1, m2, progress);
    return Pose(position[0], position[1], position[2], yaw, pitch, roll);
  }
};


// Same as regular PoseSpline but adds a rigid offset pose.
class OffsetPoseSpline : public PoseGen {
  PoseSpline base_spline;
  Pose base_offset;

public:
  OffsetPoseSpline(PoseSpline base_spline, Pose base_offset) : base_spline(base_spline), base_offset(base_offset){}
  Pose Value(double progress) {
    Pose base_pose = base_spline.Value(progress);
    return base_pose.FromFrame(base_offset);
  }
};

#endif // INTERPOLATORS_H_
