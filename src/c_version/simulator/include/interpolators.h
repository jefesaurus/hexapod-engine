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

// Represents a 3-dimensional position path.
class PathGen : public Interpolator<Eigen::Vector3d>, public Drawable {
public:
  virtual Eigen::Vector3d Value(double progress)=0;
  virtual void Transform(Eigen::Matrix4d transform)=0;

  void Draw(Eigen::Matrix4d to_global) {
    static int n_points = 50;
    Eigen::Vector4d points[n_points];
    for (int i = 0; i <= n_points; i++) {
      Eigen::Vector3d inter = Value(((float)i/(n_points - 1)));
      points[i] = to_global * Vector3dTo4d(inter);
    }
    LineStrip(n_points, points, 1.0, 0.0, 0.0);
  }
};

// Represents a 6-dimensional position and orientation path.
class PoseGen : public Interpolator<Pose>, public Drawable {
public:
  virtual Pose Value(double progress)=0;
  void Draw(Eigen::Matrix4d to_global) {
    static int n_points = 100;
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

  void Transform(Eigen::Matrix4d transform) {
    start = Vector4dTo3d(transform * Vector3dTo4d(start));
    end = Vector4dTo3d(transform * Vector3dTo4d(end));
  }
};

template <class T> 
inline static T ConicBezierInterpolate(T p1, T p2, T p3, double p) {
  return (1-p)*(1-p)*p1 + 2*(1-p)*p*p2 + p*p*p3;
};

class ConicBezierPath : public PathGen {
protected:
  Eigen::Vector3d p0, p1, p2;
public:
  ConicBezierPath() {};
  ConicBezierPath(Eigen::Vector3d p0, Eigen::Vector3d p1, Eigen::Vector3d p2) : p0(p0), p1(p1), p2(p2) {};
  Eigen::Vector3d Value(double progress) {
    return ConicBezierInterpolate<Eigen::Vector3d>(p0, p1, p2, progress);
  }

  void Transform(Eigen::Matrix4d transform) {
    p0 = Vector4dTo3d(transform * Vector3dTo4d(p0));
    p1 = Vector4dTo3d(transform * Vector3dTo4d(p1));
    p2 = Vector4dTo3d(transform * Vector3dTo4d(p2));
  }
};

template <class T> 
inline static T CubicHermiteInterpolate(T p1, T p2, T m1, T m2, double p) {
  return (2*p*p*p - 3*p*p + 1)*p1 + (p*p*p - 2*p*p + p)*m1 + (-2*p*p*p + 3*p*p)*p2 + (p*p*p - p*p)*m2;
};

template <class T> 
inline static T CubicBezierInterpolate(T p1, T p2, T p3, T p4, double p) {
  return (1-p)*(1-p)*(1-p)*p1 + 3*(1-p)*(1-p)*p*p2 + 3*(1-p)*p*p*p3 + p*p*p*p4;
};

class CubicBezierPath : public PathGen {
protected:
  Eigen::Vector3d p0, p1, p2, p3;
public:
  CubicBezierPath() {};
  CubicBezierPath(Eigen::Vector3d p0, Eigen::Vector3d p1, Eigen::Vector3d p2, Eigen::Vector3d p3) : p0(p0), p1(p1), p2(p2), p3(p3) {};
  Eigen::Vector3d Value(double progress) {
    return CubicBezierInterpolate<Eigen::Vector3d>(p0, p1, p2, p3, progress);
  }

  void SetPoints(Eigen::Vector3d _p0, Eigen::Vector3d _p1, Eigen::Vector3d _p2, Eigen::Vector3d _p3) {
    p0 = _p0;
    p1 = _p1;
    p2 = _p2;
    p3 = _p3;
  }

  void Transform(Eigen::Matrix4d transform) {
    p0 = Vector4dTo3d(transform * Vector3dTo4d(p0));
    p1 = Vector4dTo3d(transform * Vector3dTo4d(p1));
    p2 = Vector4dTo3d(transform * Vector3dTo4d(p2));
    p3 = Vector4dTo3d(transform * Vector3dTo4d(p3));
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

class CubicHermitePath : public PathGen {
protected:
  // Endpoints.
  Eigen::Vector3d p0, p1;
  // Derivatives at end points.
  Eigen::Vector3d m0, m1;
public:
  CubicHermitePath() {};
  CubicHermitePath(Eigen::Vector3d p0, Eigen::Vector3d p1, Eigen::Vector3d m0, Eigen::Vector3d m1) : p0(p0), p1(p1), m0(m0), m1(m1) {};
  Eigen::Vector3d Value(double progress) {
    return CubicHermiteInterpolate<Eigen::Vector3d>(p0, p1, m0, m1, progress);
  }
  void SetPoints(Eigen::Vector3d _p0, Eigen::Vector3d _p1, Eigen::Vector3d _m0, Eigen::Vector3d _m1) {
    p0 = _p0;
    p1 = _p1;
    m0 = _m0;
    m1 = _m1;
  }

  void Transform(Eigen::Matrix4d transform) {
    p0 = Vector4dTo3d(transform * Vector3dTo4d(p0));
    p1 = Vector4dTo3d(transform * Vector3dTo4d(p1));
    m0 = Vector4dTo3d(transform * Vector3dTo4d(m0));
    m1 = Vector4dTo3d(transform * Vector3dTo4d(m1));
  }
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
class OffsetPoseGen : public PoseGen {
  PoseGen* base_gen;
  Pose base_offset;

public:
  OffsetPoseGen() {};
  OffsetPoseGen(PoseGen* base_gen, Pose base_offset) : base_gen(base_gen), base_offset(base_offset){}
  Pose Value(double progress) {
    Pose base_pose = base_gen->Value(progress);
    return base_pose.FromFrame(base_offset);
  }
};

// Generates a path representing the input pathgen as viewed by the input posegen.
class PathFromPose : public PathGen {
  // The pose-path of the results local frame
  std::unique_ptr<PoseGen> local;
  // The input path.
  std::unique_ptr<PathGen> external;

public:
  PathFromPose() {};
  PathFromPose(std::unique_ptr<PoseGen> local, std::unique_ptr<PathGen> external) : local(std::move(local)), external(std::move(external)){}
  Eigen::Vector3d Value(double progress) {
    Pose base_pose = local->Value(progress);
    return Vector4dTo3d(base_pose.ToFrame(Vector3dTo4d(external->Value(progress))));
  }
};

#endif // INTERPOLATORS_H_
