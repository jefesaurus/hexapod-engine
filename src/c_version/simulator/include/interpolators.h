#ifndef INTERPOLATORS_H_
#define INTERPOLATORS_H_

#include <Eigen/Core>
#include <cmath>

template <class T>
class PathGen {
public:
  virtual T Value(double progress)=0;
};

template <class T>
class LinearInterpolator : public PathGen<T> {
protected:
  T start, end;
public:
  LinearInterpolator() {};
  LinearInterpolator(T start, T end) : start(start), end(end) {};
  T Value(double progress) {
    return (1.0 - progress) * start + progress*end;
  }

  void SetPoints(T _start, T _end) {
    start = _start;
    end = _end;
  }
};

template <class T>
T LinearInterpolate(double progress, T start, T end) {
  return (1.0 - progress) * start + progress * end;
}

template <class T>
class CubicBezierInterpolator : public PathGen<T> {
protected:
  T p0, p1, p2, p3;
public:
  CubicBezierInterpolator() {};
  CubicBezierInterpolator(T p0, T p1, T p2, T p3) : p0(p0), p1(p1), p2(p2), p3(p3) {};
  T Value(double progress) {
    return pow((1 - progress), 3)*p0 + pow((1 - progress), 2)*progress*p1 + (1 - progress)*pow(progress, 2)*p2 + pow(progress, 3)*p3;
  }

  void SetPoints(T _p0, T _p1, T _p2, T _p3) {
    p0 = _p0;
    p1 = _p1;
    p2 = _p2;
    p3 = _p3;
  }
};

template <class T>
T CubicBezierInterpolate(double progress, T p0, T p1, T p2, T p3) {
  return pow((1 - progress), 3)*p0 + pow((1 - progress), 2)*progress*p1 + (1 - progress)*pow(progress, 2)*p2 + pow(progress, 3)*p3;
}

class StepInterpolator : public CubicBezierInterpolator<Eigen::Vector3d> {
public:
  StepInterpolator(Eigen::Vector3d start, Eigen::Vector3d end, double step_height) {
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

typedef LinearInterpolator<Eigen::Vector3d> DragInterpolator;

#endif // INTERPOLATORS_H_
