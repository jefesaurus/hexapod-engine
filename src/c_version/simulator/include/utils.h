#ifndef UTILS_H_
#define UTILS_H_

#include <Eigen/Core>

inline static double RandFloat(double min, double max) {
  double f = (double)rand() / RAND_MAX;
  return min + f * (max - min);
}

// Removes last element from vector.
inline static Eigen::Vector3d Vector4dTo3d(Eigen::Vector4d in) {
  return in.block<3,1>(0,0);
}

// Pads input vector with 1 on the end.
inline static Eigen::Vector4d Vector3dTo4d(Eigen::Vector3d in) {
  return Eigen::Vector4d(in[0], in[1], in[2], 1.0);
}

inline static double WrapRadians(double x) {
  return x-2*M_PI*floor(x/(2*M_PI)+0.5);  
}


#endif // UTILS_H_
