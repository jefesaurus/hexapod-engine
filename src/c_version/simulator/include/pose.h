#ifndef POSE_H_
#define POSE_H_

#include <Eigen/Core>
#include <cmath>

#include "drawing_primitives.h"

class Pose : public Drawable {
  // Transformation Matrices
  Eigen::Matrix4d from_frame;
  Eigen::Matrix4d to_frame;

public:
  double x, y, z;
  double yaw, pitch, roll;

  Pose() {};
  Pose(double x, double y, double z, double yaw, double pitch, double roll) : x(x), y(y), z(z), yaw(yaw), pitch(pitch), roll(roll) {
    const double cy = cos(yaw);
    const double sy = sin(yaw);
    const double cp = cos(pitch);
    const double sp = sin(pitch);
    const double cr = cos(roll);
    const double sr = sin(roll);
    from_frame << cy*cp, cy*sp*sr - sy*cr, cy*sp*cr + sy*sr, x, 
                  sy*cp, sy*sp*sr + cy*cr, sy*sp*cr - cy*sr, y,
                  -sp, cp*sr, cp*cr, z,
                  0, 0, 0, 1;
    // Get the inverted rotation submatrix
    to_frame.block<3,3>(0, 0) = from_frame.block<3,3>(0,0).transpose();

    // Get the inverted translation submatrix
    to_frame.block<3, 1>(0, 3) = -to_frame.block<3, 3>(0, 0) * from_frame.block<3, 1>(0, 3);
    to_frame.block<1, 4>(3, 0) << 0, 0, 0, 1;
  };

  inline Eigen::Vector4d FromFrame(Eigen::Vector4d in) const {
    return from_frame * in; 
  }

  inline Pose FromFrame(Pose in) const {
    Eigen::Vector4d position = from_frame * Eigen::Vector4d(in.x, in.y, in.z, 1.0);
    double n_yaw = yaw + in.yaw;
    double n_pitch = pitch + in.pitch;
    double n_roll = roll + in.roll;
    return Pose(position[0], position[1], position[2], n_yaw, n_pitch, n_roll);
  }

  inline Eigen::Vector4d ToFrame(Eigen::Vector4d in) const {
    return to_frame * in;
  }

  inline const Eigen::Matrix4d& ToFrameMat() const {
    return to_frame;
  }

  inline const Eigen::Matrix4d& FromFrameMat() const {
    return from_frame;
  }

  Pose AddDelta(double dx, double dy, double dz, double dyaw, double dpitch, double droll) {
    return Pose(x + dx, y + dy, z + dz, yaw + dyaw, pitch + dpitch, roll + droll);
  }

  void Draw(Eigen::Matrix4d to_global) {
    CoordinateAxes(.1, to_global * from_frame );
  }
};

#endif // POSE_H_
