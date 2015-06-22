#ifndef DRAWING_PRIMITIVES_H_
#define DRAWING_PRIMITIVES_H_


#include <GL/glut.h>
#include <Eigen/Core>
#include <pthread.h>

#include "pose.h"
#include "utils.h"

class Drawable {
  pthread_mutex_t lock = PTHREAD_MUTEX_INITIALIZER;

  public:
    inline void DrawLock() {
      pthread_mutex_lock(&lock);
    }
    inline void DrawUnlock() {
      pthread_mutex_unlock(&lock);
    }
    virtual void Draw(Eigen::Matrix4d to_global)=0;
    void Draw() {
      this->Draw(Eigen::Matrix4d::Identity());
    }
};


// A basic drawable object to offset the drawing origin.
// Mainly for testing.
class OriginOffset : public Drawable {
  Drawable* child_node;
  Pose pose;
public:
  OriginOffset(Drawable* child, Pose pose) : child_node(child), pose(pose) {};
  void Draw(Eigen::Matrix4d to_global) {
    Eigen::Matrix4d child_to_global = to_global * pose.FromFrameMat();
    child_node->Draw(child_to_global);
  };
};

void GetHeatMapColor(double value, double *red, double *green, double *blue);
void LineStrip(int n_segs, Eigen::Vector4d segs[], double r, double g, double b);
void LineStrip(int n_segs, Eigen::Vector4d segs[], double r[], double g[], double b[]);
void LineStrip(int n_segs, Eigen::Vector3d segs[], double r, double g, double b);

void Point(Eigen::Vector3d point, int size, double r, double g, double b);
void Points(int n_points, Eigen::Vector4d points[], int size, double r, double g, double b);
void Points(int n_points, Eigen::Vector4d points[], int size, double r[], double g[], double b[]);

inline void Point(Eigen::Vector4d point, int size, double r, double g, double b) {
  Point(Vector4dTo3d(point), size, r, g, b);
}

inline void Point(Eigen::Vector4d point, double r, double g, double b) {
  Point(point, 10, r, g, b);
}

inline void Point(Eigen::Vector3d point, double r, double g, double b) {
  Point(point, 10, r, g, b);
}


#endif // DRAWING_PRIMITIVES_H_
