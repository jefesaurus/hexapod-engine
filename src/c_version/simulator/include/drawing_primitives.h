#ifndef DRAWING_PRIMITIVES_H_
#define DRAWING_PRIMITIVES_H_


#include <GL/glut.h>
#include <Eigen/Core>
#include <pthread.h>
#include <vector>

#include "utils.h"

// TODO extend to animatable which has a better thread safety interface.
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

class Scene : public Drawable {
  std::vector<Drawable*> objects;

public:
  Scene(){};
  void AddDrawable(Drawable* to_add) {
    objects.push_back(to_add);
  }

  void Draw(Eigen::Matrix4d to_global) {
    for (uint i = 0; i < objects.size(); i++) {
      objects[i]->Draw(to_global);
    }
  }
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


void CoordinateAxes(double size, Eigen::Matrix4d to_global);

#endif // DRAWING_PRIMITIVES_H_
