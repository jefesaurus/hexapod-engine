#ifndef DRAWING_PRIMITIVES_H_
#define DRAWING_PRIMITIVES_H_

#include <GL/glut.h>
#include <Eigen/Core>

#include <pthread.h>

class Drawable {
  pthread_mutex_t lock = PTHREAD_MUTEX_INITIALIZER;

  public:
    inline void DrawLock() {
      pthread_mutex_lock(&lock);
    }
    inline void DrawUnlock() {
      pthread_mutex_unlock(&lock);
    }
    virtual void Draw()=0;
};

void LineStrip(int n_segs, Eigen::Vector4d segs[], double r, double g, double b);
void LineStrip(int n_segs, Eigen::Vector3d segs[], double r, double g, double b);
void Point(Eigen::Vector3d point, double r, double g, double b);


#endif // DRAWING_PRIMITIVES_H_
