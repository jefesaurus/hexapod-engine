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


class ColoredGrid : Drawable {
  const int width;
  const int height;

  double* x;
  double* y;
  double* z;

  double* r;
  double* g;
  double* b;

  inline int GetIndex(int row, int col) const {
    return row*width + col;
  }

  public:
  ColoredGrid(int width, int height) : width(width), height(height),
                                       x(new double[width*height]),
                                       y(new double[width*height]),
                                       z(new double[width*height]),
                                       r(new double[width*height]),
                                       g(new double[width*height]),
                                       b(new double[width*height]) {
  }

  ColoredGrid(const ColoredGrid& grid): width(grid.width), height(grid.height) {
    x = new double[width*height];
    y = new double[width*height];
    z = new double[width*height];
    r = new double[width*height];
    g = new double[width*height];
    b = new double[width*height];
    for (int row = 0; row < height; row++) {
      for (int col = 0; col < width; col++) {
        x[GetIndex(row, col)] = grid.x[GetIndex(row, col)];
        y[GetIndex(row, col)] = grid.y[GetIndex(row, col)];
        z[GetIndex(row, col)] = grid.z[GetIndex(row, col)];
        r[GetIndex(row, col)] = grid.r[GetIndex(row, col)];
        g[GetIndex(row, col)] = grid.g[GetIndex(row, col)];
        b[GetIndex(row, col)] = grid.b[GetIndex(row, col)];
      }
    }
  }

  ~ColoredGrid() {
    delete[] x;
    delete[] y;
    delete[] z;
    delete[] r;
    delete[] g;
    delete[] b;
  }

  inline void SetVal(int row, int col, double _x, double _y, double _z, double _r, double _g, double _b) {
    int index = GetIndex(row, col);
    x[index] = _x;
    y[index] = _y;
    z[index] = _z;
    r[index] = _r;
    g[index] = _g;
    b[index] = _b;
  }

  void Draw(Eigen::Matrix4d to_global) {
    Eigen::Vector4d final_point;
    float normal[3] = {0.0, 0.0, 1.0};
    int top, bot;
    for (int row = 0; row < height - 1; row++) {
      glBegin(GL_TRIANGLE_STRIP);
      for (int col = 0; col < width; col++) {
        bot = GetIndex(row+1, col);
        glNormal3fv(&normal[0]);
        glColor3f((float)r[bot], (float)g[bot], (float)b[bot]);
        final_point = to_global*Eigen::Vector4d(x[bot], y[bot], z[bot], 1.0);
        glVertex3f(final_point[0], final_point[1], final_point[2]);

        top = GetIndex(row, col);
        glNormal3fv(&normal[0]);
        glColor3f((float)r[top], (float)g[top], (float)b[top]);
        final_point = to_global*Eigen::Vector4d(x[top], y[top], z[top], 1.0);
        glVertex3f(final_point[0], final_point[1], final_point[2]);
      }
      glEnd();
    }
  }
};

#endif // DRAWING_PRIMITIVES_H_
