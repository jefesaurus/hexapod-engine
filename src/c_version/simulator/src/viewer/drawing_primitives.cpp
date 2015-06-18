#include "drawing_primitives.h"

void LineStrip(int n_segs, Eigen::Vector4d segs[], double r, double g, double b) {
  glColor3f((float)r, (float)g, (float)b);

  glBegin(GL_LINE_STRIP);
  for (int i = 0; i < n_segs; i++) {
    glVertex3f(segs[i](0), segs[i](1), segs[i](2));
  }
  glEnd();
}

void LineStrip(int n_segs, Eigen::Vector3d segs[], double r, double g, double b) {
  glColor3f((float)r, (float)g, (float)b);

  glBegin(GL_LINE_STRIP);
  for (int i = 0; i < n_segs; i++) {
    glVertex3f(segs[i](0), segs[i](1), segs[i](2));
  }
  glEnd();
}

void Point(Eigen::Vector3d point, double r, double g, double b) {
  glColor3f((float)r, (float)g, (float)b);
  glPointSize(10);
  glBegin(GL_POINTS);
  glVertex3f(point[0], point[1], point[2]);
  glEnd();
}

void Point(Eigen::Vector4d point, double r, double g, double b) {
  glColor3f((float)r, (float)g, (float)b);
  glPointSize(10);
  glBegin(GL_POINTS);
  glVertex3f(point[0], point[1], point[2]);
  glEnd();
}


/*
void LineLoop(int n_segs, Eigen::Vector4d segs[], double r, double g, double b) {
  glColor3f((float)r, (float)g, (float)b);

  glBegin(GL_LINE_LOOP);
  for (int i = 0; i < n_segs; i++) {
    glVertex3f(x[i], y[i], z[i]);
  }
  glEnd();
}
*/
