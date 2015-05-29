#ifndef DRAWING_PRIMITIVES_H_
#define DRAWING_PRIMITIVES_H_

#include <GL/glut.h>
#include <Eigen/Core>


void LineStrip(int n_segs, Eigen::Vector4d segs[], double r, double g, double b);


#endif // DRAWING_PRIMITIVES_H_
