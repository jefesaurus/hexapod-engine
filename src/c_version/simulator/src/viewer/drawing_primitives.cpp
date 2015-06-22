#include "drawing_primitives.h"

// http://www.andrewnoske.com/wiki/Code_-_heatmaps_and_color_gradients
void GetHeatMapColor(double value, double *red, double *green, double *blue) {
  const int NUM_COLORS = 4;
  /*
  static double color[NUM_COLORS][3] = {{202/255.0,0/255.0,32/255.0},
                                        {244/255.0,165/255.0,130/255.0},
                                        {146/255.0,197/255.0,222/255.0},
                                        {5/255.0,113/255.0,176/255.0}};
  static double color[NUM_COLORS][3] = {{215/255.0, 25/255.0,  28/255.0},
                                        {253/255.0, 174/255.0, 97/255.0},
                                        {171/255.0, 221/255.0, 164/255.0},
                                        {43/255.0,  131/255.0, 186/255.0}};
  */
  static double color[NUM_COLORS][3] = {{215/255.0, 25/255.0,  28/255.0},
                                        {253/255.0, 174/255.0, 97/255.0},
                                        {166/255.0, 217/255.0, 106/255.0},
                                        {26/255.0,  150/255.0, 65/255.0}};
    // A static array of 4 colors:  (blue,   green,  yellow,  red) using {r,g,b} for each.
 
  int idx1;        // |-- Our desired color will be between these two indexes in "color".
  int idx2;        // |
  double fractBetween = 0;  // Fraction between "idx1" and "idx2" where our value is.
 
  if(value <= 0)      {  idx1 = idx2 = 0;            }    // accounts for an input <=0
  else if(value >= 1)  {  idx1 = idx2 = NUM_COLORS-1; }    // accounts for an input >=0
  else
  {
    value = value * (NUM_COLORS-1);        // Will multiply value by 3.
    idx1  = floor(value);                  // Our desired color will be after this index.
    idx2  = idx1+1;                        // ... and before this index (inclusive).
    fractBetween = value - double(idx1);    // Distance between the two indexes (0-1).
  }
 
  *red   = (color[idx2][0] - color[idx1][0])*fractBetween + color[idx1][0];
  *green = (color[idx2][1] - color[idx1][1])*fractBetween + color[idx1][1];
  *blue  = (color[idx2][2] - color[idx1][2])*fractBetween + color[idx1][2];
}

void LineStrip(int n_segs, Eigen::Vector4d segs[], double r, double g, double b) {
  glColor3f((float)r, (float)g, (float)b);

  glBegin(GL_LINE_STRIP);
  for (int i = 0; i < n_segs; i++) {
    glVertex3f(segs[i](0), segs[i](1), segs[i](2));
  }
  glEnd();
}

void LineStrip(int n_segs, Eigen::Vector4d segs[], double r[], double g[], double b[]) {

  glBegin(GL_LINE_STRIP);
  for (int i = 0; i < n_segs; i++) {
    glColor3f((float)r[i], (float)g[i], (float)b[i]);
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

void Point(Eigen::Vector3d point, int size, double r, double g, double b) {
  glColor3f((float)r, (float)g, (float)b);
  glPointSize(size);
  glBegin(GL_POINTS);
  glVertex3f(point[0], point[1], point[2]);
  glEnd();
}

void Points(int n_points, Eigen::Vector4d points[], int size, double r, double g, double b) {
  glColor3f((float)r, (float)g, (float)b);
  glPointSize(size);
  glBegin(GL_POINTS);
    for (int i = 0; i < n_points; i++) {
      glVertex3f(points[i][0], points[i][1], points[i][2]);
    }
  glEnd();
}
void Points(int n_points, Eigen::Vector4d points[], int size, double r[], double g[], double b[]) {
  glPointSize(size);
  glBegin(GL_POINTS);
    for (int i = 0; i < n_points; i++) {
      glColor3f((float)r[i], (float)g[i], (float)b[i]);
      glVertex3f(points[i][0], points[i][1], points[i][2]);
    }
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
