// Generic 3D environment for drawing basic geometry.

// in order to get function prototypes from glext.h, define GL_GLEXT_PROTOTYPES before including glext.h
#define GL_GLEXT_PROTOTYPES

#include <GL/glut.h>
#include "glext.h"
#include <iostream>
#include <sstream>
#include <iomanip>
#include <cstdlib>
#include <cstring>
#include <stdio.h>
#include "Timer.h"


// GLUT CALLBACK functions ////////////////////////////////////////////////////
void displayCB();
void reshapeCB(int w, int h);
void timerCB(int millisec);
void idleCB();
void keyboardCB(unsigned char key, int x, int y);
void mouseCB(int button, int stat, int x, int y);
void mouseMotionCB(int x, int y);

// CALLBACK function when exit() called ///////////////////////////////////////
void exitCB();

void initGL();
int  initGLUT(int argc, char **argv);
bool initSharedMem();
void clearSharedMem();
void initLights();
void setCamera(float posX, float posY, float posZ, float targetX, float targetY, float targetZ);
void updatePixels(GLubyte* dst, int size);
void drawString(const char *str, int x, int y, float color[4], void *font);
void drawString3D(const char *str, float pos[3], float color[4], void *font);
void DrawGeometry();
void showInfo();
void showTransferRate();
void printTransferRate();


// constants
const int    SCREEN_WIDTH    = 400;
const int    SCREEN_HEIGHT   = 300;
const float  CAMERA_DISTANCE = 3.0f;
const int    TEXT_WIDTH      = 8;
const int    TEXT_HEIGHT     = 13;
const int    IMAGE_WIDTH     = 1024;
const int    IMAGE_HEIGHT    = 1024;
const int    CHANNEL_COUNT   = 4;
const int    DATA_SIZE       = IMAGE_WIDTH * IMAGE_HEIGHT * CHANNEL_COUNT;
const GLenum PIXEL_FORMAT    = GL_BGRA;

// global variables
void *font = GLUT_BITMAP_8_BY_13;
int screenWidth;
int screenHeight;
bool mouse_left_down, mouse_mid_down, mouse_right_down;
float mouseX, mouseY;
float cam_angle_x, cam_angle_y;
float cam_pos_x, cam_pos_y;
float cam_distance;
Timer timer;
float fps;
float last_draw_time;


///////////////////////////////////////////////////////////////////////////////
int main(int argc, char **argv) {
  initSharedMem();

  // register exit callback
  atexit(exitCB);

  // init GLUT and GL
  initGLUT(argc, argv);
  initGL();

  // start timer, the elapsed time will be used for updateVertices()
  timer.start();

  // the last GLUT call (LOOP)
  // window will be shown and display callback is triggered by events
  // NOTE: this call never return main().
  glutMainLoop(); /* Start GLUT event-processing loop */

  return 0;
}

///////////////////////////////////////////////////////////////////////////////
// initialize GLUT for windowing
///////////////////////////////////////////////////////////////////////////////
int initGLUT(int argc, char **argv) {
  // GLUT stuff for windowing
  // initialization openGL window.
  // it is called before any other GLUT routine
  glutInit(&argc, argv);

  glutInitDisplayMode(GLUT_RGB | GLUT_DOUBLE | GLUT_ALPHA | GLUT_DEPTH); // display mode

  glutInitWindowSize(400, 300);               // window size

  glutInitWindowPosition(100, 100);           // window location

  // finally, create a window with openGL context
  // Window will not displayed until glutMainLoop() is called
  // it returns a unique ID
  int handle = glutCreateWindow(argv[0]);     // param is the title of window

  // register GLUT callback functions
  glutDisplayFunc(displayCB);
  //glutTimerFunc(33, timerCB, 33);             // redraw only every given millisec
  glutIdleFunc(idleCB);                       // redraw only every given millisec
  glutReshapeFunc(reshapeCB);
  glutKeyboardFunc(keyboardCB);
  glutMouseFunc(mouseCB);
  glutMotionFunc(mouseMotionCB);

  return handle;
}



///////////////////////////////////////////////////////////////////////////////
// initialize OpenGL
// disable unused features
///////////////////////////////////////////////////////////////////////////////
void initGL()
{
    //@glShadeModel(GL_SMOOTH);                    // shading mathod: GL_SMOOTH or GL_FLAT
    glShadeModel(GL_FLAT);                      // shading mathod: GL_SMOOTH or GL_FLAT
    glPixelStorei(GL_UNPACK_ALIGNMENT, 4);      // 4-byte pixel alignment

    // enable /disable features
    //@glHint(GL_PERSPECTIVE_CORRECTION_HINT, GL_NICEST);
    //glHint(GL_LINE_SMOOTH_HINT, GL_NICEST);
    //glHint(GL_POLYGON_SMOOTH_HINT, GL_NICEST);
    glEnable(GL_DEPTH_TEST);
    //@glEnable(GL_LIGHTING);
    glDisable(GL_LIGHTING);
    glEnable(GL_TEXTURE_2D);
    glEnable(GL_CULL_FACE);

     // track material ambient and diffuse from surface color, call it before glEnable(GL_COLOR_MATERIAL)
    glColorMaterial(GL_FRONT_AND_BACK, GL_AMBIENT_AND_DIFFUSE);
    glEnable(GL_COLOR_MATERIAL);

    glClearColor(0, 0, 0, 0);                   // background color
    glClearStencil(0);                          // clear stencil buffer
    glClearDepth(1.0f);                         // 0 is near, 1 is far
    glDepthFunc(GL_LEQUAL);

    //initLights();
}



///////////////////////////////////////////////////////////////////////////////
// write 2d text using GLUT
// The projection matrix must be set to orthogonal before call this function.
///////////////////////////////////////////////////////////////////////////////
void drawString(const char *str, int x, int y, float color[4], void *font) {
  glPushAttrib(GL_LIGHTING_BIT | GL_CURRENT_BIT); // lighting and color mask
  glDisable(GL_LIGHTING);     // need to disable lighting for proper text color
  glDisable(GL_TEXTURE_2D);

  glColor4fv(color);          // set text color
  glRasterPos2i(x, y);        // place text position

  // loop all characters in the string
  while(*str) {
    glutBitmapCharacter(font, *str);
    ++str;
  }

  glEnable(GL_TEXTURE_2D);
  glEnable(GL_LIGHTING);
  glPopAttrib();
}



///////////////////////////////////////////////////////////////////////////////
// draw a string in 3D space
///////////////////////////////////////////////////////////////////////////////
void drawString3D(const char *str, float pos[3], float color[4], void *font) {
  glPushAttrib(GL_LIGHTING_BIT | GL_CURRENT_BIT); // lighting and color mask
  glDisable(GL_LIGHTING);     // need to disable lighting for proper text color
  glDisable(GL_TEXTURE_2D);

  glColor4fv(color);          // set text color
  glRasterPos3fv(pos);        // place text position

  // loop all characters in the string
  while(*str) {
    glutBitmapCharacter(font, *str);
    ++str;
  }

  glDisable(GL_TEXTURE_2D);
  glEnable(GL_LIGHTING);
  glPopAttrib();
}


// Draws 3D geometry using whatever OpenGL matrix is currently loaded.
void DrawGeometry() {
  glLineWidth(5.0); 
  glColor3f(1.0, 0.0, 0.0);

  glBegin(GL_LINES);
  glVertex3f(0.0, 0.0, 0.0);
  glVertex3f(0.0, 0, 1.0);

  glColor3f(0.0, 1.0, 0.0);
  glVertex3f(1.0, 0.5, 0.5);
  glVertex3f(-1.0, 0.5, 0.5);
  glEnd();

  glColor3f(0.0, 0.0, 1.0);
  glBegin(GL_QUADS);
  glNormal3f(0, 0, 1);
  glTexCoord2f(0.0f, 0.0f);   glVertex3f(-1.0f, -1.0f, 0.0f);
  glTexCoord2f(1.0f, 0.0f);   glVertex3f( 1.0f, -1.0f, 0.0f);
  glTexCoord2f(1.0f, 1.0f);   glVertex3f( 1.0f,  1.0f, 0.0f);
  glTexCoord2f(0.0f, 1.0f);   glVertex3f(-1.0f,  1.0f, 0.0f);
  glEnd();
}



///////////////////////////////////////////////////////////////////////////////
// initialize global variables
///////////////////////////////////////////////////////////////////////////////
bool initSharedMem() {
  screenWidth = SCREEN_WIDTH;
  screenHeight = SCREEN_HEIGHT;

  mouse_left_down = mouse_right_down = mouse_mid_down = false;
  mouseX = mouseY = 0;

  cam_angle_x = cam_angle_y = 0;
  cam_distance = CAMERA_DISTANCE;
  return true;
}



///////////////////////////////////////////////////////////////////////////////
// clean up shared memory
///////////////////////////////////////////////////////////////////////////////
void clearSharedMem()
{
}


///////////////////////////////////////////////////////////////////////////////
// initialize lights
///////////////////////////////////////////////////////////////////////////////
void initLights() {
  // set up light colors (ambient, diffuse, specular)
  GLfloat lightKa[] = {.2f, .2f, .2f, 1.0f};  // ambient light
  GLfloat lightKd[] = {.7f, .7f, .7f, 1.0f};  // diffuse light
  GLfloat lightKs[] = {1, 1, 1, 1};           // specular light
  glLightfv(GL_LIGHT0, GL_AMBIENT, lightKa);
  glLightfv(GL_LIGHT0, GL_DIFFUSE, lightKd);
  glLightfv(GL_LIGHT0, GL_SPECULAR, lightKs);

  // position the light
  float lightPos[4] = {0, 0, 20, 1}; // positional light
  glLightfv(GL_LIGHT0, GL_POSITION, lightPos);

  glEnable(GL_LIGHT0);                        // MUST enable each light source after configuration
}

///////////////////////////////////////////////////////////////////////////////
// set camera position and lookat direction
///////////////////////////////////////////////////////////////////////////////
void setCamera(float posX, float posY, float posZ, float targetX, float targetY, float targetZ) {
  glMatrixMode(GL_MODELVIEW);
  glLoadIdentity();
  gluLookAt(posX, posY, posZ, targetX, targetY, targetZ, 0, 1, 0); // eye(x,y,z), focal(x,y,z), up(x,y,z)
}


///////////////////////////////////////////////////////////////////////////////
// display info messages
///////////////////////////////////////////////////////////////////////////////
void showInfo() {
  // backup current model-view matrix
  glPushMatrix();                     // save current modelview matrix
  glLoadIdentity();                   // reset modelview matrix

  // set to 2D orthogonal projection
  glMatrixMode(GL_PROJECTION);     // switch to projection matrix
  glPushMatrix();                  // save current projection matrix
  glLoadIdentity();                // reset projection matrix
  gluOrtho2D(0, screenWidth, 0, screenHeight); // set to orthogonal projection

  float color[4] = {1, 1, 1, 1};

  std::stringstream ss;

  ss << "FPS: " << fps << " ms" << std::ends;
  drawString(ss.str().c_str(), 1, screenHeight-(1*TEXT_HEIGHT), color, font);
  // Clear string
  ss.str("");

  // unset floating format
  ss << std::resetiosflags(std::ios_base::fixed | std::ios_base::floatfield);

  // restore projection matrix
  glPopMatrix();                   // restore to previous projection matrix

  // restore modelview matrix
  glMatrixMode(GL_MODELVIEW);      // switch to modelview matrix
  glPopMatrix();                   // restore to previous modelview matrix
}




///////////////////////////////////////////////////////////////////////////////
// set projection matrix as orthogonal
///////////////////////////////////////////////////////////////////////////////
void toOrtho() {
  // set viewport to be the entire window
  glViewport(0, 0, (GLsizei)screenWidth, (GLsizei)screenHeight);

  // set orthographic viewing frustum
  glMatrixMode(GL_PROJECTION);
  glLoadIdentity();
  gluOrtho2D(0, screenWidth, 0, screenHeight); // set to orthogonal projection

  // switch to modelview matrix in order to set scene
  glMatrixMode(GL_MODELVIEW);
  glLoadIdentity();
}



///////////////////////////////////////////////////////////////////////////////
// set the projection matrix as perspective
///////////////////////////////////////////////////////////////////////////////
void toPerspective() {
  // set viewport to be the entire window
  glViewport(0, 0, (GLsizei)screenWidth, (GLsizei)screenHeight);

  // set perspective viewing frustum
  glMatrixMode(GL_PROJECTION);
  glLoadIdentity();
  gluPerspective(60.0f, (float)(screenWidth)/screenHeight, 1.0f, 1000.0f); // FOV, AspectRatio, NearClip, FarClip

  // switch to modelview matrix in order to set scene
  glMatrixMode(GL_MODELVIEW);
  glLoadIdentity();
}



//=============================================================================
// CALLBACKS
//=============================================================================

void displayCB() {
  // clear buffer
  glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT | GL_STENCIL_BUFFER_BIT);

  // save the initial ModelView matrix before modifying ModelView matrix
  glPushMatrix();

  // transform camera
  glTranslatef(0, 0, -cam_distance);
  glRotatef(cam_angle_x, 1, 0, 0);   // pitch
  glRotatef(cam_angle_y, 0, 1, 0);   // heading
  glTranslatef(-cam_pos_x, -cam_pos_y, 0);

  DrawGeometry();

  // draw info messages
  float current_time = timer.getElapsedTimeInMicroSec();
  fps = 1000000.0 / (current_time - last_draw_time);
  last_draw_time = current_time;

  showInfo();
  glPopMatrix();
  glutSwapBuffers();
}

void reshapeCB(int width, int height) {
  screenWidth = width;
  screenHeight = height;
  toPerspective();
}

void timerCB(int millisec) {
  glutTimerFunc(millisec, timerCB, millisec);
  glutPostRedisplay();
}

void idleCB() {
  glutPostRedisplay();
}

void keyboardCB(unsigned char key, int x, int y) {
  switch(key) {
  case 27: // ESCAPE
    exit(0);
    break;
  case 'm':
    toPerspective();
    break;
  }
}


void mouseCB(int button, int state, int x, int y) {
  mouseX = x;
  mouseY = y;

  if(button == GLUT_LEFT_BUTTON) {
    if(state == GLUT_DOWN) {
      mouse_left_down = true;
    } else if(state == GLUT_UP) {
      mouse_left_down = false;
    }
  } else if(button == GLUT_RIGHT_BUTTON) {
    if(state == GLUT_DOWN) {
      mouse_right_down = true;
    } else if(state == GLUT_UP) {
      mouse_right_down = false;
    }
  } else if(button == GLUT_MIDDLE_BUTTON) {
    if(state == GLUT_DOWN) {
      mouse_mid_down = true;
    } else if(state == GLUT_UP) {
      mouse_mid_down = false;
    }
  }
}

void mouseMotionCB(int x, int y) {
  if(mouse_left_down) {
    cam_angle_y += (x - mouseX)*.5f;
    cam_angle_x += (y - mouseY)*.5f;
    mouseX = x;
    mouseY = y;
  }
  if(mouse_right_down) {
    cam_distance += (y - mouseY) * 0.2f;
    if(cam_distance < 2.0f) {
      cam_distance = 2.0f;
    }

    mouseY = y;
  }

  if (mouse_mid_down) {
    cam_pos_x -= (x - mouseX)/screenWidth*2.0f;
    cam_pos_y += (y - mouseY)/screenHeight*2.0f;
    mouseX = x;
    mouseY = y;
  }
}

void exitCB() {
  clearSharedMem();
}
