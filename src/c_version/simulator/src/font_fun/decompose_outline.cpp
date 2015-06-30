
#include <pthread.h>

#include "test_parts.h"
#include "leg.h"
#include "timer.h"
#include "font_fun.h"
#include "drawing_primitives.h"
#include "viewer.h"

int MoveTo(const FT_Vector* to, void* fp) {
  DecompositionData* data = (DecompositionData*)fp;
  //printf("Move To: (%ld, %ld)\n", to->x + data->pen.x, to->y + data->pen.y);
  data->last_point.x = to->x;
  data->last_point.y = to->y;
  return 0;
}

int LineTo(const FT_Vector* to, void* fp) {
  DecompositionData* data = (DecompositionData*)fp;
  Eigen::Vector3d p_1(data->last_point.x + data->pen.x, data->last_point.y + data->pen.y, 0.0);
  Eigen::Vector3d p_2(to->x + data->pen.x, to->y + data->pen.y, 0.0);
  data->splines.push_back(new LinearPath(p_1*data->scale, p_2*data->scale));
  data->last_point.x = to->x;
  data->last_point.y = to->y;
  return 0;
}

int ConicTo(const FT_Vector* p1, const FT_Vector* to, void* fp) {
  DecompositionData* data = (DecompositionData*)fp;
  Eigen::Vector3d p_1(data->last_point.x + data->pen.x, data->last_point.y + data->pen.y, 0.0);
  Eigen::Vector3d p_2(p1->x + data->pen.x, p1->y + data->pen.y, 0.0);
  Eigen::Vector3d p_3(to->x + data->pen.x, to->y + data->pen.y, 0.0);
  data->splines.push_back(new ConicBezierPath(p_1*data->scale, p_2*data->scale, p_3*data->scale));
  data->last_point.x = to->x;
  data->last_point.y = to->y;
  return 0;
}

int CubicTo(const FT_Vector* p1, const FT_Vector* p2, const FT_Vector* to, void* fp) {
  DecompositionData* data = (DecompositionData*)fp;
  Eigen::Vector3d p_1(data->last_point.x + data->pen.x, data->last_point.y + data->pen.y, 0.0);
  Eigen::Vector3d p_2(p1->x + data->pen.x, p1->y + data->pen.y, 0.0);
  Eigen::Vector3d p_3(p2->x + data->pen.x, p2->y + data->pen.y, 0.0);
  Eigen::Vector3d p_4(to->x + data->pen.x, to->y + data->pen.y, 0.0);
  data->splines.push_back(new CubicBezierPath(p_1*data->scale, p_2*data->scale, p_3*data->scale, p_4*data->scale));
  data->last_point.x = to->x;
  data->last_point.y = to->y;
  return 0;
}

namespace FontScene {
  struct ThreadArgs {
    Leg<3> leg;
    LegController<3> cont;
    std::vector<PathGen*> splines;
  };

  void* AnimationLoop(void* argptr) {
    Timer timer;
    timer.start();
    double last_time = timer.getElapsedTimeInSec();
    double current_time = timer.getElapsedTimeInSec();
    ThreadArgs* args = (ThreadArgs*) argptr;

    double deadline = .1;

    // Set the initial command so that it moves to first spline
    int current_spline = -1;
    int next_spline = 0;
    Eigen::Vector3d current = args->cont.GetEndpoint();
    Eigen::Vector3d next_spline_start = args->splines[next_spline]->Value(0.0);
    LinearPath makeup;
    double epsilon = 1e-10;
    if ((current - next_spline_start).squaredNorm() > epsilon){
      makeup = LinearPath(current, next_spline_start);
      args->cont.SetControl(&makeup, 0.0);
    } 

    LegCommand<3> command;
    while (true) {
      if (!args->cont.InProgress()) {
        // Check for a break in splines
        current = args->cont.GetEndpoint();
        next_spline_start = args->splines[next_spline]->Value(0.0);
        if ((current - next_spline_start).squaredNorm() > epsilon){
          makeup = LinearPath(current, next_spline_start);
          args->cont.SetControl(&makeup, 0.0);
        } else {
          current_spline = next_spline;
          next_spline ++;
          next_spline %= args->splines.size();
          args->cont.SetControl(args->splines[current_spline], deadline);
        }
      }
      
      current_time = timer.getElapsedTimeInSec();
      args->leg.UpdateState(current_time - last_time);
      args->cont.UpdateState(current_time - last_time, &command);
      args->leg.SetCommand(command);
      last_time = current_time;

      usleep(10000);
    }
    return NULL;
  }
}

void FontFun(int argc, char**  argv) {
  if ( argc < 2 ) {
    fprintf ( stderr, "usage: %s font_file (optional text)\n", argv[0] );
    exit( 1 );
  }
  std::string text = "";
  if (argc > 2) {
    text += argv[2];
    for (int i = 3; i < argc; i++) {
      text += " ";
      text += argv[i];
    }
  } else {
    text = std::string("ayy lmao");
  }

  std::string font_path(argv[1]);
  FreetypeSplineInterface char_splines(font_path);
  std::vector<PathGen*> splines = char_splines.GetSplines(text, 2e-4);
  Pose text_origin(1.5, 2.5, 0.0, -M_PI/2.0, 0.0, 0.0);
  Scene scene;
  for (uint i = 0; i < splines.size(); i++) {
    splines[i]->Transform(text_origin.FromFrameMat());
    scene.AddDrawable(splines[i]);
  }

  // Get a test leg
  Leg<3> test_leg = GetTestLeg();
  FontScene::ThreadArgs thread_args;
  thread_args.leg = test_leg;
  thread_args.cont = GetTestLegController(&thread_args.leg);
  thread_args.splines = splines;

  // Create a thread to move the leg over time
  pthread_t movement;
  pthread_create(&movement, NULL, FontScene::AnimationLoop, (void*) &thread_args);

  scene.AddDrawable(&(thread_args.cont));

  // Start a window to draw the leg as it moves.
  StartWindow(&scene);
  pthread_join(movement, NULL);
}
