#include <stdio.h>

#include "kinematic_pair.h"
#include "leg.h"
#include "ik_solver.h"
#include "timer.h"
#include "interpolators.h"
#include "test_parts.h"
#include "viewer.h"

#include "animation_examples.h"


/********************************************************/
// An example of animation using raw joint controls.
/********************************************************/
struct ThreadArgs {
  Leg<3> leg;
};

void* AnimationLoop(void* argptr) {
  RevoluteJointCommand coxa_a(0.0, 1.0);
  RevoluteJointCommand femur_a(M_PI/4.0, 1.0);
  RevoluteJointCommand tibia_a(-M_PI/2.0, 1.0);
  RevoluteJointCommand coxa_b(M_PI/4.0, .5);
  RevoluteJointCommand femur_b(0.0, 0.5);
  RevoluteJointCommand tibia_b(-M_PI/4.0, 0.5);
  RevoluteJointCommand joints_a[3] = {coxa_a, femur_a, tibia_a};
  RevoluteJointCommand joints_b[3] = {coxa_b, femur_b, tibia_b};
  LegCommand<3> command_a(joints_a);
  LegCommand<3> command_b(joints_b);

  bool state_a = true;
  Timer timer;
  timer.start();
  double last_time = timer.getElapsedTimeInSec();
  double current_time = timer.getElapsedTimeInSec();
  ThreadArgs* args = (ThreadArgs*) argptr;
  while (true) {
    if (!args->leg.IsMoving()) {
      if (state_a) {
        args->leg.SetCommand(command_b);
        state_a = false;
      } else {
        args->leg.SetCommand(command_a);
        state_a = true;
      }
    }
    
    current_time = timer.getElapsedTimeInSec();
    args->leg.UpdateState(current_time - last_time);
    last_time = current_time;

    usleep(10000);
  }
  return NULL;
}

void TestAnimation() {
  // Get a test leg
  ThreadArgs thread_args = {GetTestLeg()};
  double leg_state_a[3] = {0.0, M_PI/4.0, -M_PI/2.0};
  thread_args.leg.SetState(leg_state_a);

  // Create a thread to move the leg over time
  pthread_t movement;
  pthread_create(&movement, NULL, AnimationLoop, (void*) &thread_args);

  // Start a window to draw the leg as it moves.
  StartWindow(&(thread_args.leg));
  pthread_join(movement, NULL);
}

/********************************************************/
// An example of animation using a IK controlled test leg.
/********************************************************/
namespace IKExample {
  struct ThreadArgs {
    Leg<3> leg;
    LegController<3> cont;
  };

  void* AnimationLoop(void* argptr) {
    Eigen::Vector3d point_a(2.5, 1.0, 0.0);
    Eigen::Vector3d point_b(2.5, -1.0, 0.0);
    LinearPath path_a(point_b, point_a);
    // One curved bezier path of height 1.0
    StepPath path_b(point_a, point_b, 2.0);

    double deadline_a = 1.0;
    double deadline_b = 1.0;

    Timer timer;
    timer.start();
    double last_time = timer.getElapsedTimeInSec();
    double current_time = timer.getElapsedTimeInSec();
    ThreadArgs* args = (ThreadArgs*) argptr;

    // Set the initial command so that it moves to point_a.
    Eigen::Vector3d starting_point = args->cont.GetEndpoint();
    LinearPath path_to_start(starting_point, point_a);
    args->cont.SetControl(&path_to_start, deadline_a);
    bool state_a = true;

    LegCommand<3> command;
    while (true) {
      if (!args->cont.InProgress()) {
        if (state_a) {
          args->cont.SetControl(&path_b, deadline_b);
          state_a = false;
        } else {
          args->cont.SetControl(&path_a, deadline_a);
          state_a = true;
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

void TestAnimationIK() {
  // Get a test leg
  Leg<3> test_leg = GetTestLeg();
  IKExample::ThreadArgs thread_args;
  thread_args.leg = test_leg;
  thread_args.cont = GetTestLegController(&thread_args.leg);
  
  double leg_state_a[3] = {0.0, M_PI/4.0, -M_PI/2.0};
  thread_args.cont.SetState(leg_state_a);

  // Create a thread to move the leg over time
  pthread_t movement;
  pthread_create(&movement, NULL, IKExample::AnimationLoop, (void*) &thread_args);

  // Start a window to draw the leg as it moves.
  StartWindow(&(thread_args.cont));
  pthread_join(movement, NULL);
}

/********************************************************/
// An example of randomized animation using a IK controlled test leg.
/********************************************************/
namespace RandomExample {
  struct ThreadArgs {
    Leg<3> leg;
    LegController<3> cont;
  };

  void* AnimationLoop(void* argptr) {
    Timer timer;
    timer.start();
    double last_time = timer.getElapsedTimeInSec();
    double current_time = timer.getElapsedTimeInSec();
    ThreadArgs* args = (ThreadArgs*) argptr;

    double space_lim = 3.0;
    double vel_lim = 5.0;
    Eigen::Vector3d center(2.0, 0.0, 0.0);
    double deadline = 2.0;

    // Start wherever the leg is right now.
    Eigen::Vector3d start = args->cont.GetEndpoint();
    Eigen::Vector3d start_deriv(0.0, 0.0, 0.0); 

    // Randomize ending.
    Eigen::Vector3d end(center[0] + RandFloat(-space_lim, space_lim), 
                        center[1] + RandFloat(-space_lim, space_lim),
                        center[2] + RandFloat(-space_lim, space_lim));
    Eigen::Vector3d end_deriv(RandFloat(-vel_lim, vel_lim), 
                              RandFloat(-vel_lim, vel_lim),
                              RandFloat(-vel_lim, vel_lim));
    CubicHermitePath path(start, end, start_deriv, end_deriv);


    // Set the initial command so that it moves to point_a.
    args->cont.SetControl(&path, deadline);
    LegCommand<3> command;

    // The control/simulation update loop.
    while (true) {
      current_time = timer.getElapsedTimeInSec();

      // Update the leg simulation with the current control inputs.
      args->leg.UpdateState(current_time - last_time);

      if (!args->cont.InProgress()) {
        // Cycle the previous endpoint to the new start point
        start = end;
        start_deriv = end_deriv;

        // Randomize the new endpoints.
        end[0] = center[0] + RandFloat(-space_lim, space_lim); 
        end[1] = center[1] + RandFloat(-space_lim, space_lim); 
        end[2] = center[2] + RandFloat(-space_lim, space_lim); 

        end_deriv[0] = RandFloat(-vel_lim, vel_lim); 
        end_deriv[1] = RandFloat(-vel_lim, vel_lim); 
        end_deriv[2] = RandFloat(-vel_lim, vel_lim); 

        // Create a new path and send it to the controller.
        path.SetPoints(start, end, start_deriv, end_deriv);
        args->cont.SetControl(&path, deadline);
      }

      // Update the controller.
      args->cont.UpdateState(current_time - last_time, &command);

      // Set the new control inputs on the simulation.
      args->leg.SetCommand(command);
      last_time = current_time;
      usleep(10000);
    }
    return NULL;
  }
}

void TestAnimationRandom() {
  // Get a test leg
  Leg<3> test_leg = GetTestLeg();
  RandomExample::ThreadArgs thread_args;
  thread_args.leg = test_leg;
  thread_args.cont = GetTestLegController(&thread_args.leg);
  
  double leg_state_a[3] = {0.0, M_PI/4.0, -M_PI/2.0};
  thread_args.cont.SetState(leg_state_a);

  // Create a thread to move the leg over time
  pthread_t movement;
  pthread_create(&movement, NULL, RandomExample::AnimationLoop, (void*) &thread_args);

  // Start a window to draw the leg as it moves.
  StartWindow(&(thread_args.cont));
  pthread_join(movement, NULL);
}


/********************************************************/
// An example of animation using a Chassis Controller
/********************************************************/
/*
struct ThreadArgsChassis {
  ChassisController<6, 3> cont;
};

void* AnimationLoopChassis(void* argptr) {
  Eigen::Vector3d point_a(2.5, 1.0, 0.0);
  Eigen::Vector3d point_b(2.5, -1.0, 0.0);
  LinearPath path_a(point_b, point_a);
  // One curved bezier path of height 1.0
  StepPath path_b(point_a, point_b, 2.0);

  double deadline_a = 1.0;
  double deadline_b = 1.0;

  Timer timer;
  timer.start();
  double last_time = timer.getElapsedTimeInSec();
  double current_time = timer.getElapsedTimeInSec();
  ThreadArgsIK* args = (ThreadArgsIK*) argptr;

  // Set the initial command so that it moves to point_a.
  Eigen::Vector3d starting_point = args->cont.GetEndpoint();
  LinearPath path_to_start(starting_point, point_a);
  args->cont.SetControl(&path_to_start, deadline_a);
  bool state_a = true;

  LegCommand<3> command;
  while (true) {
    if (!args->cont.IsMoving()) {
      if (state_a) {
        args->cont.SetControl(&path_b, deadline_b);
        state_a = false;
      } else {
        args->cont.SetControl(&path_a, deadline_a);
        state_a = true;
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

void TestAnimationChassis() {
  ThreadArgsChassis thread_args;
  thread_args.cont = GetTestChassisController<6>();
  
  // Create a thread to move the leg over time
  pthread_t movement;
  pthread_create(&movement, NULL, AnimationLoopChassis, (void*) &thread_args);

  // Start a window to draw the leg as it moves.
  StartWindow(&(thread_args.cont));
  pthread_join(movement, NULL);
}
*/
