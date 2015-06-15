#include "kinematic_pair.h"
#include "leg.h"
#include "viewer.h"
#include "drawing_primitives.h"
#include "ik_solver.h"
#include "timer.h"
#include "interpolators.h"

#include <stdio.h>
#include <cassert>


// Test Leg config: Alpha, R, D
void GetTestJoints(RevoluteJoint* joints) {
  joints[0] = RevoluteJoint(M_PI/2.0, .5, .5);
  joints[1] = RevoluteJoint(0.0, 1.5, -.2);
  joints[2] = RevoluteJoint(0.0, 2.0, .2);
}

Leg<3> GetTestLeg() {
  RevoluteJoint joints[3];
  GetTestJoints(joints);
  return Leg<3>(joints);
}

LegController<3> GetTestLegController() {
  RevoluteJoint joints[3];
  GetTestJoints(joints);

  // TODO Don't leak memory here :p
  // Use shared_ptr or whatever instead.
  IKSolver* ik_3dof = new IK3DoF(joints[0], joints[1], joints[2]);
  return LegController<3>(joints, ik_3dof);
}


struct ThreadArgs {
  Leg<3> leg;
};

void* AnimationLoop(void* argptr) {
  double leg_state_a[3] = {0.0, M_PI/4.0, -M_PI/2.0};
  double leg_state_b[3] = {M_PI/4.0, 0.0, -M_PI/4.0};
  double velocities_a[3] = {1.0, 1.0, 1.0};
  double velocities_b[3] = {0.5, .5, .5};
  bool state_a = true;
  Timer timer;
  timer.start();
  double last_time = timer.getElapsedTimeInSec();
  double current_time = timer.getElapsedTimeInSec();
  ThreadArgs* args = (ThreadArgs*) argptr;
  while (true) {
    if (!args->leg.IsMoving()) {
      if (state_a) {
        args->leg.SetJointCommands(leg_state_b, velocities_b);
        state_a = false;
      } else {
        args->leg.SetJointCommands(leg_state_a, velocities_a);
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

struct ThreadArgsIK {
  LegController<3> cont;
};

void* AnimationLoopIK(void* argptr) {
  Eigen::Vector3d point_a(2.5, 1.0, 0.0);
  Eigen::Vector3d point_b(2.5, -1.0, 0.0);
  LinearPath path_a(point_b, point_a);
  // One curved bezier path of height 1.0
  StepPath path_b(point_a, point_b, 1.0);

  double deadline_a = 1.0;
  double deadline_b = 1.0;

  Timer timer;
  timer.start();
  double last_time = timer.getElapsedTimeInSec();
  double current_time = timer.getElapsedTimeInSec();
  ThreadArgsIK* args = (ThreadArgsIK*) argptr;

  // Set the initial command so that it moves to point_a.
  Eigen::Vector3d starting_point = args->cont.ToGlobal3();
  LinearPath path_to_start(starting_point, point_a);
  args->cont.SetCommand(&path_to_start, deadline_a);
  bool state_a = true;

  while (true) {
    if (!args->cont.IsMoving()) {
      if (state_a) {
        args->cont.SetCommand(&path_b, deadline_b);
        state_a = false;
      } else {
        args->cont.SetCommand(&path_a, deadline_a);
        state_a = true;
      }
    }
    
    current_time = timer.getElapsedTimeInSec();
    args->cont.UpdateState(current_time - last_time);
    last_time = current_time;

    usleep(10000);
  }
  return NULL;
}


void TestAnimationIK() {
  // Get a test leg
  ThreadArgsIK thread_args = {GetTestLegController()};
  double leg_state_a[3] = {0.0, M_PI/4.0, -M_PI/2.0};
  thread_args.cont.SetState(leg_state_a);

  // Create a thread to move the leg over time
  pthread_t movement;
  pthread_create(&movement, NULL, AnimationLoopIK, (void*) &thread_args);

  // Start a window to draw the leg as it moves.
  StartWindow(&(thread_args.cont));
  pthread_join(movement, NULL);
}


double RandFloat(double min, double max) {
  double f = (double)rand() / RAND_MAX;
  return min + f * (max - min);
}

void TestIK() {
  Eigen::Vector3d goal;
  double solution[3];
  Eigen::Vector4d end_effector;
  double range = 4.0;

  double biggest_error = 0.0;
  double total_error = 0.0;

  int test_iters = 1e7;
  int n_solved = 0;

  LegController<3> test_leg = GetTestLegController();

  for (int i = 0; i < test_iters; i++) {
    goal[0] = RandFloat(-range, range);
    goal[1] = RandFloat(-range, range);
    goal[2] = RandFloat(-range, range);

    int solved = test_leg.GetJointCommands(goal, solution);
    if (solved == 0) {
      test_leg.SetState(solution);
      end_effector = test_leg.ToGlobal();
      total_error = fabs(goal[0] - end_effector[0]) + fabs(goal[1] - end_effector[1]) + fabs(goal[2] - end_effector[2]);
      if (total_error > biggest_error) {
        biggest_error = total_error;
      }
      n_solved ++;
    }
  }
  printf("Biggest Error: %f, Percent Solved: %f\n", biggest_error, ((float)n_solved)/test_iters);
}

void StaticLegDrawTest() {
  double leg_state[3] = {0.0, M_PI/4.0, -M_PI/2.0};
  Leg<3> test_leg = GetTestLeg();
  test_leg.SetState(leg_state);
  StartWindow(&test_leg);
}

void PathTest() {
  Eigen::Vector3d point_a(2.5, 1.0, 0.0);
  Eigen::Vector3d point_b(2.5, -1.0, 0.0);
  LinearPath path_a(point_b, point_a);
  int num = 10;
  for (int i = 0; i < num; i++) {
    printf("%f, %f\n", (float)i/num, path_a.Value((float)i/num)[1]);
  }
}

int main() {
  //StaticLegDrawTest();
  //TestIK();
  //TestAnimation();
  TestAnimationIK();
  //PathTest();
  return 0;
}
