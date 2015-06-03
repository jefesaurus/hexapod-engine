#include "kinematic_pair.h"
#include "leg.h"
#include "viewer.h"
#include "drawing_primitives.h"
#include "ik_solver.h"
#include "timer.h"

#include <stdio.h>


// Test Leg config: Alpha, R, D
RevoluteJoint coxa(M_PI/2.0, .5, .5);
RevoluteJoint femur(0.0, 1.5, -.2);
RevoluteJoint tibia(0.0, 2.0, .2);
RevoluteJoint joints[3] = {coxa, femur, tibia};
Leg<3> test_leg(joints);
IK3DoF ik_3dof(coxa, femur, tibia);
LegController<3> cont(test_leg, &ik_3dof);

double RandFloat(double min, double max) {
  double f = (double)rand() / RAND_MAX;
  return min + f * (max - min);
}

void* AnimationLoop(void* args) {
  double leg_state_a[3] = {0.0, M_PI/4.0, -M_PI/2.0};
  double leg_state_b[3] = {M_PI/4.0, 0.0, -M_PI/4.0};
  double velocities_a[3] = {1.0, 1.0, 1.0};
  double velocities_b[3] = {0.5, .5, .5};
  bool state_a = true;
  Timer timer;
  timer.start();
  double last_time = timer.getElapsedTimeInSec();
  double current_time = timer.getElapsedTimeInSec();
  while (true) {
    if (!test_leg.IsMoving()) {
      if (state_a) {
        test_leg.SetJointCommands(leg_state_b, velocities_b);
        state_a = false;
      } else {
        test_leg.SetJointCommands(leg_state_a, velocities_a);
        state_a = true;
      }
    }
    
    current_time = timer.getElapsedTimeInSec();
    test_leg.UpdateState(current_time - last_time);
    last_time = current_time;

    usleep(10000);
  }
  return NULL;
}


void TestAnimation() {
  double leg_state_a[3] = {0.0, M_PI/4.0, -M_PI/2.0};
  test_leg.SetState(leg_state_a);
  pthread_t movement;
  pthread_create(&movement, NULL, AnimationLoop, (void*) NULL);
  StartWindow(&test_leg);
  pthread_join(movement, NULL);
}


void TestIK() {
  double goal[3];
  double solution[3];
  Eigen::Vector4d end_effector;
  double range = 4.0;

  double biggest_error = 0.0;
  double total_error = 0.0;

  int test_iters = 1e7;
  int n_solved = 0;

  for (int i = 0; i < test_iters; i++) {
    goal[0] = RandFloat(-range, range);
    goal[1] = RandFloat(-range, range);
    goal[2] = RandFloat(-range, range);

    int solved = ik_3dof.Solve(goal[0], goal[1], goal[2], solution, 3);
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

void GenericTest() {
  double leg_state[3] = {0.0, M_PI/4.0, -M_PI/2.0};
  test_leg.SetState(leg_state);
  StartWindow(&test_leg);
}

int main() {
  //GenericTest();
  //TestIK();
  TestAnimation();
  return 0;
}
