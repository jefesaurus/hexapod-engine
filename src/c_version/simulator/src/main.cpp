#include "kinematic_pair.h"
#include "leg.h"
#include "viewer.h"
#include "drawing_primitives.h"
#include "ik_solver.h"

#include <stdio.h>


// Test Leg config: Alpha, R, D
RevoluteJoint coxa(M_PI/2.0, .5, 0);
RevoluteJoint femur(0.0, 1.5, .8);
RevoluteJoint tibia(0.0, 2.0, -.3);
RevoluteJoint joints[3] = {coxa, femur, tibia};
Leg<3> front_right(joints);
IK3DoF ik_3dof(coxa, femur, tibia);
LegController<3> cont(front_right, &ik_3dof);

double RandFloat(double min, double max) {
  double f = (double)rand() / RAND_MAX;
  return min + f * (max - min);
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
      front_right.SetState(solution);
      end_effector = front_right.ToGlobal();
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
  front_right.SetState(leg_state);
  StartWindow(&front_right);
}

int main() {
  GenericTest();
  //TestIK();
  return 0;
}
