#include <stdio.h>
#include <cassert>

#include "basic_examples.h"
#include "kinematic_pair.h"
#include "leg.h"
#include "chassis.h"
#include "viewer.h"
#include "ik_solver.h"
#include "interpolators.h"
#include "test_parts.h"
#include "utils.h"
#include "pose.h"


void TestIK() {
  Eigen::Vector3d goal;
  double solution[3];
  Eigen::Vector3d end_effector;
  double range = 4.0;

  double biggest_error = 0.0;
  double total_error = 0.0;

  int test_iters = 1e7;
  int n_solved = 0;

  Leg<3> leg = GetTestLeg();
  LegController<3> test_leg = GetTestLegController(&leg);

  for (int i = 0; i < test_iters; i++) {
    goal[0] = RandFloat(-range, range);
    goal[1] = RandFloat(-range, range);
    goal[2] = RandFloat(-range, range);

    double solved = test_leg.GetJointCommands(goal, solution);
    if (solved >= 0) {
      test_leg.SetState(solution);
      end_effector = test_leg.GetEndpoint();
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

void PosePathTest() {
  double start_angle = 0.0;
  double end_angle = M_PI/2.0;
  Pose offset(1.0, 0.0, 0.0, 0.0, 0.0, 0.0);
  Pose p1(0.0, 0.0, 0.0, start_angle, 0.0, 0.0);
  Pose p2(0.0, 0.0, 1.0, end_angle, 0.0, 0);


  Eigen::Vector3d m1(2.0, 0.0, 0.0);
  Eigen::Vector3d m2(0.0, -2.0, 0.0);
  PoseSpline path(p1, p2, m1, m2);
  OffsetPoseGen derivative_path(&path, offset);

  Scene scene;
  scene.AddDrawable(&path);
  scene.AddDrawable(&derivative_path);

  StartWindow(&scene);
}

void ChassisPosePathTest() {
  double start_angle = 0.0;
  double end_angle = M_PI/6.0;

  Pose p1(0.0, 0.0, 0.0, start_angle, 0.0, 0.0);
  Pose p2(1.0, 0.0, 0.0, end_angle, 0.0, 0.0);
  Eigen::Vector3d m1(0.0, 0.0, 0.0);
  Eigen::Vector3d m2(0.0, 0.0, 0.0);

  PoseSpline path(p1, p2, m1, m2);

  ChassisController<6, 3> test_chassis = GetTestChassisController<6>();
  test_chassis.SetPose(Pose(0, 0, 1.0, 0, 0, 0));
  test_chassis.SetControl(std::unique_ptr<PoseGen>(new PoseSpline(p1, p2, m1, m2)));

  Scene scene;
  scene.AddDrawable(&test_chassis);
  StartWindow(&scene);
}
