#ifndef BASIC_EXAMPLES_H_
#define BASIC_EXAMPLES_H_



void TestIK();
void StaticLegDrawTest();
void PathTest();


#include "chassis.h"
#include "test_parts.h"
#include "viewer.h"
#include "utils.h"
template <int n_legs>
void StaticChassisDrawTest() {
  Chassis<n_legs, 3> test_chassis = GetTestChassis<n_legs>();

  for (int i = 0; i < n_legs; i++) {
    double leg_state[3] = {0.0, RandFloat(0.0, M_PI/2.0), RandFloat(0.0, -M_PI)};
    test_chassis.SetState(i, leg_state);
  }

  StartWindow(&test_chassis);
}

#endif // BASIC_EXAMPLES_H_
