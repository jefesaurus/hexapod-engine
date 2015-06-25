#include <stdio.h>

#include "animation_examples.h"
#include "basic_examples.h"
#include "range_vis.h"
#include "minimization.h"

int main() {
  // Draws one static leg
  //StaticLegDrawTest();

  // Draws a chassis of with n legs randomized in state.
  //StaticChassisDrawTest<6>();
  //StaticChassisDrawTest<1000>();

  // Some miscellaneous intermediate tests.
  //TestIK();
  //TestAnimation();
  //PathTest();

  // Shows the range visualization for a test leg
  //TestRangeVis();

  // Animate one leg in motion.
  //TestAnimationIK();
  TestAnimationRandom();

  // An example of how to make splines for position and orientation.
  //PosePathTest();

  //TestMinimization();
  return 0;
}
