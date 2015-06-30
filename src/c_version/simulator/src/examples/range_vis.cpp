
#include "viewer.h"
#include "range_vis.h"
#include "test_parts.h"

void TestRangeVis() {
  Leg<3> leg = GetTestLeg();
  LegController<3> test_leg = GetTestLegController(&leg);
  RangeVis<3> vis(&test_leg, -5.0, 5.0, 10, 5);
  StartWindow(&vis);
}


void TestPlaneVis() {
  Leg<3> leg = GetTestLeg();
  LegController<3> test_leg = GetTestLegController(&leg);
  PlanarRangeVis<3> vis(Eigen::Vector3d(2, -1, 0.0), Eigen::Vector3d(-1, .1, 0.0), &test_leg, 10, 5);
  StartWindow(&vis);
}
