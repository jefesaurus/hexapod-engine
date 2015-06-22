
#include "viewer.h"
#include "range_vis.h"
#include "test_parts.h"

void TestRangeVis() {
  Leg<3> leg = GetTestLeg();
  LegController<3> test_leg = GetTestLegController(&leg);
  double angles[3] = {0.0, 0.0, 0.0};
  leg.SetState(angles);
  RangeVis<3> vis(&test_leg, -5.0, 5.0, 10, 5);

  StartWindow(&vis);
}
