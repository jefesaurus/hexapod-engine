
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
  PlanarRangeVis<3> vis(Eigen::Vector3d(0, 0, -.2), Eigen::Vector3d(0, 0, 1.0), &test_leg, 10, 5);
  StartWindow(&vis);
}

void TestAllVis() {
  Leg<3> leg = GetTestLeg();
  LegController<3> test_leg = GetTestLegController(&leg);
  PlanarRangeVis<3> vis1(Eigen::Vector3d(0, 0, -.2), Eigen::Vector3d(0, 0, 1.0), &test_leg, 10, 5);
  PlanarRangeVis<3> vis2(Eigen::Vector3d(0, 1, 0), Eigen::Vector3d(0, -1, 0), &test_leg, 10, 5);
  PlanarRangeVis<3> vis3(Eigen::Vector3d(1, 1, 0), Eigen::Vector3d(-1, -1, 0), &test_leg, 10, 5);
  PlanarRangeVis<3> vis4(Eigen::Vector3d(1, 0, 0), Eigen::Vector3d(-1, 0, 0), &test_leg, 10, 5);
  RangeVis<3> vis5(&test_leg, -5.0, 5.0, 10, 5);

  Scene scene;
  scene.AddDrawable(&vis1);
  scene.AddDrawable(&vis2);
  scene.AddDrawable(&vis3);
  scene.AddDrawable(&vis4);
  scene.AddDrawable(&vis5);
  StartWindow(&scene);
}
