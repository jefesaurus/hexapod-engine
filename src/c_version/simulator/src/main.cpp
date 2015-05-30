#include "kinematic_chain.h"
#include "viewer.h"
#include "drawing_primitives.h"

#include <stdio.h>


RevoluteJoint coxa(M_PI/2.0, .5, 0.0);
RevoluteJoint femur(0.0, 1.5, 0.0);
RevoluteJoint tibia(0.0, 2.0, 0.0);
RevoluteJoint joints[3] = {RevoluteJoint(M_PI/2.0, .5, 0.0), RevoluteJoint(0.0, 1.5, 0.0), RevoluteJoint(0.0, 2.0, 0.0)};
Leg<3> front_right(joints);

void DrawLeg() {
  Eigen::Vector4d segs[4];
  front_right.AllSegments(segs);
  LineStrip(4, segs, 1.0, 0.0, 0.0);

}

int main() {
  double leg_state[3] = {0.0, M_PI/4.0, -M_PI/2.0};
  front_right.SetState(leg_state);
  StartWindow(&DrawLeg);

  return 0;
}
