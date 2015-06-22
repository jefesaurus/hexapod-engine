#ifndef RANGE_VIS_H_
#define RANGE_VIS_H_


#include <Eigen/Core>
#include <vector>
#include <stdio.h>

#include "drawing_primitives.h"
#include "leg.h"
#include "utils.h"

template <int n_joints>
class RangeVis : public Drawable {
std::vector<Eigen::Vector3d> reachable_edge;
std::vector<Eigen::Vector3d> unreachable_edge;
public:
  RangeVis(LegController<n_joints>* controller, double min_z, double max_z, double samples_per_unit, double max_range) {
    double d_r = 1.0/(samples_per_unit*max_range);
    int n_samples = 0;

    for (double z = min_z; z < max_z; z += d_r) {
      for (double t = -M_PI/2; t <= M_PI/2; t += d_r) {
        Eigen::Vector3d goal = Eigen::Vector3d(0, 0, z);
        Eigen::Vector3d last_goal = goal;
        double angles[n_joints];
        bool inside = (controller->GetJointCommands(goal, angles) == 0);
        for (double r = 0; r <= max_range; r += d_r) {
          goal = Eigen::Vector3d(cos(t)*r, sin(t)*r, z);
          if (controller->GetJointCommands(goal, angles) == 0) {
            // Hit an edge
            if (!inside) {
              unreachable_edge.push_back(last_goal);
              reachable_edge.push_back(goal);
              inside = true;
            }
          } else {
            // Hit an edge
            if (inside) {
              reachable_edge.push_back(last_goal);
              unreachable_edge.push_back(goal);
              inside = false;
            }
          }
          last_goal = goal;
          n_samples ++;
        }
      }
    }
    printf("Sampled %d points.\n", n_samples);
  }

  void Draw(Eigen::Matrix4d to_global) {
    Eigen::Vector4d unreach[unreachable_edge.size()];
    for (int i = 0; i < unreachable_edge.size(); i++) {
      unreach[i] = to_global * Vector3dTo4d(unreachable_edge[i]);
    }
    Points(unreachable_edge.size(), unreach, 1, 1.0, 0.0, 0.0);

    Eigen::Vector4d reach[reachable_edge.size()];
    for (int i = 0; i < reachable_edge.size(); i++) {
      reach[i] = to_global * Vector3dTo4d(reachable_edge[i]);
    }
    Points(reachable_edge.size(), reach, 1, 0.0, 1.0, 0.0);
  }
};



void TestRangeVis();

#endif // RANGE_VIS_H_
