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
        bool inside = (controller->GetJointCommands(goal, angles) >= 0);
        for (double r = 0; r <= max_range; r += d_r) {
          goal = Eigen::Vector3d(cos(t)*r, sin(t)*r, z);
          if (controller->GetJointCommands(goal, angles) >= 0) {
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

template <int n_joints>
class PlanarRangeVis : public Drawable {
  std::vector<Eigen::Vector3d> plane_query;
  std::vector<double> plane_query_scores;
  Eigen::Vector3d point;
  Eigen::Vector3d normal;
  Pose center_pose;

  ColoredGrid grid;
  static constexpr double norm_max = 1.5;
  static constexpr double norm_min = 0;

public:
  PlanarRangeVis(Eigen::Vector3d point, Eigen::Vector3d normal, LegController<n_joints>* controller, double samples_per_unit, double max_range) : point(point), normal(normal), grid(samples_per_unit*2*max_range, samples_per_unit*2*max_range) {
    Eigen::Vector3d center = (point.dot(normal) / normal.squaredNorm()) * normal;
    double yaw = atan2(normal[1], normal[0]);
    double pitch = atan2(normal[2], sqrt(normal[0]*normal[0] + normal[1]*normal[1]));
    center_pose = Pose(center[0], center[1], center[2], yaw, pitch, 0);

    int n_samples = samples_per_unit*2*max_range;
    double delta = 1.0/samples_per_unit;
    double grid_x, grid_y;

    Eigen::Vector3d goal;
    double angles[n_joints];
    double r, g, b;
    for (int x = 0; x < n_samples; x++) {
      grid_x = -max_range + x*delta;
      for (int y = 0; y < n_samples; y++) {
        grid_y = -max_range + y*delta;
        goal = Vector4dTo3d(center_pose.FromFrame(Eigen::Vector4d(0.0, grid_x, grid_y, 1.0)));
        double score = controller->GetJointCommands(goal, angles);
        if (score < norm_min) {
          grid.SetVal(x, y, goal[0], goal[1], goal[2], 1.0, 0.0, 1.0);
          continue;
        } else if (score > norm_max) {
          score = norm_max;
        }
        GetHeatMapColor((score - norm_min)/(norm_max - norm_min), &r, &g, &b);
        grid.SetVal(x, y, goal[0], goal[1], goal[2], r, g, b);
      }
    }
  }

  void Draw(Eigen::Matrix4d to_global) {
    grid.Draw(to_global);
    center_pose.Draw(to_global);
  }
};


void TestRangeVis();
void TestPlaneVis();
void TestAllVis();

#endif // RANGE_VIS_H_
