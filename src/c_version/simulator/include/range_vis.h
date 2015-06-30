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
  Eigen::Vector3d point;
  Eigen::Vector3d normal;
  Pose center_pose;

public:
  PlanarRangeVis(Eigen::Vector3d point, Eigen::Vector3d normal, LegController<n_joints>* controller, double samples_per_unit, double max_range) : point(point), normal(normal) {
    double d_r = 1.0/samples_per_unit;//*max_range);
    Eigen::Vector3d center = (point.dot(normal) / normal.squaredNorm()) * normal;
    double yaw = atan2(normal[1], normal[0]);
    double pitch = atan2(normal[2], sqrt(normal[0]*normal[0] + normal[1]*normal[1]));
    center_pose = Pose(center[0], center[1], center[2], yaw, pitch, 0);
    double angles[n_joints];
    for (double x = -max_range; x < max_range; x += d_r) {
      for (double y = -max_range; y < max_range; y += d_r) {
        Eigen::Vector3d goal = Vector4dTo3d(center_pose.FromFrame(Eigen::Vector4d(0.0, x, y, 1.0)));
        if (controller->GetJointCommands(goal, angles) == 0) {
          plane_query.push_back(goal); 
        }
      }
    }
  }

  void Draw(Eigen::Matrix4d to_global) {
    Eigen::Vector4d plane[plane_query.size()];
    for (int i = 0; i < plane_query.size(); i++) {
      plane[i] = to_global * Vector3dTo4d(plane_query[i]);
    }
    Points(plane_query.size(), plane, 1, 0.0, 0.0, 1.0);
    center_pose.Draw(to_global);
    Eigen::Vector3d center_point(center_pose.x, center_pose.y, center_pose.z);
    Eigen::Vector3d vector_end = center_point + normal;
    Point(vector_end, 1.0, 0.0, 0.0);
  }
};


void TestRangeVis();
void TestPlaneVis();

#endif // RANGE_VIS_H_
