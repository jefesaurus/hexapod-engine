// The contents of this file are in the public domain. See LICENSE_FOR_EXAMPLE_PROGRAMS.txt
/*

    This is an example illustrating the use the general purpose non-linear 
    optimization routines from the dlib C++ Library.

    The library provides implementations of the conjugate gradient,  BFGS,
    L-BFGS, and BOBYQA optimization algorithms.  These algorithms allow you to
    find the minimum of a function of many input variables.  This example walks
    though a few of the ways you might put these routines to use.

*/
#include <dlib/optimization.h>
#include <iostream>
#include <Eigen/Core>


#include "leg.h"
#include "test_parts.h"
#include "drawing_primitives.h"
#include "viewer.h"


using namespace dlib;

// ----------------------------------------------------------------------------------------

// In dlib, the general purpose solvers optimize functions that take a column
// vector as input and return a double.  So here we make a typedef for a
// variable length column vector of doubles.  This is the type we will use to
// represent the input to our objective functions which we will be minimizing.
typedef matrix<double,0,1> column_vector;

class test_function
{
    /*
        This object is an example of what is known as a "function object" in C++.
        It is simply an object with an overloaded operator().  This means it can 
        be used in a way that is similar to a normal C function.  The interesting
        thing about this sort of function is that it can have state.  
        
        In this example, our test_function object contains a column_vector 
        as its state and it computes the mean squared error between this 
        stored column_vector and the arguments to its operator() function.

        This is a very simple function, however, in general you could compute
        any function you wanted here.  An example of a typical use would be 
        to find the parameters of some regression function that minimized 
        the mean squared error on a set of data.  In this case the arguments
        to the operator() function would be the parameters of your regression
        function.  You would loop over all your data samples and compute the output 
        of the regression function for each data sample given the parameters and 
        return a measure of the total error.   The dlib optimization functions 
        could then be used to find the parameters that minimized the error.
    */
public:

    test_function (
        const column_vector& input
    )
    {
        target = input;
    }

    double operator() ( const column_vector& arg) const
    {
        // return the mean squared error between the target vector and the input vector
        return mean(squared(target-arg));
    }

private:
    column_vector target;
};

void TestMinimization() {
        column_vector starting_point(4);
        column_vector target(4);
        target = 3, 1, 5, 7;
        starting_point = -4,5,99,3;
        find_min_bobyqa(test_function(target), 
                        starting_point, 
                        9,    // number of interpolation points
                        uniform_matrix<double>(4,1, -1e100),  // lower bound constraint
                        uniform_matrix<double>(4,1, 1e100),   // upper bound constraint
                        10,    // initial trust region radius
                        1e-6,  // stopping trust region radius
                        100    // max number of objective function evaluations
        );

        std::cout << "test_function solution:\n" << starting_point << std::endl;
}


template <int n_joints>
class PlanarRangeVis2 : public Drawable {
  std::vector<Eigen::Vector3d> plane_query;
  std::vector<double> plane_query_scores;
  LegController<n_joints>* controller;
  Eigen::Vector3d point;
  Eigen::Vector3d normal;
  Pose center_pose;

public:
  PlanarRangeVis2(Pose plane_pose, LegController<n_joints>* controller, double samples_per_unit, double max_range) : controller(controller), point(point), normal(normal) {
    double d_r = 1.0/samples_per_unit;
    Eigen::Vector3d center = (point.dot(normal) / normal.squaredNorm()) * normal;
    /*
    double yaw = atan2(normal[1], normal[0]);
    double pitch = atan2(normal[2], sqrt(normal[0]*normal[0] + normal[1]*normal[1]));
    center_pose = Pose(center[0], center[1], center[2], yaw, pitch, 0);
    */
    center_pose = plane_pose;
    center_pose.x = center[0];
    center_pose.y = center[1];
    center_pose.z = center[2];
    double angles[n_joints];
    for (double x = -max_range; x < max_range; x += d_r) {
      for (double y = -max_range; y < max_range; y += d_r) {
        Eigen::Vector3d goal = Vector4dTo3d(center_pose.FromFrame(Eigen::Vector4d(0.0, x, y, 1.0)));
        plane_query_scores.push_back(controller->GetJointCommands(goal, angles));
        plane_query.push_back(goal); 
      }
    }
  }

  double operator() (const column_vector& arg) const {
    double angles[n_joints];
    Eigen::Vector3d goal = Vector4dTo3d(center_pose.FromFrame(Eigen::Vector4d(0.0, arg(0), arg(1), 1.0)));
    double score = controller->GetJointCommands(goal, angles);
    return score;
  }

  void Draw(Eigen::Matrix4d to_global) {
    Eigen::Vector4d plane[plane_query.size()];
    double r[plane_query.size()];
    double g[plane_query.size()];
    double b[plane_query.size()];
    static const double norm_max = 1.5;
    static const double norm_min = 0.0;
    for (int i = 0; i < plane_query.size(); i++) {
      plane[i] = to_global * Vector3dTo4d(plane_query[i]);
      double score = plane_query_scores[i];
      if (score < norm_min) {
        r[i] = 1.0;
        g[i] = 0.0;
        b[i] = 1.0;
        continue;
      } else if (score > norm_max) {
        score = norm_max;
      }
      GetHeatMapColor((score - norm_min)/(norm_max - norm_min), &r[i], &g[i], &b[i]);
    }
    Points(plane_query.size(), plane, 1, r, g, b);
  }
};

void TestMaximization() {
  column_vector starting_point(2);
  starting_point = 0.3, 0.0;

  Leg<3> leg = GetTestLeg();
  LegController<3> test_leg = GetTestLegController(&leg);
  Eigen::Vector3d normal(0, 0, 1.0);
  Pose plane_pose(0, 0, -1.5, atan2(normal[1], normal[0]), atan2(normal[2], sqrt(normal[0]*normal[0] + normal[1]*normal[1])), 0);

  PlanarRangeVis2<3> vis(plane_pose, &test_leg, 10, 5);

  double best_score = find_max_bobyqa(vis, 
                                      starting_point, 
                                      5,    // number of interpolation points
                                      uniform_matrix<double>(2,1, -1e100),  // lower bound constraint
                                      uniform_matrix<double>(2,1, 1e100),   // upper bound constraint
                                      10,    // initial trust region radius
                                      1e-10,  // stopping trust region radius
                                      1000000    // max number of objective function evaluations
  );

  printf("Best Score: %f, at (%f, %f)\n", best_score, starting_point(0), starting_point(1));
  Scene scene;
  scene.AddDrawable(&vis);
  Eigen::Vector4d global_best = plane_pose.FromFrame(Eigen::Vector4d(0.0, starting_point(0), starting_point(1), 1.0));
  Pose best_spot(global_best[0], global_best[1], global_best[2], 0, 0, 0); 
  scene.AddDrawable(&best_spot);
  StartWindow(&scene);
}
