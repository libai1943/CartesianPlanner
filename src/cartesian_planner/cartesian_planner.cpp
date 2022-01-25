/***********************************************************************************
 *  C++ Source Codes for "Autonomous Driving on Curvy Roads without Reliance on
 *  Frenet Frame: A Cartesian-based Trajectory Planning Method".
 ***********************************************************************************
 *  Copyright (C) 2022 Bai Li
 *  Users are suggested to cite the following article when they use the source codes.
 *  Bai Li et al., "Autonomous Driving on Curvy Roads without Reliance on
 *  Frenet Frame: A Cartesian-based Trajectory Planning Method",
 *  IEEE Transactions on Intelligent Transportation Systems, 2022.
 ***********************************************************************************/

#include "cartesian_planner/cartesian_planner.h"
#include "cartesian_planner/visualization/plot.h"

namespace cartesian_planner {

bool CartesianPlanner::Plan(const StartState &state, DiscretizedTrajectory &result) {
  DiscretizedTrajectory coarse_trajectory;
  if(!dp_.Plan(state.x, state.y, state.theta, coarse_trajectory)) {
    ROS_ERROR("DP failed");
    return false;
  }

  Constraints opti_constraints;
  opti_constraints.start_x = state.x; opti_constraints.start_y = state.y; opti_constraints.start_theta = state.theta;
  opti_constraints.start_v = state.v; opti_constraints.start_phi = state.phi; opti_constraints.start_a = state.a;
  opti_constraints.start_omega = state.omega;

  std::vector<double> coarse_x, coarse_y;
  for(auto &pt: coarse_trajectory.data()) {
    coarse_x.push_back(pt.x); coarse_y.push_back(pt.y);
  }

  visualization::Plot(coarse_x, coarse_y, 0.1, visualization::Color::Cyan, 1, "Coarse Trajectory");
  visualization::PlotPoints(coarse_x, coarse_y, 0.3, visualization::Color::Cyan, 2, "Coarse Trajectory");
  visualization::Trigger();

  States optimized;
  if(!opti_.OptimizeIteratively(coarse_trajectory, opti_constraints, optimized)) {
    ROS_ERROR("Optimization failed");
    return false;
  }

  std::vector<double> opti_x, opti_y, opti_v;
  Trajectory result_data;
  double incremental_s = 0.0;
  for(int i = 0; i < config_.nfe; i++) {
    TrajectoryPoint tp;
    incremental_s += i > 0 ? hypot(optimized.x[i] - optimized.x[i-1], optimized.y[i] - optimized.y[i-1]) : 0.0;
    tp.s = incremental_s;

    tp.x = optimized.x[i];
    tp.y = optimized.y[i];
    tp.theta = optimized.theta[i];
    tp.velocity = optimized.v[i];
    tp.kappa = tan(optimized.phi[i]) / config_.vehicle.wheel_base;

    opti_x.push_back(tp.x);
    opti_y.push_back(tp.y);
    opti_v.push_back(tp.velocity);

    result_data.push_back(tp);
  }

  visualization::PlotTrajectory(opti_x, opti_y, opti_v, config_.vehicle.max_velocity, 0.1, visualization::Color::Green, 1, "Optimized Trajectory");
  visualization::Trigger();

  result = DiscretizedTrajectory(result_data);
  return true;
}
}