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

#pragma once

#include "cartesian_planner_config.h"

#include "dp_planner.h"
#include "trajectory_optimizer.h"

namespace cartesian_planner {

class CartesianPlanner {
public:
  struct StartState {
    double x, y, theta, v, phi, a, omega;
  };

  explicit CartesianPlanner(const CartesianPlannerConfig &config, const Env &env)
    : config_(config), dp_(config, env), opti_(config, env) {}

  bool Plan(const StartState &state, DiscretizedTrajectory &result);


private:
  CartesianPlannerConfig config_;
  DpPlanner dp_;
  TrajectoryOptimizer opti_;

};

}