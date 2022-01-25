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

#include <array>
#include <casadi/casadi.hpp>

#include "environment.h"
#include "cartesian_planner_config.h"

namespace cartesian_planner {

using namespace casadi;

struct States {
  std::vector<double> x, y, theta, v, phi, a, omega, jerk, xf, yf, xr, yr;
};

struct Constraints {
  double start_x, start_y, start_theta, start_v, start_a, start_phi, start_omega;
  std::vector<std::array<double, 4>> front_bound;
  std::vector<std::array<double, 4>> rear_bound;
};

class TrajectoryNLP {
public:
  explicit TrajectoryNLP(const CartesianPlannerConfig &);

  double SolveIteratively(double w_inf, const Constraints &constraints, const States &guess,
                          const DiscretizedTrajectory &reference, States &result);

private:
  CartesianPlannerConfig config_;
  Dict nlp_config_;
  Function iterative_solver_;
  Function infeasibility_evaluator_;

  void BuildIterativeNLP();
};

}
