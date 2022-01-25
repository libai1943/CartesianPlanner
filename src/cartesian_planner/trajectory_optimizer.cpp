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

#include "cartesian_planner/trajectory_optimizer.h"

#include <bitset>

#include "cartesian_planner/math/math_utils.h"
#include "cartesian_planner/visualization/plot.h"

namespace cartesian_planner {

TrajectoryOptimizer::TrajectoryOptimizer(const CartesianPlannerConfig &config, const Env &env)
  : config_(config), env_(env), nlp_(config) {
  vehicle_ = config_.vehicle;
}

bool TrajectoryOptimizer::OptimizeIteratively(const DiscretizedTrajectory &coarse, const Constraints &constraints,
                                              States &result) {
  States guess;
  for (auto &pt: coarse.data()) {
    guess.x.push_back(pt.x);
    guess.y.push_back(pt.y);
    guess.theta.push_back(pt.theta);
  }
  CalculateInitialGuess(guess);

  int iter = 0;
  double w_penalty = config_.opti_w_penalty0;

  Constraints iterative_constraints = constraints;

  while (iter < config_.opti_iter_max) {
    FormulateCorridorConstraints(guess, iterative_constraints);

    double cur_infeasibility = nlp_.SolveIteratively(w_penalty, iterative_constraints, guess, coarse, guess);
    visualization::Plot(guess.x, guess.y, 0.1, visualization::Color::Red, iter, "Intermediate Trajectory");
    visualization::Trigger();

    ROS_INFO("iter = %d, cur_infeasibility = %f, w_penalty = %f", iter, cur_infeasibility, w_penalty);

    if (cur_infeasibility < config_.opti_varepsilon_tol) {
      result = guess;
      return true;
    } else {
      w_penalty *= config_.opti_alpha;
      iter++;
    }
  }

  return false;
}

void TrajectoryOptimizer::CalculateInitialGuess(States &states) const {
  states.v.resize(config_.nfe, 0.0);
  states.phi.resize(config_.nfe, 0.0);

  double hi = config_.tf / (config_.nfe - 1);
  for (size_t i = 1; i < states.x.size(); i++) {
    double velocity = hypot(states.y[i] - states.y[i - 1], states.x[i] - states.x[i - 1]) / hi;

    states.v[i] = std::min(vehicle_.max_velocity, velocity);
    states.phi[i] = std::min(vehicle_.phi_max, std::max(-vehicle_.phi_max, atan(
      (states.theta[i] - states.theta[i - 1]) * vehicle_.wheel_base / (states.v[i] * hi))));
  }

  states.a.resize(config_.nfe, 0.0);
  states.omega.resize(config_.nfe, 0.0);
  for (size_t i = 1; i < states.x.size(); i++) {
    states.a[i] = std::min(vehicle_.max_acceleration,
                           std::max(vehicle_.min_acceleration, (states.v[i] - states.v[i - 1]) / hi));
    states.omega[i] = std::min(vehicle_.omega_max,
                               std::max(-vehicle_.omega_max, (states.phi[i] - states.phi[i - 1]) / hi));
  }

  states.jerk.resize(config_.nfe, 0.0);
  for (size_t i = 1; i < states.x.size(); i++) {
    states.jerk[i] = std::min(vehicle_.jerk_max, std::max(-vehicle_.jerk_max, (states.a[i] - states.a[i - 1]) / hi));
  }
}

bool TrajectoryOptimizer::FormulateCorridorConstraints(States &states, Constraints &constraints) {
  constraints.front_bound.resize(config_.nfe);
  constraints.rear_bound.resize(config_.nfe);
  states.xf.resize(config_.nfe);
  states.yf.resize(config_.nfe);
  states.xr.resize(config_.nfe);
  states.yr.resize(config_.nfe);

  double hi = config_.tf / (config_.nfe - 1);

  for (size_t i = 0; i < config_.nfe; i++) {
    double time = hi * i;
    std::tie(states.xf[i], states.yf[i], states.xr[i], states.yr[i]) = vehicle_.GetDiscPositions(states.x[i],
                                                                                                 states.y[i],
                                                                                                 states.theta[i]);

    math::AABox2d box;
    if (!GenerateBox(time, states.xf[i], states.yf[i], vehicle_.radius, box)) {
      return false;
    }
    constraints.front_bound[i] = {box.min_x(), box.max_x(), box.min_y(), box.max_y()};

    visualization::PlotPolygon(math::Polygon2d(math::Box2d(box)), 0.02, visualization::Color::Grey, i,
                               "Front Corridor");

    if (!GenerateBox(time, states.xr[i], states.yr[i], vehicle_.radius, box)) {
      return false;
    }
    constraints.rear_bound[i] = {box.min_x(), box.max_x(), box.min_y(), box.max_y()};

    visualization::PlotPolygon(math::Polygon2d(math::Box2d(box)), 0.02, visualization::Color::Blue, i, "Rear Corridor");
  }

  visualization::Trigger();

  return true;
}

bool TrajectoryOptimizer::GenerateBox(double time, double &x, double &y, double radius, AABox2d &result) const {
  double ri = radius;
  AABox2d bound({-ri, -ri}, {ri, ri});
  if (CheckCollision(time, x, y, bound)) {
    // initial condition not satisfied, involute to find feasible box
    int inc = 4;
    double real_x, real_y;

    do {
      int iter = inc / 4;
      uint8_t edge = inc % 4;

      real_x = x;
      real_y = y;
      if (edge == 0) {
        real_x = x - iter * 0.05;
      } else if (edge == 1) {
        real_x = x + iter * 0.05;
      } else if (edge == 2) {
        real_y = y - iter * 0.05;
      } else if (edge == 3) {
        real_y = y + iter * 0.05;
      }

      inc++;
    } while (CheckCollision(time, real_x, real_y, bound) && inc < config_.corridor_max_iter);
    if (inc > config_.corridor_max_iter) {
      return false;
    }

    x = real_x;
    y = real_y;
  }

  int inc = 4;
  std::bitset<4> blocked;
  double incremental[4] = {0.0};
  double step = radius * 0.2;

  do {
    int iter = inc / 4;
    uint8_t edge = inc % 4;
    inc++;

    if (blocked[edge]) continue;

    incremental[edge] = iter * step;

    AABox2d test({-ri - incremental[0], -ri - incremental[2]},
                 {ri + incremental[1], ri + incremental[3]});

    if (CheckCollision(time, x, y, test) || incremental[edge] >= config_.corridor_incremental_limit) {
      incremental[edge] -= step;
      blocked[edge] = true;
    }
  } while (!blocked.all() && inc < config_.corridor_max_iter);
  if (inc > config_.corridor_max_iter) {
    return false;
  }

  result = {{x - incremental[0], y - incremental[2]},
            {x + incremental[1], y + incremental[3]}};
  return true;
}
}