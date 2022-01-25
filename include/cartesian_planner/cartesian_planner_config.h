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

#include "vehicle_param.h"

namespace cartesian_planner {

struct CartesianPlannerConfig {
  /**
   * Number of finite elements used to discretize an OCP
   */
  int nfe = 320;

  /**
   * Time horizon length (s)
   */
  double tf = 16;

  /**
   * nominal velocity
   */
  double dp_nominal_velocity = 10.0;

  /**
   * cost of obstacles, should be set larger to avoid collision with obstacles
   */
  double dp_w_obstacle = 1000;

  /**
   * lateral cost, the larger the trajectory the closer to the reference line
   */
  double dp_w_lateral = 0.1;

  /**
   * lateral change cost, dl/ds, penalty for lateral change
   */
  double dp_w_lateral_change = 0.5;

  /**
   * lateral change cost, dl/dt, penalty for sudden lateral change
   */
  double dp_w_lateral_velocity_change = 1.0;

  /**
   * longitudinal velocity cost, velocity to the nominal velocity
   */
  double dp_w_longitudinal_velocity_bias = 10.0;

  /**
   * Cost of longitudinal velocity change, ds/dt
   */
  double dp_w_longitudinal_velocity_change = 1.0;

  /**
   * maximum iteration count for corridor expansion
   */
  int corridor_max_iter = 1000;

  /**
   * increment limit for corridor expansion
   */
  double corridor_incremental_limit = 20.0;

  /**
   * Weighting parameter in Eq.(2)
   */
  double opti_w_u = 0.5;

  /**
   * weighting parameter in Eq.(3)
   */
  double opti_w_r_theta = 2.0;

  /**
   * weighting parameter in Eq.(4)
   */
  double opti_w_rw = 5.0;

  /**
   * Maximum iteration number in Alg.1
   */
  int opti_iter_max = 5;

  /**
   * Initial value of weighting parameter w_penalty
   */
  double opti_w_penalty0 = 1e5;

  /**
   * Multiplier to enlarge w_penalty during the iterations
   */
  double opti_alpha = 10;

  /**
   * Violation tolerance w.r.t. the softened nonlinear constraints
   */
  double opti_varepsilon_tol = 1e-4;

  VehicleParam vehicle;
};

}