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

#include <memory>
#include <unordered_map>

#include "math/polygon2d.h"
#include "discretized_trajectory.h"
#include "vehicle_param.h"
#include "cartesian_planner_config.h"

namespace cartesian_planner {

class Environment {
public:
  using DynamicObstacle = std::vector<std::pair<double, math::Polygon2d>>;

  explicit Environment(const CartesianPlannerConfig &config) : config_(config) {}

  std::vector<math::Polygon2d> &obstacles() {
    return obstacles_;
  }

  std::vector<DynamicObstacle> &dynamic_obstacles() {
    return dynamic_obstacles_;
  }

  const DiscretizedTrajectory &reference() const {
    return reference_;
  }

  void SetReference(const DiscretizedTrajectory &reference);

  bool CheckCollision(double time, const math::Box2d &rect);

  bool CheckOptimizationCollision(double time, const math::Pose &pose, double collision_buffer = 0.0);

  std::unordered_map<int, math::Polygon2d> QueryDynamicObstacles(double time);

  void Visualize();

private:
  CartesianPlannerConfig config_;
  std::vector<DynamicObstacle> dynamic_obstacles_;
  std::vector<math::Polygon2d> obstacles_;
  DiscretizedTrajectory reference_;
  std::vector<math::Vec2d> road_barrier_;

  bool CheckStaticCollision(const math::Box2d &rect);

  bool CheckDynamicCollision(double time, const math::Box2d &rect);
};

using Env = std::shared_ptr<Environment>;
}
