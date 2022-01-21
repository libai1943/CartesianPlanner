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

namespace cartesian_planner {

class Environment {
public:
  using DynamicObstacle = std::vector<std::pair<double, math::Polygon2d>>;

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

  bool CheckCollision(const math::Box2d &rect);

  bool CheckDynamicCollision(double time, const math::Box2d &rect);

  std::unordered_map<int, math::Polygon2d> QueryDynamicObstacles(double time);

  void Visualize();

private:
  std::vector<DynamicObstacle> dynamic_obstacles_;
  std::vector<math::Polygon2d> obstacles_;
  DiscretizedTrajectory reference_;
  std::vector<math::Vec2d> road_barrier_;
};

using Env = std::shared_ptr<Environment>;
}
