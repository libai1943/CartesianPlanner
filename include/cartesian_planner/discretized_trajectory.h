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

#include <utility>
#include <vector>
#include <cassert>

#include "cartesian_planner/math/vec2d.h"

namespace cartesian_planner {

struct TrajectoryPoint {
  double s = 0.0;

  double x = 0.0;
  double y = 0.0;
  double theta = 0.0;
  double kappa = 0.0;
  double velocity = 0.0;

  double left_bound = 0.0;
  double right_bound = 0.0;
};

typedef std::vector<TrajectoryPoint> Trajectory;

using math::Vec2d;

/**
 * Discretized Trajectory
 */
class DiscretizedTrajectory {
public:
  typedef std::vector<TrajectoryPoint> DataType;

  DiscretizedTrajectory() = default;

  DiscretizedTrajectory(const DiscretizedTrajectory &rhs, size_t begin, size_t end = -1);

  explicit DiscretizedTrajectory(std::vector<TrajectoryPoint> points) : data_(std::move(points)) {}

  inline const DataType &data() const { return data_; }

  DataType::const_iterator QueryLowerBoundStationPoint(double station) const;

  DataType::const_iterator QueryNearestPoint(const Vec2d &point, double *out_distance = nullptr) const;

  TrajectoryPoint EvaluateStation(double station) const;

  Vec2d GetProjection(const Vec2d &xy) const;

  Vec2d GetCartesian(double station, double lateral) const;


protected:
  std::vector<TrajectoryPoint> data_;
};

}
