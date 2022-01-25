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

#include "cartesian_planner/environment.h"
#include "cartesian_planner/visualization/plot.h"

namespace cartesian_planner {

constexpr double kSampleStep = 0.1;

void Environment::SetReference(const DiscretizedTrajectory &reference) {
  reference_ = reference;

  road_barrier_.clear();

  double start_s = reference_.data().front().s;
  double back_s = reference_.data().back().s;
  int sample_points = int((back_s - start_s) / kSampleStep);
  for (int i = 0; i <= sample_points; i++) {
    double s = start_s + i * kSampleStep;
    auto ref = reference_.EvaluateStation(s);

    road_barrier_.push_back(reference_.GetCartesian(s, ref.left_bound));
    road_barrier_.push_back(reference_.GetCartesian(s, -ref.right_bound));
  }

  std::sort(road_barrier_.begin(), road_barrier_.end(), [](const Vec2d &a, const Vec2d &b) {
    return a.x() < b.x();
  });
}

bool Environment::CheckStaticCollision(const math::Box2d &rect) {
  for (auto &obstacle: obstacles_) {
    if (obstacle.HasOverlap(rect)) {
      return true;
    }
  }

  if (road_barrier_.empty()) {
    return false;
  }

  if (rect.max_x() < road_barrier_.front().x() || rect.min_x() > road_barrier_.back().x()) {
    return false;
  }

  auto comp = [](double val, const Vec2d &a) {
    return val < a.x();
  };

  // binary search
  auto check_start = std::upper_bound(road_barrier_.begin(), road_barrier_.end(), rect.min_x(), comp);
  auto check_end = std::upper_bound(road_barrier_.begin(), road_barrier_.end(), rect.max_x(), comp);

  if (check_start > road_barrier_.begin()) {
    std::advance(check_start, -1);
  }

  for (auto iter = check_start; iter != check_end; iter++) {
    if (rect.IsPointIn(*iter)) {
      return true;
    }
  }

  return false;
}

bool Environment::CheckCollision(double time, const math::Box2d &rect) {
  if (CheckDynamicCollision(time, rect)) {
    return true;
  }

  return CheckStaticCollision(rect);
}

bool Environment::CheckOptimizationCollision(double time, const math::Pose &pose, double collision_buffer) {
  math::AABox2d initial_box({-config_.vehicle.radius - collision_buffer, -config_.vehicle.radius - collision_buffer},
                            {config_.vehicle.radius + collision_buffer, config_.vehicle.radius + collision_buffer});

  double xr, yr, xf, yf;
  std::tie(xr, yr, xf, yf) = config_.vehicle.GetDiscPositions(pose.x(), pose.y(), pose.theta());

  auto f_box = initial_box, r_box = initial_box;
  f_box.Shift({xf, yf});
  r_box.Shift({xr, yr});
  if (CheckStaticCollision(math::Box2d(f_box)) || CheckStaticCollision(math::Box2d(r_box)) ||
      CheckDynamicCollision(time, math::Box2d(f_box)) ||
      CheckDynamicCollision(time, math::Box2d(r_box))) {
    return true;
  }
  return false;
}

bool Environment::CheckDynamicCollision(double time, const math::Box2d &rect) {
  for (auto &obstacle: dynamic_obstacles_) {
    if (obstacle.front().first > time || obstacle.back().first < time) {
      continue;
    }
    auto result = std::upper_bound(obstacle.begin(), obstacle.end(), time,
                                   [](double val, const std::pair<double, math::Polygon2d> &ob) {
                                     return val < ob.first;
                                   });

    if (result->second.HasOverlap(rect)) {
      return true;
    }
  }

  return false;
}

std::unordered_map<int, math::Polygon2d> Environment::QueryDynamicObstacles(double time) {
  std::unordered_map<int, math::Polygon2d> filtered;
  int idx = 0;
  for (auto &obstacle: dynamic_obstacles_) {
    idx++;
    if (obstacle.front().first > time || obstacle.back().first < time) {
      continue;
    }
    auto result = std::upper_bound(obstacle.begin(), obstacle.end(), time,
                                   [](double val, const std::pair<double, math::Polygon2d> &ob) {
                                     return val < ob.first;
                                   });

    filtered.insert({idx, result->second});
  }
  return filtered;
}

void Environment::Visualize() {
  std::vector<double> lb_x, lb_y, ub_x, ub_y;

  for (auto &point: reference_.data()) {
    auto lb = reference_.GetCartesian(point.s, point.left_bound);
    auto ub = reference_.GetCartesian(point.s, -point.right_bound);

    lb_x.push_back(lb.x());
    lb_y.push_back(lb.y());
    ub_x.push_back(ub.x());
    ub_y.push_back(ub.y());
  }

  visualization::Plot(lb_x, lb_y, 0.1, visualization::Color::Grey, 1, "Road Left");
  visualization::Plot(ub_x, ub_y, 0.1, visualization::Color::Grey, 1, "Road Right");

  int idx = 0;
  for (auto &obstacle: obstacles_) {
    visualization::PlotPolygon(obstacle, 0.1, visualization::Color::Magenta, idx++, "Obstacles");
  }

  // plot first frame of dynamic obstacles
  idx = 1;
  for (auto &obstacle: dynamic_obstacles_) {
    auto color = visualization::Color::fromHSV(int((double) idx / dynamic_obstacles_.size() * 320), 1.0, 1.0);
    color.set_alpha(0.5);
    visualization::PlotPolygon(obstacle[0].second, 0.1, color, idx, "Online Obstacle");
    idx++;
  }

  visualization::Trigger();
}

}