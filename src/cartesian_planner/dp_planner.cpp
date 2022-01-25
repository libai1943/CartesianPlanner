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

#include "cartesian_planner/dp_planner.h"

#include <bitset>
#include <utility>
#include <sstream>

#include "cartesian_planner/math/math_utils.h"
#include "cartesian_planner/math/polygon2d.h"
#include "cartesian_planner/visualization/plot.h"

namespace cartesian_planner {

constexpr double kMathEpsilon = 1e-3;

DpPlanner::DpPlanner(const CartesianPlannerConfig &config, const Env &env)
  : env_(env), config_(config), nseg_(config.nfe / NT), unit_time_(config.tf / NT) {
  time_ = math::LinSpaced<NT>(unit_time_, config.tf);
  station_ = math::LinSpaced<NS>(0, unit_time_ * config_.vehicle.max_velocity);
  lateral_ = math::LinSpaced<NL - 1>(0, 1);
  safe_margin_ = config_.vehicle.width / 2 * 1.5;
}

double DpPlanner::GetCollisionCost(StateIndex parent_ind, StateIndex cur_ind) {
  double parent_s = state_.start_s, grandparent_s = state_.start_s;
  double last_l = state_.start_l, last_s = state_.start_s;
  if (parent_ind.t >= 0) {
    auto &cell = state_space_[parent_ind.t][parent_ind.s][parent_ind.l];
    parent_s = cell.current_s;

    if (parent_ind.t > 0) {
      auto &parent_cell = state_space_[parent_ind.t - 1][cell.parent_s_ind][cell.parent_l_ind];
      grandparent_s = parent_cell.current_s;
    }

    auto prev_path = InterpolateLinearly(grandparent_s, cell.parent_l_ind, parent_ind.s, parent_ind.l);
    last_l = prev_path.back().y();
    last_s = prev_path.back().x();
  }

  auto path = InterpolateLinearly(parent_s, parent_ind.l, cur_ind.s, cur_ind.l);

  for (int i = 0; i < path.size(); i++) {
    auto &pt = path[i];
    double dl = pt.y() - last_l;
    double ds = std::max(pt.x() - last_s, kMathEpsilon);
    last_l = pt.y();
    last_s = pt.x();

    auto cart = env_->reference().GetCartesian(pt.x(), pt.y());
    auto ref = env_->reference().EvaluateStation(pt.x());
    double lb = std::min(0.0, -ref.right_bound + safe_margin_);
    double ub = std::max(0.0, ref.left_bound - safe_margin_);
    if (pt.y() < lb - kMathEpsilon || pt.y() > ub + kMathEpsilon) {
      return config_.dp_w_obstacle;
    }

    double heading = ref.theta + atan((dl / ds) / (1 - ref.kappa * pt.y()));
    math::Pose pose(cart.x(), cart.y(), heading);

    double parent_time = parent_ind.t < 0 ? 0.0 : time_[parent_ind.t];
    double time = parent_time + i * (unit_time_ / nseg_);

    if(env_->CheckOptimizationCollision(time, pose)) {
      return config_.dp_w_obstacle;
    }
  }

  return 0.0;
}

std::pair<double, double> DpPlanner::GetCost(StateIndex parent_ind, StateIndex cur_ind) {
  double parent_s = state_.start_s, grandparent_s = state_.start_s;
  double parent_l = state_.start_l, grandparent_l = state_.start_l;

  if (parent_ind.t >= 0) {
    auto &cell = state_space_[parent_ind.t][parent_ind.s][parent_ind.l];
    int grandparent_s_ind = cell.parent_s_ind;
    int grandparent_l_ind = cell.parent_l_ind;
    parent_s = cell.current_s;
    parent_l = GetLateralOffset(parent_s, parent_ind.l);

    if (parent_ind.t >= 1) {
      grandparent_s = state_space_[parent_ind.t - 1][grandparent_s_ind][grandparent_l_ind].current_s;
      grandparent_l = GetLateralOffset(grandparent_s, grandparent_l_ind);
    }
  }

  double cur_s = parent_s + station_[cur_ind.s];
  double cur_l = GetLateralOffset(cur_s, cur_ind.l);

  double ds1 = cur_s - parent_s;
  double dl1 = cur_l - parent_l;

  double ds0 = parent_s - grandparent_s;
  double dl0 = parent_l - grandparent_l;

  double cost_obstacle = GetCollisionCost(parent_ind, cur_ind);
  if (cost_obstacle >= config_.dp_w_obstacle) {
    return std::make_pair(cur_s, config_.dp_w_obstacle);
  }

  double cost_lateral = fabs(cur_l);
  double cost_lateral_change = fabs(parent_l - cur_l) / (station_[cur_ind.s] + kMathEpsilon);
  double cost_lateral_change_t = fabs(dl1 - dl0) / unit_time_;
  double cost_longitudinal_velocity = fabs(ds1 / unit_time_ - config_.dp_nominal_velocity);
  double cost_longitudinal_velocity_change = fabs((ds1 - ds0) / unit_time_);

  double delta_cost = (
    config_.dp_w_lateral * cost_lateral +
    config_.dp_w_lateral_change * cost_lateral_change +
    config_.dp_w_lateral_velocity_change * cost_lateral_change_t +
    config_.dp_w_longitudinal_velocity_bias * cost_longitudinal_velocity +
    config_.dp_w_longitudinal_velocity_change * cost_longitudinal_velocity_change);

  return std::make_pair(cur_s, delta_cost);
}

bool DpPlanner::Plan(double start_x, double start_y, double start_theta, DiscretizedTrajectory &result) {
  auto sl = env_->reference().GetProjection({start_x, start_y});
  state_.start_s = sl.x();
  state_.start_l = sl.y();
  state_.start_theta = start_theta;

  // reset state space
  for (int i = 0; i < NT; i++) {
    for (int j = 0; j < NS; j++) {
      for (int k = 0; k < NL; k++) {
        state_space_[i][j][k] = StateCell();
      }
    }
  }

  // evaluate first layer
  for (int i = 0; i < NS; i++) {
    for (int j = 0; j < NL; j++) {
      auto tup = GetCost(StateIndex(-1, -1, -1), StateIndex(0, i, j));
      state_space_[0][i][j].current_s = tup.first;
      state_space_[0][i][j].cost = tup.second;
    }
  }

  // dynamic programming
  for (int i = 0; i < NT - 1; i++) {
    for (int j = 0; j < NS; j++) {
      for (int k = 0; k < NL; k++) {
        StateIndex parent_ind(i, j, k);

        for (int m = 0; m < NS; m++) {
          for (int n = 0; n < NL; n++) {
            StateIndex current_ind(i + 1, m, n);
            auto tup = GetCost(parent_ind, current_ind);
            double delta_cost = tup.second;
            double cur_s = tup.first;

            double cur_cost = state_space_[i][j][k].cost + delta_cost;
            if (cur_cost < state_space_[i + 1][m][n].cost) {
              state_space_[i + 1][m][n] = StateCell(cur_cost, cur_s, j, k);
            }
          }
        }
      }
    }
  }

  // find the least cost in final layer
  double min_cost = Inf;
  int min_s_ind = 0, min_l_ind = 0;
  for (int i = 0; i < NS; i++) {
    for (int j = 0; j < NL; j++) {
      double cost = state_space_[NT - 1][i][j].cost;
      if (cost < min_cost) {
        min_s_ind = i;
        min_l_ind = j;
        min_cost = cost;
      }
    }
  }

  std::vector<std::pair<StateIndex, StateCell>> waypoints(NT);

  // trace back layers to find optimum trajectory
  for (int i = NT - 1; i >= 0; i--) {
    auto &cell = state_space_[i][min_s_ind][min_l_ind];
    waypoints[i] = std::make_pair(StateIndex(i, min_s_ind, min_l_ind), cell);
    min_s_ind = cell.parent_s_ind;
    min_l_ind = cell.parent_l_ind;
  }

  ROS_INFO("s[0] = %f", state_.start_s);
  int t_ind = 1;
  for(auto &wp: waypoints) {
    ROS_INFO("s[%d] = %f", t_ind++, wp.second.current_s);
  }

  // interpolation
  Trajectory data;
  data.resize(config_.nfe);
  double last_l = state_.start_l, last_s = state_.start_s;

  for (int i = 0; i < NT; i++) {
    double parent_s = i > 0 ? waypoints[i - 1].second.current_s : state_.start_s;
    auto segment = InterpolateLinearly(parent_s, waypoints[i].second.parent_l_ind, waypoints[i].first.s,
                                       waypoints[i].first.l);

    for (int j = 0; j < nseg_; j++) {
      auto dl = segment[j].y() - last_l;
      auto ds = std::max(segment[j].x() - last_s, kMathEpsilon);
      last_l = segment[j].y();
      last_s = segment[j].x();

      auto xy = env_->reference().GetCartesian(segment[j].x(), segment[j].y());
      auto tp = env_->reference().EvaluateStation(segment[j].x());
      int n = i * nseg_ + j;
      data[n].s = segment[j].x();
      data[n].x = xy.x();
      data[n].y = xy.y();
      data[n].theta = tp.theta + atan((dl / ds) / (1 - tp.kappa * segment[j].y()));
    }
  }
  data[0].theta = state_.start_theta;

  result = DiscretizedTrajectory(data);

  return min_cost < config_.dp_w_obstacle;
}

std::vector<Vec2d> DpPlanner::InterpolateLinearly(double parent_s, int parent_l_ind, int cur_s_ind, int cur_l_ind) {
  std::vector<Vec2d> result(nseg_);

  double p_l = state_.start_l;
  double p_s = state_.start_s;
  if (parent_l_ind >= 0) {
    p_s = parent_s;
    p_l = GetLateralOffset(p_s, parent_l_ind);
  }
  double cur_s = p_s + station_[cur_s_ind];
  double cur_l = GetLateralOffset(cur_s, cur_l_ind);

  double s_step = station_[cur_s_ind] / nseg_;
  double l_step = (cur_l - p_l) / nseg_;

  for (int i = 0; i < nseg_; i++) {
    result[i].set_x(p_s + i * s_step);
    result[i].set_y(p_l + i * l_step);
  }

  return result;
}

}
