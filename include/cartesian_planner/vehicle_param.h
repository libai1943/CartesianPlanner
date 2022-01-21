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

#include <tuple>

#include "math/pose.h"
#include "math/box2d.h"

namespace cartesian_planner {

class VehicleParam {
public:
  /**
   * L_F, front hang length of the ego vehicle (m)
   */
  double front_hang_length = 0.96;

  /**
   * L_W, wheelbase of the ego vehicle (m)
   */
  double wheel_base = 2.80;

  /**
   * L_R, rear hang length of the ego vehicle (m)
   */
  double rear_hang_length = 0.929;

  /**
   * L_B, width of the ego vehicle (m)
   */
  double width = 1.942;

  /**
   * Upper bound of v(t) (m/s)
   */
  double max_velocity = 12.0;

  /**
   * Lower and upper bounds of a(t) (m/s^2)
   */
  double min_acceleration = -5.0, max_acceleration = 5.0;

  /**
   * Upper bound of |jerk(t)| (m/s^3)
   */
  double jerk_max = 10.0;

  /**
   * Upper bound of |\phi(t)| (rad)
   */
  double phi_max = 0.85;

  /**
   * Upper bound of |\omega(t)| (rad/s)
   */
  double omega_max = 1.5;

  double radius;
  double f2x, r2x;

  VehicleParam() {
    double length = (wheel_base + rear_hang_length + front_hang_length);
    radius = hypot(0.25 * length, 0.5 * width);
    r2x = 0.25 * length - rear_hang_length;
    f2x = 0.75 * length - rear_hang_length;
  }

  template<class T>
  std::tuple<T, T, T, T> GetDiscPositions(const T &x, const T &y, const T &theta) const {
    auto xf = x + f2x * cos(theta);
    auto xr = x + r2x * cos(theta);
    auto yf = y + f2x * sin(theta);
    auto yr = y + r2x * sin(theta);
    return std::make_tuple(xf, yf, xr, yr);
  }

  math::Box2d GenerateBox(const math::Pose &pose) const {
    double length = (wheel_base + rear_hang_length + front_hang_length);
    double distance = length / 2 - rear_hang_length;
    return {pose.extend(distance), pose.theta(), length, width};
  }

};
}