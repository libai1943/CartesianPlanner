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

#include "cartesian_planner/vehicle_param.h"

#include "cartesian_planner/math/vec2d.h"
#include "cartesian_planner/math/polygon2d.h"

#include <mutex>
#include <ros/ros.h>
#include <visualization_msgs/MarkerArray.h>

#include "color.h"

namespace cartesian_planner {
namespace visualization {

using math::Vec2d;
using math::Polygon2d;

using Vector = std::vector<double>;

void Init(ros::NodeHandle &node, const std::string &frame, const std::string &topic);

void Plot(const Vector &xs, const Vector &ys, double width = 0.1, Color color = Color(1, 1, 1),
          int id = -1, const std::string &ns = "");

void Plot(const Vector &xs, const Vector &ys, double width = 0.1, const std::vector<Color> &color = {},
          int id = -1, const std::string &ns = "");

void PlotPolygon(const Vector &xs, const Vector &ys, double width = 0.1, Color color = Color::White,
                 int id = -1, const std::string &ns = "");

void PlotPolygon(const Polygon2d &polygon, double width = 0.1, Color color = Color::White,
                 int id = -1, const std::string &ns = "");

void PlotTrajectory(const Vector &xs, const Vector &ys, const Vector &vs, double max_velocity = 10.0,
                    double width = 0.1, const Color &color = Color::Blue,
                    int id = -1, const std::string &ns = "");

void PlotPoints(const Vector &xs, const Vector &ys, double width = 0.1, const Color &color = Color::White,
                int id = -1, const std::string &ns = "");

void Trigger();

void Clear();
}

}