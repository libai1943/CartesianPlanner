/******************************************************************************
 * Copyright 2017 The Apollo Authors. All Rights Reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *****************************************************************************/

#include "cartesian_planner/math/line_segment2d.h"

#include <algorithm>
#include <utility>
#include <cassert>

#include "cartesian_planner/math/math_utils.h"

namespace cartesian_planner {
namespace math {
namespace {

bool IsWithin(double val, double bound1, double bound2) {
  if (bound1 > bound2) {
    std::swap(bound1, bound2);
  }
  return val >= bound1 - kMathEpsilon && val <= bound2 + kMathEpsilon;
}

}  // namespace

LineSegment2d::LineSegment2d() { unit_direction_ = Vec2d(1, 0); }

LineSegment2d::LineSegment2d(const Vec2d &start, const Vec2d &end)
    : start_(start), end_(end) {
  const double dx = end_.x() - start_.x();
  const double dy = end_.y() - start_.y();
  length_ = hypot(dx, dy);
  unit_direction_ =
      (length_ <= kMathEpsilon ? Vec2d(0, 0)
                               : Vec2d(dx / length_, dy / length_));
  heading_ = unit_direction_.Angle();
}

Vec2d LineSegment2d::rotate(const double angle) {
  Vec2d diff_vec = end_ - start_;
  diff_vec.SelfRotate(angle);
  return start_ + diff_vec;
}

double LineSegment2d::length() const { return length_; }

double LineSegment2d::length_sqr() const { return length_ * length_; }

double LineSegment2d::DistanceTo(const Vec2d &point) const {
  if (length_ <= kMathEpsilon) {
    return point.DistanceTo(start_);
  }
  const double x0 = point.x() - start_.x();
  const double y0 = point.y() - start_.y();
  const double proj = x0 * unit_direction_.x() + y0 * unit_direction_.y();
  if (proj <= 0.0) {
    return hypot(x0, y0);
  }
  if (proj >= length_) {
    return point.DistanceTo(end_);
  }
  return std::abs(x0 * unit_direction_.y() - y0 * unit_direction_.x());
}

double LineSegment2d::DistanceTo(const Vec2d &point,
                                 Vec2d *const nearest_pt) const {
  assert(nearest_pt);
  if (length_ <= kMathEpsilon) {
    *nearest_pt = start_;
    return point.DistanceTo(start_);
  }
  const double x0 = point.x() - start_.x();
  const double y0 = point.y() - start_.y();
  const double proj = x0 * unit_direction_.x() + y0 * unit_direction_.y();
  if (proj < 0.0) {
    *nearest_pt = start_;
    return hypot(x0, y0);
  }
  if (proj > length_) {
    *nearest_pt = end_;
    return point.DistanceTo(end_);
  }
  *nearest_pt = start_ + unit_direction_ * proj;
  return std::abs(x0 * unit_direction_.y() - y0 * unit_direction_.x());
}

double LineSegment2d::DistanceSquareTo(const Vec2d &point) const {
  if (length_ <= kMathEpsilon) {
    return point.DistanceSquareTo(start_);
  }
  const double x0 = point.x() - start_.x();
  const double y0 = point.y() - start_.y();
  const double proj = x0 * unit_direction_.x() + y0 * unit_direction_.y();
  if (proj <= 0.0) {
    return Square(x0) + Square(y0);
  }
  if (proj >= length_) {
    return point.DistanceSquareTo(end_);
  }
  return Square(x0 * unit_direction_.y() - y0 * unit_direction_.x());
}

double LineSegment2d::DistanceToRay(const Vec2d &ray_origin, double ray_direction) {
  auto v1 = ray_origin - start_;
  auto v2 = end_ - start_;
  Vec2d v3(-sin(ray_direction), cos(ray_direction));

  auto dot = v2.InnerProd(v3);
  if (std::abs(dot) < kMathEpsilon)
    return -1.0;

  auto t1 = v2.CrossProd(v1) / dot;
  auto t2 = v1.InnerProd(v3) / dot;

  if (t1 >= 0.0 && (t2 >= 0.0 && t2 <= 1.0))
    return t1;

  return -1.0;
}

double LineSegment2d::DistanceSquareTo(const Vec2d &point,
                                       Vec2d *const nearest_pt) const {
  assert(nearest_pt);
  if (length_ <= kMathEpsilon) {
    *nearest_pt = start_;
    return point.DistanceSquareTo(start_);
  }
  const double x0 = point.x() - start_.x();
  const double y0 = point.y() - start_.y();
  const double proj = x0 * unit_direction_.x() + y0 * unit_direction_.y();
  if (proj <= 0.0) {
    *nearest_pt = start_;
    return Square(x0) + Square(y0);
  }
  if (proj >= length_) {
    *nearest_pt = end_;
    return point.DistanceSquareTo(end_);
  }
  *nearest_pt = start_ + unit_direction_ * proj;
  return Square(x0 * unit_direction_.y() - y0 * unit_direction_.x());
}

bool LineSegment2d::IsPointIn(const Vec2d &point) const {
  if (length_ <= kMathEpsilon) {
    return std::abs(point.x() - start_.x()) <= kMathEpsilon &&
           std::abs(point.y() - start_.y()) <= kMathEpsilon;
  }
  const double prod = CrossProd(point, start_, end_);
  if (std::abs(prod) > kMathEpsilon) {
    return false;
  }
  return IsWithin(point.x(), start_.x(), end_.x()) &&
         IsWithin(point.y(), start_.y(), end_.y());
}

double LineSegment2d::ProjectOntoUnit(const Vec2d &point) const {
  return unit_direction_.InnerProd(point - start_);
}

double LineSegment2d::ProductOntoUnit(const Vec2d &point) const {
  return unit_direction_.CrossProd(point - start_);
}

bool LineSegment2d::HasIntersect(const LineSegment2d &other_segment) const {
  Vec2d point;
  return GetIntersect(other_segment, &point);
}

bool LineSegment2d::GetIntersect(const LineSegment2d &other_segment,
                                 Vec2d *const point) const {
  assert(point);
  if (IsPointIn(other_segment.start())) {
    *point = other_segment.start();
    return true;
  }
  if (IsPointIn(other_segment.end())) {
    *point = other_segment.end();
    return true;
  }
  if (other_segment.IsPointIn(start_)) {
    *point = start_;
    return true;
  }
  if (other_segment.IsPointIn(end_)) {
    *point = end_;
    return true;
  }
  if (length_ <= kMathEpsilon || other_segment.length() <= kMathEpsilon) {
    return false;
  }
  const double cc1 = CrossProd(start_, end_, other_segment.start());
  const double cc2 = CrossProd(start_, end_, other_segment.end());
  if (cc1 * cc2 >= -kMathEpsilon) {
    return false;
  }
  const double cc3 =
      CrossProd(other_segment.start(), other_segment.end(), start_);
  const double cc4 =
      CrossProd(other_segment.start(), other_segment.end(), end_);
  if (cc3 * cc4 >= -kMathEpsilon) {
    return false;
  }
  const double ratio = cc4 / (cc4 - cc3);
  *point = Vec2d(start_.x() * ratio + end_.x() * (1.0 - ratio),
                 start_.y() * ratio + end_.y() * (1.0 - ratio));
  return true;
}

// return distance with perpendicular foot point.
double LineSegment2d::GetPerpendicularFoot(const Vec2d &point,
                                           Vec2d *const foot_point) const {
  assert(foot_point);
  if (length_ <= kMathEpsilon) {
    *foot_point = start_;
    return point.DistanceTo(start_);
  }
  const double x0 = point.x() - start_.x();
  const double y0 = point.y() - start_.y();
  const double proj = x0 * unit_direction_.x() + y0 * unit_direction_.y();
  *foot_point = start_ + unit_direction_ * proj;
  return std::abs(x0 * unit_direction_.y() - y0 * unit_direction_.x());
}

std::vector<Vec2d> LineSegment2d::SamplePoints(double step) {
  int num = int(length_ / step);
  if(length_ / step - num > 1e-5) {
    // warning
  }
  std::vector<Vec2d> points(num);

  for (int i = 0; i < num; i++) {
    points[i] = start_ + unit_direction_ * step * i;
  }

  return points;
}

}  // namespace math
}  // namespace common
