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

#include <string>
#include <algorithm>
#include <sstream>
#include <cstdio>
#include <cmath>

#include <std_msgs/ColorRGBA.h>

using std_msgs::ColorRGBA;

namespace cartesian_planner {
namespace visualization {

class Color {
public:
  Color() = default;

  Color(float r, float g, float b) : r_(r), g_(g), b_(b), a_(1.0) {}

  std::string toPlotColor() const {
    char buf[10];
    snprintf(buf, 10, "#%02x%02x%02x", uint8_t(r_ * 255), uint8_t(g_ * 255), uint8_t(b_ * 255));
    return buf;
  }

  ColorRGBA toColorRGBA() const {
    ColorRGBA color;
    color.r = r_;
    color.g = g_;
    color.b = b_;
    color.a = a_;
    return color;
  }

  static Color Black;
  static Color Grey;
  static Color White;
  static Color Red;
  static Color Green;
  static Color Blue;
  static Color Cyan;
  static Color Yellow;
  static Color Magenta;

  static Color fromRGB(float r, float g, float b) {
    return Color(std::min(r, 1.0f), std::min(g, 1.0f), std::min(b, 1.0f));
  }

  /*
   * H(Hue): 0 - 360 degree (integer)
   * S(Saturation): 0 - 1.00 (double)
   * V(Value): 0 - 1.00 (double)
   */
  static Color fromHSV(int H, double S, double V);

  /*! \brief Convert RGB to HSV color space

  Converts a given set of RGB values `r', `g', `b' into HSV
  coordinates. The input RGB values are in the range [0, 1], and the
  output HSV values are in the ranges h = [0, 360], and s, v = [0,
  1], respectively.

  \param fH Hue component, used as output, range: [0, 360]
  \param fS Hue component, used as output, range: [0, 1]
  \param fV Hue component, used as output, range: [0, 1]

*/
  void toHSV(float &fH, float &fS, float &fV) const;

  inline double r() { return r_; }

  inline double g() { return g_; }

  inline double b() { return b_; }

  inline void set_alpha(double alpha) {
    a_ = alpha;
  }


private:
  double r_, g_, b_, a_;

};

}
}