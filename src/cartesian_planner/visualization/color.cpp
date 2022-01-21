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

#include "cartesian_planner/visualization/color.h"
#include <cmath>

namespace cartesian_planner {
namespace visualization {

Color Color::Black(0.0, 0.0, 0.0);
Color Color::Grey(0.7, 0.7, 0.7);
Color Color::White(1.0, 1.0, 1.0);
Color Color::Red(1.0, 0.0, 0.0);
Color Color::Green(0.0, 1.0, 0.0);
Color Color::Blue(0.0, 0.0, 1.0);
Color Color::Cyan(0.0, 1.0, 1.0);
Color Color::Yellow(1.0, 1.0, 0.0);
Color Color::Magenta(1.0, 0.0, 1.0);

Color Color::fromHSV(int H, double S, double V) {
  double C = S * V;
  double X = C * (1 - fabs(fmod(H / 60.0, 2) - 1));
  double m = V - C;
  double Rs, Gs, Bs;

  if (H >= 0 && H < 60) {
    Rs = C;
    Gs = X;
    Bs = 0;
  } else if (H >= 60 && H < 120) {
    Rs = X;
    Gs = C;
    Bs = 0;
  } else if (H >= 120 && H < 180) {
    Rs = 0;
    Gs = C;
    Bs = X;
  } else if (H >= 180 && H < 240) {
    Rs = 0;
    Gs = X;
    Bs = C;
  } else if (H >= 240 && H < 300) {
    Rs = X;
    Gs = 0;
    Bs = C;
  } else {
    Rs = C;
    Gs = 0;
    Bs = X;
  }

  return Color(Rs + m, Gs + m, Bs + m);
}

void Color::toHSV(float &fH, float &fS, float &fV) const {
  float fCMax = std::max(std::max(r_, g_), b_);
  float fCMin = std::min(std::min(r_, g_), b_);
  float fDelta = fCMax - fCMin;

  if (fDelta > 0) {
    if (fCMax == r_) {
      fH = 60 * (std::fmod(((g_ - b_) / fDelta), 6));
    } else if (fCMax == g_) {
      fH = 60 * (((b_ - r_) / fDelta) + 2);
    } else if (fCMax == b_) {
      fH = 60 * (((r_ - g_) / fDelta) + 4);
    }

    if (fCMax > 0) {
      fS = fDelta / fCMax;
    } else {
      fS = 0;
    }

    fV = fCMax;
  } else {
    fH = 0;
    fS = 0;
    fV = fCMax;
  }

  if (fH < 0) {
    fH = 360 + fH;
  }
}
}
}
