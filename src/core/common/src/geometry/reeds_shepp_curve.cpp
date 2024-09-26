/**
 * *********************************************************
 *
 * @file: reeds_shepp_curve.h
 * @brief: Reeds shepp curve generation
 * @author: Yang Haodong
 * @date: 2023-12-26
 * @version: 1.0
 *
 * Copyright (c) 2024, Yang Haodong.
 * All rights reserved.
 *
 * --------------------------------------------------------
 *
 * ********************************************************
 */
#include <climits>
#include <cassert>
#include <iostream>

#include "common/math/math_helper.h"
#include "common/geometry/reeds_shepp_curve.h"

namespace rmp
{
namespace common
{
namespace geometry
{
/**
 * @brief Construct a new Reeds-Shepp Path object
 * @param lengths  the length of segments
 * @param ctypes   the motion patterns of segments
 */
ReedsSheppCurve::RSPath::RSPath(std::vector<double> lengths, std::vector<int> ctypes)
{
  lengths_ = lengths;
  ctypes_ = ctypes;
}

/**
 * @brief Destroy the Reeds-Shepp Path object
 */
ReedsSheppCurve::RSPath::~RSPath()
{
}

/**
 * @brief Calculate the length of the generated trajectory
 * @return length   the length of the generated trajectory
 */
double ReedsSheppCurve::RSPath::len()
{
  double path_length = 0.0;
  for (const auto& l : lengths_)
    path_length += fabs(l);

  return path_length;
}

/**
 * @brief Determine whether the generated trajectory is valid
 * @return flag   true is valid else invalid
 */
bool ReedsSheppCurve::RSPath::valid()
{
  return ((lengths_.size() > 0) && (lengths_.size() == ctypes_.size()));
}

/**
 * @brief Calculate the number of segments for generating trajectories
 * @return number   the number of segments for generating trajectories
 */
size_t ReedsSheppCurve::RSPath::size()
{
  assert(valid());
  return lengths_.size();
}

/**
 * @brief Obtain segment i with its motion pattern
 * @param i       index
 * @param length  the length of segment i
 * @param ctype   the motion of segment i
 */
void ReedsSheppCurve::RSPath::get(int i, double& length, int& ctype)
{
  assert(valid());
  length = lengths_[i];
  ctype = ctypes_[i];
}

/**
 * @brief Construct a new ReedsShepp generation object
 * @param step        Simulation or interpolation size (default: 0.1)
 * @param max_curv    The maximum curvature of the curve (default: 0.25)
 */
ReedsSheppCurve::ReedsSheppCurve(double step, double max_curv) : Curve(step), max_curv_(max_curv)
{
}
ReedsSheppCurve::ReedsSheppCurve() : Curve(0.1), max_curv_(0.25)
{
}

/**
 * @brief Destroy the ReedsShepp generation object
 */
ReedsSheppCurve::~ReedsSheppCurve()
{
}

/**
 * @brief Return the polar coordinates (r, theta) of the point (x, y), i.e. rcos(theta) = x; rsin(theta) = y
 */
void ReedsSheppCurve::R(double x, double y, double& r, double& theta)
{
  r = hypot(x, y);
  theta = atan2(y, x);
}

/**
 * @brief Truncate the angle to the interval of -π to π.
 */
double ReedsSheppCurve::M(double theta)
{
  return rmp::common::math::pi2pi(theta);
}

/**
 * @brief Straight-Left-Straight generation mode.
 */
bool ReedsSheppCurve::SLS(double x, double y, double phi, RSLength& length)
{
  phi = M(phi);

  if ((y > 0.0) && (phi > 0.0) && (phi < M_PI * 0.99))
  {
    double xd = -y / tan(phi) + x;
    double t = xd - tan(phi / 2.0);
    double u = phi;
    double v = sqrt((x - xd) * (x - xd) + y * y) - tan(phi / 2.0);
    length = { t, u, v };
    return true;
  }
  else if ((y < 0.0) && (phi > 0.0) && (phi < M_PI * 0.99))
  {
    double xd = -y / tan(phi) + x;
    double t = xd - tan(phi / 2.0);
    double u = phi;
    double v = -sqrt((x - xd) * (x - xd) + y * y) - tan(phi / 2.0);
    length = { t, u, v };
    return true;
  }

  length = { REEDS_SHEPP_NONE, REEDS_SHEPP_NONE, REEDS_SHEPP_NONE };
  return false;
}

/**
 * @brief Left-Right-Left generation mode. (L+R-L-)
 */
bool ReedsSheppCurve::LRL(double x, double y, double phi, RSLength& length)
{
  double r, theta;
  R(x - sin(phi), y - 1.0 + cos(phi), r, theta);

  if (r <= 4.0)
  {
    double u = -2.0 * asin(0.25 * r);
    double t = M(theta + 0.5 * u + M_PI);
    double v = M(phi - t + u);

    if ((t >= 0.0) && (u <= 0.0))
    {
      length = { t, u, v };
      return true;
    }
  }

  length = { REEDS_SHEPP_NONE, REEDS_SHEPP_NONE, REEDS_SHEPP_NONE };
  return false;
}

/**
 * @brief Left-Straight-Left generation mode. (L+S+L+)
 */
bool ReedsSheppCurve::LSL(double x, double y, double phi, RSLength& length)
{
  double u, t;
  R(x - sin(phi), y - 1.0 + cos(phi), u, t);

  if (t >= 0.0)
  {
    double v = M(phi - t);
    if (v >= 0.0)
    {
      length = { t, u, v };
      return true;
    }
  }

  length = { REEDS_SHEPP_NONE, REEDS_SHEPP_NONE, REEDS_SHEPP_NONE };
  return false;
}

/**
 * @brief Left-Straight-Right generation mode. (L+S+R+)
 */
bool ReedsSheppCurve::LSR(double x, double y, double phi, RSLength& length)
{
  double r, theta;
  R(x + sin(phi), y - 1.0 - cos(phi), r, theta);

  r = r * r;

  if (r >= 4.0)
  {
    double u = sqrt(r - 4.0);
    double t = M(theta + atan2(2.0, u));
    double v = M(t - phi);

    if ((t >= 0.0) && (v >= 0.0))
    {
      length = { t, u, v };
      return true;
    }
  }

  length = { REEDS_SHEPP_NONE, REEDS_SHEPP_NONE, REEDS_SHEPP_NONE };
  return false;
}

/**
 * @brief Left-Right(beta)-Left(beta)-Right generation mode. (L+R+L-R-)
 */
bool ReedsSheppCurve::LRLRn(double x, double y, double phi, RSLength& length)
{
  double xi = x + sin(phi);
  double eta = y - 1.0 - cos(phi);
  double rho = 0.25 * (2.0 + sqrt(xi * xi + eta * eta));

  if (rho <= 1.0)
  {
    double u = acos(rho);
    double t, v;
    _calTauOmega(u, -u, xi, eta, phi, t, v);
    if ((t >= 0.0) && (v <= 0.0))
    {
      length = { t, u, v };
      return true;
    }
  }
  length = { REEDS_SHEPP_NONE, REEDS_SHEPP_NONE, REEDS_SHEPP_NONE };
  return false;
}

/**
 * @brief Left-Right(beta)-Left(beta)-Right generation mode. (L+R-L-R+)
 */
bool ReedsSheppCurve::LRLRp(double x, double y, double phi, RSLength& length)
{
  double xi = x + sin(phi);
  double eta = y - 1.0 - cos(phi);
  double rho = (20.0 - xi * xi - eta * eta) / 16.0;

  if ((rho >= 0.0) && (rho <= 1.0))
  {
    double u = -acos(rho);
    if (u >= -0.5 * M_PI)
    {
      double t, v;
      _calTauOmega(u, u, xi, eta, phi, t, v);
      if ((t >= 0.0) && (v >= 0.0))
      {
        length = { t, u, v };
        return true;
      }
    }
  }

  length = { REEDS_SHEPP_NONE, REEDS_SHEPP_NONE, REEDS_SHEPP_NONE };
  return false;
}

/**
 * @brief Left-Right(pi/2)-Straight-Right generation mode. (L+R-S-R-)
 */
bool ReedsSheppCurve::LRSR(double x, double y, double phi, RSLength& length)
{
  double xi = x + sin(phi);
  double eta = y - 1.0 - cos(phi);
  double rho, theta;
  R(-eta, xi, rho, theta);

  if (rho >= 2.0)
  {
    double t = theta;
    double u = 2.0 - rho;
    double v = M(t + 0.5 * M_PI - phi);
    if ((t >= 0.0) && (u <= 0.0) && (v <= 0.0))
    {
      length = { t, u, v };
      return true;
    }
  }

  length = { REEDS_SHEPP_NONE, REEDS_SHEPP_NONE, REEDS_SHEPP_NONE };
  return false;
}

/**
 * @brief Left-Right(pi/2)-Straight-Left generation mode. (L+R-S-L-)
 */
bool ReedsSheppCurve::LRSL(double x, double y, double phi, RSLength& length)
{
  double xi = x - sin(phi);
  double eta = y - 1.0 + cos(phi);
  double rho, theta;
  R(xi, eta, rho, theta);

  if (rho >= 2.0)
  {
    double r = sqrt(rho * rho - 4.0);
    double u = 2.0 - r;
    double t = M(theta + atan2(r, -2.0));
    double v = M(phi - 0.5 * M_PI - t);
    if ((t >= 0.0) && (u <= 0.0) && (v <= 0.0))
    {
      length = { t, u, v };
      return true;
    }
  }

  length = { REEDS_SHEPP_NONE, REEDS_SHEPP_NONE, REEDS_SHEPP_NONE };
  return false;
}

/**
 * @brief Left-Right(pi/2)-Straight-Left(pi/2)-Right generation mode. (L+R-S-L-R+)
 */
bool ReedsSheppCurve::LRSLR(double x, double y, double phi, RSLength& length)
{
  double xi = x + sin(phi);
  double eta = y - 1.0 - cos(phi);
  double r, theta;
  R(xi, eta, r, theta);

  if (r >= 2.0)
  {
    double u = 4.0 - sqrt(r * r - 4.0);
    if (u <= 0.0)
    {
      double t = M(atan2((4.0 - u) * xi - 2.0 * eta, -2.0 * xi + (u - 4.0) * eta));
      double v = M(t - phi);

      if ((t >= 0.0) && (v >= 0.0))
      {
        length = { t, u, v };
        return true;
      }
    }
  }

  length = { REEDS_SHEPP_NONE, REEDS_SHEPP_NONE, REEDS_SHEPP_NONE };
  return false;
}

/**
 * @brief # 2 Straight-Circle-Straight generation mode(using reflect).
 * @param x/y Goal position
 * @param phi Goal pose
 * @return Reeds-Shepp Paths
 */
std::vector<ReedsSheppCurve::RSPath> ReedsSheppCurve::SCS(double x, double y, double phi)
{
  std::vector<RSPath> paths;
  RSLength length;
  double t, u, v;
  bool flag;

  flag = SLS(x, y, phi, length);
  if (flag)
  {
    std::tie(t, u, v) = length;

    paths.push_back({ { t, u, v }, { REEDS_SHEPP_S, REEDS_SHEPP_L, REEDS_SHEPP_S } });
  }

  flag = SLS(x, -y, -phi, length);
  if (flag)
  {
    std::tie(t, u, v) = length;

    paths.push_back({ { t, u, v }, { REEDS_SHEPP_S, REEDS_SHEPP_R, REEDS_SHEPP_S } });
  }

  return paths;
}

/**
 * @brief # 8 Circle-Circle-Circle generation mode(using reflect, timeflip and backwards).
 * @param x/y Goal position
 * @param phi Goal pose
 * @return Reeds-Shepp Paths
 */
std::vector<ReedsSheppCurve::RSPath> ReedsSheppCurve::CCC(double x, double y, double phi)
{
  std::vector<RSPath> paths;
  RSLength length;
  double t, u, v;
  bool flag;

  // L+R-L-
  flag = LRL(x, y, phi, length);
  if (flag)
  {
    std::tie(t, u, v) = length;

    paths.push_back({ { t, u, v }, { REEDS_SHEPP_L, REEDS_SHEPP_R, REEDS_SHEPP_L } });
  }

  // timefilp: L-R+L+
  flag = LRL(-x, y, -phi, length);
  if (flag)
  {
    std::tie(t, u, v) = length;
    paths.push_back({ { -t, -u, -v }, { REEDS_SHEPP_L, REEDS_SHEPP_R, REEDS_SHEPP_L } });
  }

  // reflect: R+L-R-
  flag = LRL(x, -y, -phi, length);
  if (flag)
  {
    std::tie(t, u, v) = length;

    paths.push_back({ { t, u, v }, { REEDS_SHEPP_R, REEDS_SHEPP_L, REEDS_SHEPP_R } });
  }

  // timeflip + reflect: R-L+R+
  flag = LRL(-x, -y, phi, length);
  if (flag)
  {
    std::tie(t, u, v) = length;

    paths.push_back({ { -t, -u, -v }, { REEDS_SHEPP_R, REEDS_SHEPP_L, REEDS_SHEPP_R } });
  }

  // backwards
  double xb = x * cos(phi) + y * sin(phi);
  double yb = x * sin(phi) - y * cos(phi);

  // backwards: L-R-L+
  flag = LRL(xb, yb, phi, length);
  if (flag)
  {
    std::tie(t, u, v) = length;
    paths.push_back({ { v, u, t }, { REEDS_SHEPP_L, REEDS_SHEPP_R, REEDS_SHEPP_L } });
  }

  // backwards + timefilp: L+R+L-
  flag = LRL(-xb, yb, -phi, length);
  if (flag)
  {
    std::tie(t, u, v) = length;
    paths.push_back({ { -v, -u, -t }, { REEDS_SHEPP_L, REEDS_SHEPP_R, REEDS_SHEPP_L } });
  }

  // backwards + reflect: R-L-R+
  flag = LRL(xb, -yb, -phi, length);
  if (flag)
  {
    std::tie(t, u, v) = length;
    paths.push_back({ { v, u, t }, { REEDS_SHEPP_R, REEDS_SHEPP_L, REEDS_SHEPP_R } });
  }

  // backwards + timeflip + reflect: R+L+R-
  flag = LRL(-xb, -yb, phi, length);
  if (flag)
  {
    std::tie(t, u, v) = length;
    paths.push_back({ { -v, -u, -t }, { REEDS_SHEPP_R, REEDS_SHEPP_L, REEDS_SHEPP_R } });
  }

  return paths;
}

/**
 * @brief # 8 Circle-Straight-Circle generation mode(using reflect, timeflip and backwards).
 * @param x/y Goal position
 * @param phi Goal pose
 * @return Reeds-Shepp Paths
 */
std::vector<ReedsSheppCurve::RSPath> ReedsSheppCurve::CSC(double x, double y, double phi)
{
  std::vector<RSPath> paths;
  RSLength length;
  double t, u, v;
  bool flag;

  // L+S+L+
  flag = LSL(x, y, phi, length);
  if (flag)
  {
    std::tie(t, u, v) = length;

    paths.push_back({ { t, u, v }, { REEDS_SHEPP_L, REEDS_SHEPP_S, REEDS_SHEPP_L } });
  }

  // timefilp: L-S-L-
  flag = LSL(-x, y, -phi, length);
  if (flag)
  {
    std::tie(t, u, v) = length;

    paths.push_back({ { -t, -u, -v }, { REEDS_SHEPP_L, REEDS_SHEPP_S, REEDS_SHEPP_L } });
  }

  // reflect: R+S+R+
  flag = LSL(x, -y, -phi, length);
  if (flag)
  {
    std::tie(t, u, v) = length;

    paths.push_back({ { t, u, v }, { REEDS_SHEPP_R, REEDS_SHEPP_S, REEDS_SHEPP_R } });
  }

  // timeflip + reflect: R-S-R-
  flag = LSL(-x, -y, phi, length);
  if (flag)
  {
    std::tie(t, u, v) = length;
    paths.push_back({ { -t, -u, -v }, { REEDS_SHEPP_R, REEDS_SHEPP_S, REEDS_SHEPP_R } });
  }

  // L+S+R+
  flag = LSR(x, y, phi, length);
  if (flag)
  {
    std::tie(t, u, v) = length;

    paths.push_back({ { t, u, v }, { REEDS_SHEPP_L, REEDS_SHEPP_S, REEDS_SHEPP_R } });
  }

  // timefilp: L-S-R-
  flag = LSR(-x, y, -phi, length);
  if (flag)
  {
    std::tie(t, u, v) = length;
    paths.push_back({ { -t, -u, -v }, { REEDS_SHEPP_L, REEDS_SHEPP_S, REEDS_SHEPP_R } });
  }

  // reflect: R+S+L+
  flag = LSR(x, -y, -phi, length);
  if (flag)
  {
    std::tie(t, u, v) = length;

    paths.push_back({ { t, u, v }, { REEDS_SHEPP_R, REEDS_SHEPP_S, REEDS_SHEPP_L } });
  }

  // timeflip + reflect: R+S+l-
  flag = LSR(-x, -y, phi, length);
  if (flag)
  {
    std::tie(t, u, v) = length;
    paths.push_back({ { -t, -u, -v }, { REEDS_SHEPP_R, REEDS_SHEPP_S, REEDS_SHEPP_L } });
  }

  return paths;
}

/**
 * @brief # 8 Circle-Circle(beta)-Circle(beta)-Circle generation mode (using reflect, timeflip and backwards).
 * @param x/y Goal position
 * @param phi Goal pose
 * @return Reeds-Shepp Paths
 */
std::vector<ReedsSheppCurve::RSPath> ReedsSheppCurve::CCCC(double x, double y, double phi)
{
  std::vector<RSPath> paths;
  RSLength length;
  double t, u, v;
  bool flag;

  // L+R+L-R-
  flag = LRLRn(x, y, phi, length);
  if (flag)
  {
    std::tie(t, u, v) = length;
    paths.push_back({ { t, u, -u, v }, { REEDS_SHEPP_L, REEDS_SHEPP_R, REEDS_SHEPP_L, REEDS_SHEPP_R } });
  }

  // timefilp: L-R-L+R+
  flag = LRLRn(-x, y, -phi, length);
  if (flag)
  {
    std::tie(t, u, v) = length;
    paths.push_back({ { -t, -u, u, -v }, { REEDS_SHEPP_L, REEDS_SHEPP_R, REEDS_SHEPP_L, REEDS_SHEPP_R } });
  }

  // reflect: R+L+R-L-
  flag = LRLRn(x, -y, -phi, length);
  if (flag)
  {
    std::tie(t, u, v) = length;
    paths.push_back({ { t, u, -u, v }, { REEDS_SHEPP_R, REEDS_SHEPP_L, REEDS_SHEPP_R, REEDS_SHEPP_L } });
  }

  // timeflip + reflect: R-L-R+L+
  flag = LRLRn(-x, -y, phi, length);
  if (flag)
  {
    std::tie(t, u, v) = length;
    paths.push_back({ { -t, -u, u, -v }, { REEDS_SHEPP_R, REEDS_SHEPP_L, REEDS_SHEPP_R, REEDS_SHEPP_L } });
  }

  // L+R-L-R+
  flag = LRLRp(x, y, phi, length);
  if (flag)
  {
    std::tie(t, u, v) = length;
    paths.push_back({ { t, u, u, v }, { REEDS_SHEPP_L, REEDS_SHEPP_R, REEDS_SHEPP_L, REEDS_SHEPP_R } });
  }

  // timefilp: L-R+L+R-
  flag = LRLRp(-x, y, -phi, length);
  if (flag)
  {
    std::tie(t, u, v) = length;
    paths.push_back({ { -t, -u, -u, -v }, { REEDS_SHEPP_L, REEDS_SHEPP_R, REEDS_SHEPP_L, REEDS_SHEPP_R } });
  }

  // reflect: R+L-R-L+
  flag = LRLRp(x, -y, -phi, length);
  if (flag)
  {
    std::tie(t, u, v) = length;
    paths.push_back({ { t, u, u, v }, { REEDS_SHEPP_R, REEDS_SHEPP_L, REEDS_SHEPP_R, REEDS_SHEPP_L } });
  }

  // timeflip + reflect: R-L+R+L-
  flag = LRLRp(-x, -y, phi, length);
  if (flag)
  {
    std::tie(t, u, v) = length;
    paths.push_back({ { -t, -u, -u, -v }, { REEDS_SHEPP_R, REEDS_SHEPP_L, REEDS_SHEPP_R, REEDS_SHEPP_L } });
  }

  return paths;
}

/**
 * @brief # 16 Circle-Circle(pi/2)-Straight-Circle and Circle-Straight-Circle(pi/2)-Circle
 * generation mode (using reflect, timeflip and backwards).
 * @param x/y Goal position
 * @param phi Goal pose
 * @return Reeds-Shepp Paths
 */
std::vector<ReedsSheppCurve::RSPath> ReedsSheppCurve::CCSC(double x, double y, double phi)
{
  std::vector<RSPath> paths;
  RSLength length;
  double t, u, v;
  bool flag;

  // L+R-(pi/2)S-L-
  flag = LRSL(x, y, phi, length);
  if (flag)
  {
    std::tie(t, u, v) = length;
    paths.push_back({ { t, -0.5 * M_PI, u, v }, { REEDS_SHEPP_L, REEDS_SHEPP_R, REEDS_SHEPP_S, REEDS_SHEPP_L } });
  }

  // timefilp: L-R+(pi/2)S+L+
  flag = LRSL(-x, y, -phi, length);
  if (flag)
  {
    std::tie(t, u, v) = length;
    paths.push_back({ { -t, 0.5 * M_PI, -u, -v }, { REEDS_SHEPP_L, REEDS_SHEPP_R, REEDS_SHEPP_S, REEDS_SHEPP_L } });
  }

  // reflect: R+L-(pi/2)S-R-
  flag = LRSL(x, -y, -phi, length);
  if (flag)
  {
    std::tie(t, u, v) = length;
    paths.push_back({ { t, -0.5 * M_PI, u, v }, { REEDS_SHEPP_R, REEDS_SHEPP_L, REEDS_SHEPP_S, REEDS_SHEPP_R } });
  }

  // timeflip + reflect: R-L+(pi/2)S+R+
  flag = LRSL(-x, -y, phi, length);
  if (flag)
  {
    std::tie(t, u, v) = length;
    paths.push_back({ { -t, 0.5 * M_PI, -u, -v }, { REEDS_SHEPP_R, REEDS_SHEPP_L, REEDS_SHEPP_S, REEDS_SHEPP_R } });
  }

  // L+R-(pi/2)S-R-
  flag = LRSR(x, y, phi, length);
  if (flag)
  {
    std::tie(t, u, v) = length;
    paths.push_back({ { t, -0.5 * M_PI, u, v }, { REEDS_SHEPP_L, REEDS_SHEPP_R, REEDS_SHEPP_S, REEDS_SHEPP_R } });
  }

  // timefilp: L-R+(pi/2)S+R+
  flag = LRSR(-x, y, -phi, length);
  if (flag)
  {
    std::tie(t, u, v) = length;
    paths.push_back({ { -t, 0.5 * M_PI, -u, -v }, { REEDS_SHEPP_L, REEDS_SHEPP_R, REEDS_SHEPP_S, REEDS_SHEPP_R } });
  }

  // reflect: R+L-(pi/2)S-L-
  flag = LRSR(x, -y, -phi, length);
  if (flag)
  {
    std::tie(t, u, v) = length;
    paths.push_back({ { t, -0.5 * M_PI, u, v }, { REEDS_SHEPP_R, REEDS_SHEPP_L, REEDS_SHEPP_S, REEDS_SHEPP_L } });
  }

  // timeflip + reflect: R-L+(pi/2)S+L+
  flag = LRSR(-x, -y, phi, length);
  if (flag)
  {
    std::tie(t, u, v) = length;
    paths.push_back({ { -t, 0.5 * M_PI, -u, -v }, { REEDS_SHEPP_R, REEDS_SHEPP_L, REEDS_SHEPP_S, REEDS_SHEPP_L } });
  }

  // backwards
  double xb = x * cos(phi) + y * sin(phi);
  double yb = x * sin(phi) - y * cos(phi);

  // backwards: L-S-R-(pi/2)L+
  flag = LRSL(xb, yb, phi, length);
  if (flag)
  {
    std::tie(t, u, v) = length;
    paths.push_back({ { v, u, -0.5 * M_PI, t }, { REEDS_SHEPP_L, REEDS_SHEPP_S, REEDS_SHEPP_R, REEDS_SHEPP_L } });
  }

  // backwards + timefilp: L+S+R+(pi/2)L-
  flag = LRSL(-xb, yb, -phi, length);
  if (flag)
  {
    std::tie(t, u, v) = length;
    paths.push_back({ { -v, -u, 0.5 * M_PI, -t }, { REEDS_SHEPP_L, REEDS_SHEPP_S, REEDS_SHEPP_R, REEDS_SHEPP_L } });
  }

  // backwards + reflect: R-S-L-(pi/2)R+
  flag = LRSL(xb, -yb, -phi, length);
  if (flag)
  {
    std::tie(t, u, v) = length;
    paths.push_back({ { v, u, -0.5 * M_PI, t }, { REEDS_SHEPP_R, REEDS_SHEPP_S, REEDS_SHEPP_L, REEDS_SHEPP_R } });
  }

  // backwards + timefilp + reflect: R+S+L+(pi/2)R-
  flag = LRSL(-xb, -yb, phi, length);
  if (flag)
  {
    std::tie(t, u, v) = length;
    paths.push_back({ { -v, -u, 0.5 * M_PI, -t }, { REEDS_SHEPP_R, REEDS_SHEPP_S, REEDS_SHEPP_L, REEDS_SHEPP_R } });
  }

  // backwards: R-S-R-(pi/2)L+
  flag = LRSR(xb, yb, phi, length);
  if (flag)
  {
    std::tie(t, u, v) = length;
    paths.push_back({ { v, u, -0.5 * M_PI, t }, { REEDS_SHEPP_R, REEDS_SHEPP_S, REEDS_SHEPP_R, REEDS_SHEPP_L } });
  }

  // backwards + timefilp: R+S+R+(pi/2)L-
  flag = LRSR(-xb, yb, -phi, length);
  if (flag)
  {
    std::tie(t, u, v) = length;
    paths.push_back({ { -v, -u, 0.5 * M_PI, -t }, { REEDS_SHEPP_R, REEDS_SHEPP_S, REEDS_SHEPP_R, REEDS_SHEPP_L } });
  }

  // backwards + reflect: L-S-L-(pi/2)R+
  flag = LRSR(xb, -yb, -phi, length);
  if (flag)
  {
    std::tie(t, u, v) = length;
    paths.push_back({ { v, u, -0.5 * M_PI, t }, { REEDS_SHEPP_L, REEDS_SHEPP_S, REEDS_SHEPP_L, REEDS_SHEPP_R } });
  }

  // backwards + timefilp + reflect: L+S+L+(pi/2)R-
  flag = LRSR(-xb, -yb, phi, length);
  if (flag)
  {
    std::tie(t, u, v) = length;
    paths.push_back({ { -v, -u, 0.5 * M_PI, -t }, { REEDS_SHEPP_L, REEDS_SHEPP_S, REEDS_SHEPP_L, REEDS_SHEPP_R } });
  }

  return paths;
}

/**
 * @brief # 4 Circle-Circle(pi/2)-Straight--Circle(pi/2)-Circle generation mode (using reflect, timeflip and
 * backwards).
 * @param x/y Goal position
 * @param phi Goal pose
 * @return Reeds-Shepp Paths
 */
std::vector<ReedsSheppCurve::RSPath> ReedsSheppCurve::CCSCC(double x, double y, double phi)
{
  std::vector<RSPath> paths;
  RSLength length;
  double t, u, v;
  bool flag;

  // L+R-(pi/2)S-L-(pi/2)R+
  flag = LRSLR(x, y, phi, length);
  if (flag)
  {
    std::tie(t, u, v) = length;
    paths.push_back({ { t, -0.5 * M_PI, u, -0.5 * M_PI, v },
                      { REEDS_SHEPP_L, REEDS_SHEPP_R, REEDS_SHEPP_S, REEDS_SHEPP_L, REEDS_SHEPP_R } });
  }

  // timefilp: L-R+(pi/2)S+L+(pi/2)R-
  flag = LRSLR(-x, y, -phi, length);
  if (flag)
  {
    std::tie(t, u, v) = length;
    paths.push_back({ { -t, 0.5 * M_PI, -u, 0.5 * M_PI, -v },
                      { REEDS_SHEPP_L, REEDS_SHEPP_R, REEDS_SHEPP_S, REEDS_SHEPP_L, REEDS_SHEPP_R } });
  }

  // reflect: R+L-(pi/2)S-R-(pi/2)L+
  flag = LRSLR(x, -y, -phi, length);
  if (flag)
  {
    std::tie(t, u, v) = length;
    paths.push_back({ { t, -0.5 * M_PI, u, -0.5 * M_PI, v },
                      { REEDS_SHEPP_R, REEDS_SHEPP_L, REEDS_SHEPP_S, REEDS_SHEPP_R, REEDS_SHEPP_L } });
  }

  // timefilp + reflect: R-L+(pi/2)S+R+(pi/2)L-
  flag = LRSLR(-x, -y, phi, length);
  if (flag)
  {
    std::tie(t, u, v) = length;
    paths.push_back({ { -t, 0.5 * M_PI, -u, 0.5 * M_PI, -v },
                      { REEDS_SHEPP_R, REEDS_SHEPP_L, REEDS_SHEPP_S, REEDS_SHEPP_R, REEDS_SHEPP_L } });
  }

  return paths;
}

/**
 * @brief Planning path interpolation.
 * @param mode      motion, i.e., REEDS_SHEPP_L, REEDS_SHEPP_S, REEDS_SHEPP_R
 * @param length    Single step motion path length
 * @param init_pose Initial pose (x, y, yaw)
 * @return new_pose	New pose (new_x, new_y, new_yaw) after moving
 */
Point3d ReedsSheppCurve::interpolate(int mode, double length, Point3d init_pose)
{
  double new_x, new_y, new_yaw;
  double x = init_pose.x(), y = init_pose.y(), yaw = init_pose.theta();

  if (mode == REEDS_SHEPP_S)
  {
    new_x = x + length / max_curv_ * cos(yaw);
    new_y = y + length / max_curv_ * sin(yaw);
    new_yaw = yaw;
  }
  else if (mode == REEDS_SHEPP_L)
  {
    new_x = x + (sin(yaw + length) - sin(yaw)) / max_curv_;
    new_y = y - (cos(yaw + length) - cos(yaw)) / max_curv_;
    new_yaw = yaw + length;
  }
  else if (mode == REEDS_SHEPP_R)
  {
    new_x = x - (sin(yaw - length) - sin(yaw)) / max_curv_;
    new_y = y + (cos(yaw - length) - cos(yaw)) / max_curv_;
    new_yaw = yaw - length;
  }
  else
    std::cerr << "Error mode" << std::endl;

  return { new_x, new_y, new_yaw };
}

/**
 * @brief Generate the path.
 * @param start Initial pose (x, y, yaw)
 * @param goal  Target pose (x, y, yaw)
 * @return path The smoothed trajectory points
 */
Points2d ReedsSheppCurve::generation(Point3d start, Point3d goal)
{
  Points2d path;
  double sx = start.x(), sy = start.y(), syaw = start.theta();
  double gx = goal.x(), gy = goal.y(), gyaw = goal.theta();

  // coordinate transformation
  double dx = gx - sx;
  double dy = gy - sy;
  double dyaw = gyaw - syaw;
  double x = (cos(syaw) * dx + sin(syaw) * dy) * max_curv_;
  double y = (-sin(syaw) * dx + cos(syaw) * dy) * max_curv_;

  // select the best motion
  RSPath best_path({ std::numeric_limits<double>::max() }, { REEDS_SHEPP_NONE });

  _update(SCS(x, y, dyaw), best_path);
  _update(CCC(x, y, dyaw), best_path);
  _update(CSC(x, y, dyaw), best_path);
  _update(CCCC(x, y, dyaw), best_path);
  _update(CCSC(x, y, dyaw), best_path);
  _update(CCSCC(x, y, dyaw), best_path);

  if (best_path.len() == std::numeric_limits<double>::max())
    return path;

  // interpolation
  int points_num = int(best_path.len() / step_) + 6;

  std::vector<double> path_x(points_num);
  std::vector<double> path_y(points_num);
  std::vector<double> path_yaw(points_num);

  int i = 0;
  for (size_t j = 0; j < best_path.size(); j++)
  {
    int m;
    double seg_length;
    best_path.get(j, seg_length, m);

    // path increment
    double d_l = seg_length > 0.0 ? step_ : -step_;
    double x = path_x[i];
    double y = path_y[i];
    double yaw = path_yaw[i];

    // current path length
    double l = d_l;
    while (fabs(l) <= fabs(seg_length))
    {
      i += 1;
      auto pt = interpolate(m, l, { x, y, yaw });
      path_x[i] = pt.x(), path_y[i] = pt.y(), path_yaw[i] = pt.theta();
      l += d_l;
    }
    i += 1;
    auto pt = interpolate(m, seg_length, { x, y, yaw });
    path_x[i] = pt.x(), path_y[i] = pt.y(), path_yaw[i] = pt.theta();
  }

  // remove unused data
  while ((path_x.size() >= 1) && (path_x.back() == 0.0))
  {
    path_x.pop_back();
    path_y.pop_back();
    path_yaw.pop_back();
  }

  // coordinate transformation
  for (size_t i = 0; i < path_x.size(); i++)
    path.push_back({ cos(-syaw) * path_x[i] + sin(-syaw) * path_y[i] + sx,
                     -sin(-syaw) * path_x[i] + cos(-syaw) * path_y[i] + sy });

  return path;
}

/**
 * @brief Running trajectory generation
 * @param points path points <x, y>
 * @param path generated trajectory
 * @return true if generate successfully, else failed
 */
bool ReedsSheppCurve::run(const Points2d points, Points2d& path)
{
  if (points.size() < 4)
    return false;
  else
  {
    Points3d poses;
    poses.emplace_back(points.begin()->x(), points.begin()->y(), 0);
    for (size_t i = 1; i < points.size() - 1; i++)
    {
      double theta1 = std::atan2(points[i].y() - points[i - 1].y(), points[i].x() - points[i - 1].x());
      double theta2 = std::atan2(points[i + 1].y() - points[i].y(), points[i + 1].x() - points[i].x());
      poses.emplace_back(points[i].x(), points[i].y(), (theta1 + theta2) / 2);
    }
    poses.emplace_back(points.back().x(), points.back().y(), 0);

    return run(poses, path);
  }
}

/**
 * @brief Running trajectory generation
 * @param points path points <x, y, theta>
 * @param path generated trajectory
 * @return true if generate successfully, else failed
 */
bool ReedsSheppCurve::run(const Points3d points, Points2d& path)
{
  if (points.size() < 4)
    return false;
  else
  {
    path.clear();
    for (size_t i = 0; i < points.size() - 1; i++)
    {
      Points2d path_i = generation(points[i], points[i + 1]);
      if (!path_i.empty())
        path.insert(path.end(), path_i.begin(), path_i.end());
    }

    return !path.empty();
  }
}

/**
 * @brief Configure the maximum curvature.
 * @param max_curv  the maximum curvature
 */
void ReedsSheppCurve::setMaxCurv(double max_curv)
{
  assert(max_curv > 0);
  max_curv_ = max_curv;
}

void ReedsSheppCurve::_calTauOmega(double u, double v, double xi, double eta, double phi, double& tau, double& omega)
{
  double delta = M(u - v);
  double A = sin(u) - sin(delta);
  double B = cos(u) - cos(delta) - 1.0;

  double t1 = atan2(eta * A - xi * B, xi * A + eta * B);
  double t2 = 2.0 * (cos(delta) - cos(v) - cos(u)) + 3.0;

  tau = t2 < 0 ? M(t1 + M_PI) : M(t1);
  omega = M(tau - u + v - phi);
}

/**
 * @brief Update the best motion mode.
 * @param cur_paths   current generated Reeds-Shepp paths
 * @param best_path   The best generated Reeds-Shepp path so far
 */
void ReedsSheppCurve::_update(std::vector<RSPath> cur_paths, RSPath& best_path)
{
  for (auto p : cur_paths)
  {
    if (p.len() < best_path.len())
      best_path = p;
  }
}
}  // namespace geometry
}  // namespace common
}  // namespace rmp
