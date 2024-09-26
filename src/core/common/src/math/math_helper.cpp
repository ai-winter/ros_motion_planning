/**
 * *********************************************************
 *
 * @file: math_helper.cpp
 * @brief: math helper function
 * @author: Yang Haodong
 * @date: 2024-9-6
 * @version: 1.0
 *
 * Copyright (c) 2024, Yang Haodong.
 * All rights reserved.
 *
 * --------------------------------------------------------
 *
 * ********************************************************
 */

#include "common/math/math_helper.h"

#include <cmath>
#include <utility>

namespace rmp
{
namespace common
{
namespace math
{
double crossProd(const rmp::common::geometry::Vec2d& start_point, const rmp::common::geometry::Vec2d& end_point_1,
                 const rmp::common::geometry::Vec2d& end_point_2)
{
  return (end_point_1 - start_point).crossProd(end_point_2 - start_point);
}

double innerProd(const rmp::common::geometry::Vec2d& start_point, const rmp::common::geometry::Vec2d& end_point_1,
                 const rmp::common::geometry::Vec2d& end_point_2)
{
  return (end_point_1 - start_point).innerProd(end_point_2 - start_point);
}

double crossProd(const double x0, const double y0, const double x1, const double y1)
{
  return x0 * y1 - x1 * y0;
}

double innerProd(const double x0, const double y0, const double x1, const double y1)
{
  return x0 * x1 + y0 * y1;
}

/**
 * @brief Perform modulus operation on 2π.
 * @param theta    the angle to modulu
 * @return theta_m the angle after modulus operator
 */
double mod2pi(double theta)
{
  return theta - 2.0 * M_PI * floor(theta / M_PI / 2.0);
}

/**
 * @brief Truncate the angle to the interval of -π to π.
 * @param theta    the angle to truncate
 * @return theta_t the truncated angle
 */
double pi2pi(double theta)
{
  while (theta > M_PI)
    theta -= 2.0 * M_PI;
  while (theta < -M_PI)
    theta += 2.0 * M_PI;
  return theta;
}

std::vector<std::pair<double, double>> circleSegmentIntersection(const std::pair<double, double>& p1,
                                                                 const std::pair<double, double>& p2, double r)
{
  std::vector<std::pair<double, double>> i_points;

  double x1 = p1.first;
  double x2 = p2.first;
  double y1 = p1.second;
  double y2 = p2.second;

  double dx = x2 - x1;
  double dy = y2 - y1;
  double dr2 = dx * dx + dy * dy;
  double D = x1 * y2 - x2 * y1;

  // the first element is the point within segment
  double d1 = x1 * x1 + y1 * y1;
  double d2 = x2 * x2 + y2 * y2;
  double dd = d2 - d1;

  double delta = std::sqrt(r * r * dr2 - D * D);

  if (delta >= 0)
  {
    if (delta == 0)
      i_points.emplace_back(D * dy / dr2, -D * dx / dr2);
    else
    {
      i_points.emplace_back((D * dy + std::copysign(1.0, dd) * dx * delta) / dr2,
                            (-D * dx + std::copysign(1.0, dd) * dy * delta) / dr2);
      i_points.emplace_back((D * dy - std::copysign(1.0, dd) * dx * delta) / dr2,
                            (-D * dx - std::copysign(1.0, dd) * dy * delta) / dr2);
    }
  }

  return i_points;
}

}  // namespace math
}  // namespace common
}  // namespace rmp
