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
#include <algorithm>
#include <cmath>
#include <utility>

#include "common/math/math_helper.h"

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

/**
 * @brief Formula for intersection of a line with a circle centered at the origin
 * @note  https://mathworld.wolfram.com/Circle-LineIntersection.html
 * @param p1/p2     the two point in the segment
 * @param r         the radius of circle centered at the origin
 * @return points   the intersection points of a line and the circle
 */
std::vector<rmp::common::geometry::Vec2d> circleSegmentIntersection(const rmp::common::geometry::Vec2d& p1,
                                                                    const rmp::common::geometry::Vec2d& p2, double r)
{
  std::vector<rmp::common::geometry::Vec2d> i_points;

  double x1 = p1.x();
  double x2 = p2.x();
  double y1 = p1.y();
  double y2 = p2.y();

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

/**
 * @brief Center of an arc between three points
 * @param pt_prev Starting point of the arc
 * @param pt Mid point of the arc
 * @param pt_next Last point of the arc
 * @param is_cusp True if pt is a cusp point
 * @result position of the center or Vector2(inf, inf) for straight lines and 180 deg turns
 */
double arcCenter(const rmp::common::geometry::Vec2d& pt_prev, const rmp::common::geometry::Vec2d& pt,
                 const rmp::common::geometry::Vec2d& pt_next, bool is_cusp, rmp::common::geometry::Vec2d* center)
{
  auto d1 = pt - pt_prev;
  auto d2 = is_cusp ? pt - pt_next : pt_next - pt;
  double det = d1.x() * d2.y() - d1.y() * d2.x();
  if (std::fabs(det) < kMathEpsilon)
  {  // straight line
    if (center != nullptr)
    {
      center->setX(std::numeric_limits<double>::infinity());
      center->setY(std::numeric_limits<double>::infinity());
    }
    return 0.0;
  }

  auto mid1 = (pt_prev + pt) * 0.5;
  auto mid2 = is_cusp ? (2 * pt + d2) * 0.5 : (pt_next + pt) * 0.5;
  rmp::common::geometry::Vec2d n1(-d1.y(), d1.x());
  rmp::common::geometry::Vec2d n2(-d2.y(), d2.x());
  double det1 = (mid1.x() + n1.x()) * mid1.y() - (mid1.y() + n1.y()) * mid1.x();
  double det2 = (mid2.x() + n2.x()) * mid2.y() - (mid2.y() + n2.y()) * mid2.x();
  rmp::common::geometry::Vec2d arc_center((det1 * n2.x() - det2 * n1.x()) / det, (det1 * n2.y() - det2 * n1.y()) / det);
  if (center != nullptr)
  {
    center->setX(arc_center.x());
    center->setY(arc_center.y());
  }

  double radius = (pt - arc_center).length();

  // turn left is positive
  return radius < kMathEpsilon ? 0.0 : std::copysign(1.0 / radius, crossProd(pt_prev, pt, arc_center));
}

/**
 * @brief Direction of a line which contains pt and is tangential to arc
 * @param pt_prev Starting point of the arc
 * @param pt Mid point of the arc
 * @param pt_next Last point of the arc
 * @param is_cusp True if pt is a cusp point
 * @result Tangential line direction.
 * @note the sign of tangentDir is undefined here, should be assigned in post-process depending
 *       on movement direction. Also, for speed reasons, direction vector is not normalized.
 */
rmp::common::geometry::Vec2d tangentDir(const rmp::common::geometry::Vec2d& pt_prev,
                                        const rmp::common::geometry::Vec2d& pt,
                                        const rmp::common::geometry::Vec2d& pt_next, bool is_cusp)
{
  rmp::common::geometry::Vec2d center;
  arcCenter(pt_prev, pt, pt_next, is_cusp, &center);

  // straight line
  if (std::isinf(center.x()) && std::isinf(center.y()))
  {
    auto d1 = pt - pt_prev;
    auto d2 = pt_next - pt;
    auto pt_next_cusp = is_cusp ? pt - d2 : pt_next;

    auto result = rmp::common::geometry::Vec2d(pt_next_cusp.x() - pt_prev.x(), pt_next_cusp.y() - pt_prev.y());
    if (std::fabs(result.x()) < kMathEpsilon && std::fabs(result.y()) < kMathEpsilon)
    {
      return { d1.y(), -d1.x() };
    }
    return result;
  }

  // tangent is perpendicular to (pt - center)
  // Note: not determining + or - direction here, this should be handled at the caller side
  return { center.y() - pt.y(), pt.x() - center.x() };
}

/**
 * @brief Sort a set of points in counterclockwise order
 * @param points a set of points to sort
 */
void sortPoints(std::vector<rmp::common::geometry::Vec2d>& points)
{
  double sum_x = 0, sum_y = 0;
  for (const auto& p : points)
  {
    sum_x += p.x();
    sum_y += p.y();
  }

  rmp::common::geometry::Vec2d middle_point(sum_x / static_cast<double>(points.size()),
                                            sum_y / static_cast<double>(points.size()));
  std::sort(points.begin(), points.end(),
            [&](const rmp::common::geometry::Vec2d& pt1, const rmp::common::geometry::Vec2d& pt2) {
              const auto& orien_v_1 = pt1 - middle_point;
              const auto& orien_v_2 = pt2 - middle_point;
              return orien_v_1.angle() < orien_v_2.angle();
            });
}

}  // namespace math
}  // namespace common
}  // namespace rmp
