/**
 * *********************************************************
 *
 * @file: bezier_curve.cpp
 * @brief: Bezier curve generation
 * @author: Yang Haodong
 * @date: 2023-12-22
 * @version: 1.0
 *
 * Copyright (c) 2024, Yang Haodong.
 * All rights reserved.
 *
 * --------------------------------------------------------
 *
 * ********************************************************
 */
#include <cassert>

#include "common/geometry/curve/bezier_curve.h"

namespace rmp
{
namespace common
{
namespace geometry
{
/**
 * @brief Construct a new Bezier generation object
 * @param step        Simulation or interpolation size (default: 0.1)
 * @param offset      The offset of control points (default: 3.0)
 */
BezierCurve::BezierCurve(double step, double offset) : Curve(step), offset_(offset)
{
}
BezierCurve::BezierCurve() : Curve(0.1), offset_(3)
{
}

/**
 * @brief Destroy the Bezier generation object
 */
BezierCurve::~BezierCurve()
{
}

/**
 * @brief Calculate the Bezier curve point.
 * @param t scale factor
 * @param control_pts control points
 * @return point point in Bezier curve with t
 */
Point2d BezierCurve::bezier(double t, const Points2d& control_pts)
{
  size_t n = control_pts.size() - 1;
  double pt_x = 0, pt_y = 0;
  for (size_t i = 0; i < n + 1; i++)
  {
    pt_x += _comb(n, i) * std::pow(t, i) * std::pow(1 - t, n - i) * control_pts[i].x();
    pt_y += _comb(n, i) * std::pow(t, i) * std::pow(1 - t, n - i) * control_pts[i].y();
  }
  return { pt_x, pt_y };
}

/**
 * @brief Calculate control points heuristically.
 * @param start Initial pose (x, y, yaw)
 * @param goal  Target pose (x, y, yaw)
 * @return control_pts control points
 */
Points2d BezierCurve::getControlPoints(const Point3d& start, const Point3d& goal)
{
  double sx = start.x(), sy = start.y(), syaw = start.theta();
  double gx = goal.x(), gy = goal.y(), gyaw = goal.theta();

  double d = std::hypot(sx - gx, sy - gy) * offset_;

  Points2d control_pts;
  control_pts.emplace_back(sx, sy);
  control_pts.emplace_back(sx + d * cos(syaw), sy + d * sin(syaw));
  control_pts.emplace_back(gx - d * cos(gyaw), gy - d * sin(gyaw));
  control_pts.emplace_back(gx, gy);

  return control_pts;
}

/**
 * @brief Generate the path.
 * @param start Initial pose (x, y, yaw)
 * @param goal  Target pose (x, y, yaw)
 * @param path  The smoothed trajectory points
 * @return true if generate successfully, else failed
 */
bool BezierCurve::generation(const Point3d& start, const Point3d& goal, Points3d& path)
{
  double sx = start.x(), sy = start.y(), syaw = start.theta();
  double gx = goal.x(), gy = goal.y(), gyaw = goal.theta();

  size_t n_points = static_cast<size_t>(std::hypot(sx - gx, sy - gy) / step_);
  const Points2d& control_pts = getControlPoints(start, goal);

  path.clear();
  path.reserve(n_points);
  for (size_t i = 0; i < n_points; i++)
  {
    double t = static_cast<double>(i) / static_cast<double>(n_points - 1);
    const auto& waypoint = bezier(t, control_pts);
    path.emplace_back(waypoint.x(), waypoint.y());
  }

  path[0].setTheta(start.theta());
  path.back().setTheta(goal.theta());
  for (size_t i = 1; i < n_points - 1; i++)
  {
    auto prev_interp_vec = Vec2d(path[i - 1].x(), path[i - 1].y());
    auto curr_interp_vec = Vec2d(path[i].x(), path[i].y());
    auto next_interp_vec = Vec2d(path[i + 1].x(), path[i + 1].y());
    auto tangent_dir = rmp::common::math::tangentDir(prev_interp_vec, curr_interp_vec, next_interp_vec, false);
    auto dir = tangent_dir.innerProd(curr_interp_vec - prev_interp_vec) >= 0 ? tangent_dir : -tangent_dir;
    path[i].setTheta(dir.angle());
  }

  return true;
}

/**
 * @brief Running trajectory generation
 * @param points path points <x, y>
 * @param path generated trajectory
 * @return true if generate successfully, else failed
 */
bool BezierCurve::run(const Points2d& points, Points3d& path)
{
  if (points.size() < 4)
  {
    return false;
  }
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
bool BezierCurve::run(const Points3d& points, Points3d& path)
{
  if (points.size() < 4)
  {
    return false;
  }
  else
  {
    path.clear();
    Points3d path_i;
    for (size_t i = 0; i < points.size() - 1; i++)
    {
      if (!generation(points[i], points[i + 1], path_i))
      {
        return false;
      }
      path.insert(path.end(), path_i.begin(), path_i.end());
    }

    return !path.empty();
  }
}

/**
 * @brief Configure the offset of control points.
 * @param offset  The offset of control points
 */
void BezierCurve::setOffset(double offset)
{
  assert(offset > 0);
  offset_ = offset;
}

// Calculate the number of combinations
int BezierCurve::_comb(int n, int r)
{
  if ((r == 0) || (r == n))
    return 1;
  else
    return _comb(n - 1, r - 1) + _comb(n - 1, r);
}

Eigen::Matrix<double, 6, 6> Bernstein::matrix5_ =
    (Eigen::Matrix<double, 6, 6>() << 1, 0, 0, 0, 0, 0, -5, 5, 0, 0, 0, 0, 10, -20, 10, 0, 0, 0, -10, 30, -30, 10, 0, 0,
     5, -20, 30, -20, 5, 0, -1, 5, -10, 10, -5, 1)
        .finished();

}  // namespace geometry
}  // namespace common
}  // namespace rmp