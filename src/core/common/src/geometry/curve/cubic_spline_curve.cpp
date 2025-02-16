/**
 * *********************************************************
 *
 * @file: cubic_spline_curve.cpp
 * @brief: Cubic spline generation
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
#include <Eigen/Dense>

#include "common/geometry/curve/cubic_spline_curve.h"

namespace rmp
{
namespace common
{
namespace geometry
{
/**
 * @brief Construct a new Cubic spline generation object
 * @param step        Simulation or interpolation size (default: 0.1)
 * @param offset      The offset of control points (default: 3.0)
 */
CubicSplineCurve::CubicSplineCurve(double step) : Curve(step)
{
}
CubicSplineCurve::CubicSplineCurve() : Curve(0.1)
{
}

/**
 * @brief Destroy the Cubic spline generation object
 */
CubicSplineCurve::~CubicSplineCurve()
{
}

/**
 * @brief Calculate the spline curve value in a certain direction
 * @param s_list distance vector
 * @param dir_list direction vector (-x or -y)
 * @param t scale factor
 * @return spline_val_dir  the spline curve value in a certain direction
 */
std::vector<double> CubicSplineCurve::spline(const std::vector<double>& s_list, const std::vector<double>& dir_list,
                                             const std::vector<double>& t)
{
  // cubic polynomial functions
  std::vector<double> a = dir_list;
  std::vector<double> b, d;

  size_t num = s_list.size();

  std::vector<double> h;
  for (size_t i = 0; i < num - 1; i++)
  {
    h.push_back(s_list[i + 1] - s_list[i]);
  }

  // calculate coefficient matrix
  Eigen::MatrixXd A = Eigen::MatrixXd::Zero(num, num);
  for (size_t i = 1; i < num - 1; i++)
  {
    A(i, i - 1) = h[i - 1];
    A(i, i) = 2.0 * (h[i - 1] + h[i]);
    A(i, i + 1) = h[i];
  }
  A(0, 0) = 1.0;
  A(num - 1, num - 1) = 1.0;

  Eigen::MatrixXd B = Eigen::MatrixXd::Zero(num, 1);
  for (size_t i = 1; i < num - 1; i++)
    B(i, 0) = 3.0 * (a[i + 1] - a[i]) / h[i] - 3.0 * (a[i] - a[i - 1]) / h[i - 1];

  Eigen::MatrixXd c = A.lu().solve(B);
  for (size_t i = 0; i < num - 1; i++)
  {
    b.push_back((a[i + 1] - a[i]) / h[i] - h[i] * (c(i + 1) + 2.0 * c(i)) / 3.0);
    d.push_back((c(i + 1) - c(i)) / (3.0 * h[i]));
  }

  // calculate spline value and its derivative
  std::vector<double> p;
  for (const auto it : t)
  {
    auto iter = std::find_if(s_list.begin(), s_list.end(), [it](double val) { return val > it; });
    if (iter != s_list.end())
    {
      size_t idx = std::distance(s_list.begin(), iter) - 1;
      double ds = it - s_list[idx];
      p.push_back(a[idx] + b[idx] * ds + c(idx) * std::pow(ds, 2) + d[idx] * std::pow(ds, 3));
    }
  }
  return p;
}

/**
 * @brief Generate the path.
 * @param start Initial pose (x, y, yaw)
 * @param goal  Target pose (x, y, yaw)
 * @param path  The smoothed trajectory points
 * @return true if generate successfully, else failed
 */
bool CubicSplineCurve::generation(const Point3d& start, const Point3d& goal, Points3d& path)
{
  R_ERROR << "This function is forbidden";
  return false;
}

/**
 * @brief Running trajectory generation
 * @param points path points <x, y>
 * @param path generated trajectory
 * @return true if generate successfully, else failed
 */
bool CubicSplineCurve::run(const Points2d& points, Points3d& path)
{
  if (points.size() < 4)
  {
    return false;
  }
  else
  {
    std::vector<double> x_list, y_list;
    for (const auto& p : points)
    {
      x_list.push_back(p.x());
      y_list.push_back(p.y());
    }

    std::vector<double> s;
    s.push_back(0.0);

    double d_cumsum = 0.0;
    for (size_t i = 0; i < points.size() - 1; i++)
    {
      double d = std::hypot(points[i].x() - points[i + 1].x(), points[i].y() - points[i + 1].y());
      d_cumsum += d;
      s.push_back(d_cumsum);
    }

    double ds = 0.0;
    std::vector<double> t;
    while (ds < s.back())
    {
      t.push_back(ds);
      ds += step_;
    }

    const std::vector<double>& path_x = spline(s, x_list, t);
    const std::vector<double>& path_y = spline(s, y_list, t);

    path.clear();
    path.reserve(path_x.size());
    path.emplace_back(path_x[0], path_y[0], start_angle_);
    for (size_t i = 1; i < path_x.size() - 1; i++)
    {
      auto prev_interp_vec = Vec2d(path_x[i - 1], path_y[i - 1]);
      auto curr_interp_vec = Vec2d(path_x[i], path_y[i]);
      auto next_interp_vec = Vec2d(path_x[i + 1], path_y[i + 1]);
      auto tangent_dir = rmp::common::math::tangentDir(prev_interp_vec, curr_interp_vec, next_interp_vec, false);
      auto dir = tangent_dir.innerProd(curr_interp_vec - prev_interp_vec) >= 0 ? tangent_dir : -tangent_dir;
      path.emplace_back(path_x[i], path_y[i], dir.angle());
    }
    path.emplace_back(path_x.back(), path_y.back(), goal_angle_);

    return !path.empty();
  }
}

/**
 * @brief Running trajectory generation
 * @param points path points <x, y, theta>
 * @param path generated trajectory
 * @return true if generate successfully, else failed
 */
bool CubicSplineCurve::run(const Points3d& points, Points3d& path)
{
  start_angle_, goal_angle_ = points[0].theta(), points.back().theta();
  if (points.size() < 4)
  {
    return false;
  }
  else
  {
    Points2d points_;
    for (const auto& p : points)
      points_.emplace_back(p.x(), p.y());

    return run(points_, path);
  }
}
}  // namespace geometry
}  // namespace common
}  // namespace rmp