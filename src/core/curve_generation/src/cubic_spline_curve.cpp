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

#include "cubic_spline_curve.h"

namespace trajectory_generation
{
/**
 * @brief Construct a new Cubic spline generation object
 * @param step        Simulation or interpolation size (default: 0.1)
 * @param offset      The offset of control points (default: 3.0)
 */
CubicSpline::CubicSpline(double step) : Curve(step)
{
}
CubicSpline::CubicSpline() : Curve(0.1)
{
}

/**
 * @brief Destroy the Cubic spline generation object
 */
CubicSpline::~CubicSpline()
{
}

/**
 * @brief Calculate the spline curve value in a certain direction
 * @param s_list distance vector
 * @param dir_list direction vector (-x or -y)
 * @param t scale factor
 * @return spline_val_dir  the spline curve value in a certain direction
 */
std::vector<double> CubicSpline::spline(std::vector<double> s_list, std::vector<double> dir_list, std::vector<double> t)
{
  // cubic polynomial functions
  std::vector<double> a = dir_list;
  std::vector<double> b, d;

  size_t num = s_list.size();

  std::vector<double> h;
  for (size_t i = 0; i < num - 1; i++)
    h.push_back(s_list[i + 1] - s_list[i]);

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
 * @brief Running trajectory generation
 * @param points path points <x, y>
 * @param path generated trajectory
 * @return true if generate successfully, else failed
 */
bool CubicSpline::run(const Points2d points, Points2d& path)
{
  if (points.size() < 4)
    return false;
  else
  {
    std::vector<double> x_list, y_list;
    for (const auto& p : points)
    {
      x_list.push_back(p.first);
      y_list.push_back(p.second);
    }

    std::vector<double> s;
    s.push_back(0.0);

    double d_cumsum = 0.0;
    for (size_t i = 0; i < points.size() - 1; i++)
    {
      double d = helper::dist(points[i], points[i + 1]);
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

    std::vector<double> path_x = spline(s, x_list, t);
    std::vector<double> path_y = spline(s, y_list, t);

    path.clear();
    for (size_t i = 0; i < path_x.size(); i++)
      path.emplace_back(path_x[i], path_y[i]);

    return !path.empty();
  }
}

/**
 * @brief Running trajectory generation
 * @param points path points <x, y, theta>
 * @param path generated trajectory
 * @return true if generate successfully, else failed
 */
bool CubicSpline::run(const Poses2d points, Points2d& path)
{
  if (points.size() < 4)
    return false;
  else
  {
    Points2d points_;
    for (const auto& p : points)
      points_.emplace_back(std::get<0>(p), std::get<1>(p));

    return run(points_, path);
  }
}

}  // namespace trajectory_generation