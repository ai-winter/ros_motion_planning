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
#include "bezier_curve.h"

namespace trajectory_generation
{
/**
 * @brief Construct a new Bezier generation object
 * @param step        Simulation or interpolation size (default: 0.1)
 * @param offset      The offset of control points (default: 3.0)
 */
Bezier::Bezier(double step, double offset) : Curve(step), offset_(offset)
{
}
Bezier::Bezier() : Curve(0.1), offset_(3)
{
}

/**
 * @brief Destroy the Bezier generation object
 */
Bezier::~Bezier()
{
}

/**
 * @brief Calculate the Bezier curve point.
 * @param t scale factor
 * @param control_pts control points
 * @return point point in Bezier curve with t
 */
Point2d Bezier::bezier(double t, Points2d control_pts)
{
  size_t n = control_pts.size() - 1;
  Point2d pt(0, 0);
  for (size_t i = 0; i < n + 1; i++)
  {
    pt.first += _comb(n, i) * std::pow(t, i) * std::pow(1 - t, n - i) * control_pts[i].first;
    pt.second += _comb(n, i) * std::pow(t, i) * std::pow(1 - t, n - i) * control_pts[i].second;
  }
  return pt;
}

/**
 * @brief Calculate control points heuristically.
 * @param start Initial pose (x, y, yaw)
 * @param goal  Target pose (x, y, yaw)
 * @return control_pts control points
 */
Points2d Bezier::getControlPoints(Pose2d start, Pose2d goal)
{
  double sx, sy, syaw;
  double gx, gy, gyaw;
  std::tie(sx, sy, syaw) = start;
  std::tie(gx, gy, gyaw) = goal;

  double d = helper::dist(Point2d(sx, sy), Point2d(gx, gy)) / offset_;

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
 * @return path The smoothed trajectory points
 */
Points2d Bezier::generation(Pose2d start, Pose2d goal)
{
  double sx, sy, syaw;
  double gx, gy, gyaw;
  std::tie(sx, sy, syaw) = start;
  std::tie(gx, gy, gyaw) = goal;

  int n_points = (int)(helper::dist(Point2d(sx, sy), Point2d(gx, gy)) / step_);
  Points2d control_pts = getControlPoints(start, goal);

  Points2d points;
  for (size_t i = 0; i < n_points; i++)
  {
    double t = (double)(i) / (double)(n_points - 1);
    points.push_back(bezier(t, control_pts));
  }

  return points;
}

/**
 * @brief Running trajectory generation
 * @param points path points <x, y>
 * @param path generated trajectory
 * @return true if generate successfully, else failed
 */
bool Bezier::run(const Points2d points, Points2d& path)
{
  if (points.size() < 4)
    return false;
  else
  {
    Poses2d poses;
    poses.emplace_back(points.begin()->first, points.begin()->second, 0);
    for (size_t i = 1; i < points.size() - 1; i++)
    {
      double theta1 = helper::angle(points[i - 1], points[i]);
      double theta2 = helper::angle(points[i], points[i + 1]);
      poses.emplace_back(points[i].first, points[i].second, (theta1 + theta2) / 2);
    }
    poses.emplace_back(points.back().first, points.back().second, 0);

    return run(poses, path);
  }
}

/**
 * @brief Running trajectory generation
 * @param points path points <x, y, theta>
 * @param path generated trajectory
 * @return true if generate successfully, else failed
 */
bool Bezier::run(const Poses2d points, Points2d& path)
{
  if (points.size() < 4)
    return false;
  else
  {
    path.clear();
    for (size_t i = 0; i < points.size() - 1; i++)
    {
      Points2d path_i = generation(points[i], points[i + 1]);
      path.insert(path.end(), path_i.begin(), path_i.end());
    }

    return !path.empty();
  }
}

/**
 * @brief Configure the offset of control points.
 * @param offset  The offset of control points
 */
void Bezier::setOffset(double offset)
{
  assert(offset > 0);
  offset_ = offset;
}

// Calculate the number of combinations
int Bezier::_comb(int n, int r)
{
  if ((r == 0) || (r == n))
    return 1;
  else
    return _comb(n - 1, r - 1) + _comb(n - 1, r);
}
}  // namespace trajectory_generation