/***********************************************************
 *
 * @file: optimizer.h
 * @breif: Trajectory optimization
 * @author: Yang Haodong
 * @update: 2023-12-29
 * @version: 1.0
 *
 * Copyright (c) 2023, Yang Haodong
 * All rights reserved.
 * --------------------------------------------------------
 *
 **********************************************************/
#ifndef OPTIMIZER_H
#define OPTIMIZER_H

#include <vector>
#include <tuple>
#include <cmath>

#include "dynamicvoronoi.h"

namespace trajectory_optimization
{
// <x, y>
using Point2d = std::pair<double, double>;
using Points2d = std::vector<Point2d>;
// <x, y, dir>
using DirPoint2d = std::tuple<double, double, int>;
using DirPoints2d = std::vector<std::tuple<double, double, int>>;

class Optimizer
{
public:
  /**
   * @brief Construct a new trajectory optimizer object
   * @param max_iter the maximum iterations for optimization
   */
  Optimizer(int max_iter);

  /**
   * @brief Destroy the trajectory optimizer object
   */
  virtual ~Optimizer() = default;

  /**
   * @brief Running trajectory optimization
   * @param path_i path points <x, y> before optimization
   * @param path_o path points <x, y> after optimization
   * @return true if optimizes successfully, else failed
   */
  virtual bool run(const Points2d path_i, Points2d& path_o) = 0;

  void setVoronoiDiagram(DynamicVoronoi& voronoi);

  /**
   * @brief Clamps a value within a specified range.
   * @tparam T             The type of the values to be clamped.
   * @param value          The value to be clamped.
   * @param low            The lower bound of the range.
   * @param high           The upper bound of the range.
   * @return const T&      The clamped value within the specified range.
   */
  template <typename T>
  const T& clamp(const T& value, const T& low, const T& high)
  {
    return std::max(low, std::min(value, high));
  }

protected:
  int max_iter_;            // the maximum iterations for optimization
  DynamicVoronoi voronoi_;  // dynamic voronoi map
};

}  // namespace trajectory_optimization

#endif