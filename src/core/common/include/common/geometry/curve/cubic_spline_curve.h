/**
 * *********************************************************
 *
 * @file: cubic_spline_curve.h
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
#ifndef RMP_COMMON_GEOMETRY_CUBIC_SPLINE_CURVE_H_
#define RMP_COMMON_GEOMETRY_CUBIC_SPLINE_CURVE_H_

#include "common/geometry/curve/curve.h"

namespace rmp
{
namespace common
{
namespace geometry
{
class CubicSplineCurve : public Curve
{
public:
  /**
   * @brief Construct a new Cubic spline generation object
   * @param step        Simulation or interpolation size (default: 0.1)
   */
  CubicSplineCurve();
  CubicSplineCurve(double step);

  /**
   * @brief Destroy the Cubic spline generation object
   */
  ~CubicSplineCurve();

  /**
   * @brief Running trajectory generation
   * @param points path points <x, y>
   * @param path generated trajectory
   * @return true if generate successfully, else failed
   */
  bool run(const Points2d& points, Points3d& path);

  /**
   * @brief Running trajectory generation
   * @param points path points <x, y, theta>
   * @param path generated trajectory
   * @return true if generate successfully, else failed
   */
  bool run(const Points3d& points, Points3d& path);

  /**
   * @brief Calculate the spline curve value in a certain direction
   * @param s_list distance vector
   * @param dir_list direction vector (-x or -y)
   * @param t scale factor
   * @return spline_val_dir  the spline curve value in a certain direction
   */
  std::vector<double> spline(const std::vector<double>& s_list, const std::vector<double>& dir_list,
                             const std::vector<double>& t);

  /**
   * @brief Generate the path.
   * @param start Initial pose (x, y, yaw)
   * @param goal  Target pose (x, y, yaw)
   * @param path  The smoothed trajectory points
   * @return true if generate successfully, else failed
   */
  bool generation(const Point3d& start, const Point3d& goal, Points3d& path);

protected:
  double start_angle_{ 0.0 }, goal_angle_{ 0.0 };
};
}  // namespace geometry
}  // namespace common
}  // namespace rmp
#endif