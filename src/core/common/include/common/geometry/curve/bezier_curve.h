/**
 * *********************************************************
 *
 * @file: bezier_curve.h
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
#ifndef RMP_COMMON_GEOMETRY_BEZIER_CURVE_H_
#define RMP_COMMON_GEOMETRY_BEZIER_CURVE_H_

#include <Eigen/Dense>

#include "common/geometry/curve/curve.h"

namespace rmp
{
namespace common
{
namespace geometry
{
class BezierCurve : public Curve
{
public:
  /**
   * @brief Construct a new Bezier generation object
   * @param step        Simulation or interpolation size (default: 0.1)
   * @param offset      The offset of control points (default: 3.0)
   */
  BezierCurve();
  BezierCurve(double step, double offset);

  /**
   * @brief Destroy the Bezier generation object
   */
  ~BezierCurve();

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
   * @brief Calculate the Bezier curve point.
   * @param t scale factor
   * @param control_pts control points
   * @return point point in Bezier curve with t
   */
  Point2d bezier(double t, const Points2d& control_pts);

  /**
   * @brief Calculate control points heuristically.
   * @param start Initial pose (x, y, yaw)
   * @param goal  Target pose (x, y, yaw)
   * @return control_pts control points
   */
  Points2d getControlPoints(const Point3d& start, const Point3d& goal);

  /**
   * @brief Generate the path.
   * @param start Initial pose (x, y, yaw)
   * @param goal  Target pose (x, y, yaw)
   * @param path  The smoothed trajectory points
   * @return true if generate successfully, else failed
   */
  bool generation(const Point3d& start, const Point3d& goal, Points3d& path);

  /**
   * @brief Configure the offset of control points.
   * @param offset  The offset of control points
   */
  void setOffset(double offset);

private:
  // Calculate the number of combinations
  int _comb(int n, int r);

protected:
  double offset_;  // The offset of control points
};

class Bernstein
{
public:
  Bernstein() = default;
  ~Bernstein() = default;

  static Eigen::MatrixXd matrix(int n)
  {
    if (n == 5U)
    {
      return matrix5_;
    }
    else
    {
      return Eigen::MatrixXd::Zero(n, n);
    }
  }

private:
  static Eigen::Matrix<double, 6, 6> matrix5_;
};
}  // namespace geometry
}  // namespace common
}  // namespace rmp
#endif