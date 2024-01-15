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
#ifndef BEZIER_CURVE_H
#define BEZIER_CURVE_H

#include "curve.h"

namespace trajectory_generation
{
class Bezier : public Curve
{
public:
  /**
   * @brief Construct a new Bezier generation object
   * @param step        Simulation or interpolation size (default: 0.1)
   * @param offset      The offset of control points (default: 3.0)
   */
  Bezier();
  Bezier(double step, double offset);

  /**
   * @brief Destroy the Bezier generation object
   */
  ~Bezier();

  /**
   * @brief Running trajectory generation
   * @param points path points <x, y>
   * @param path generated trajectory
   * @return true if generate successfully, else failed
   */
  bool run(const Points2d points, Points2d& path);

  /**
   * @brief Running trajectory generation
   * @param points path points <x, y, theta>
   * @param path generated trajectory
   * @return true if generate successfully, else failed
   */
  bool run(const Poses2d points, Points2d& path);

  /**
   * @brief Calculate the Bezier curve point.
   * @param t scale factor
   * @param control_pts control points
   * @return point point in Bezier curve with t
   */
  Point2d bezier(double t, Points2d control_pts);

  /**
   * @brief Calculate control points heuristically.
   * @param start Initial pose (x, y, yaw)
   * @param goal  Target pose (x, y, yaw)
   * @return control_pts control points
   */
  Points2d getControlPoints(Pose2d start, Pose2d goal);

  /**
   * @brief Generate the path.
   * @param start Initial pose (x, y, yaw)
   * @param goal  Target pose (x, y, yaw)
   * @return path The smoothed trajectory points
   */
  Points2d generation(Pose2d start, Pose2d goal);

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

}  // namespace trajectory_generation

#endif