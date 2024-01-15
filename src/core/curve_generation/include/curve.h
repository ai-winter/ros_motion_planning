/**
 * *********************************************************
 *
 * @file: curve.h
 * @brief: Curve generation
 * @author: Yang Haodong
 * @date: 2023-12-20
 * @version: 1.0
 *
 * Copyright (c) 2024, Yang Haodong. 
 * All rights reserved.
 * 
 * --------------------------------------------------------
 *
 * ********************************************************
 */
#ifndef CURVE_H
#define CURVE_H

#include <vector>
#include <tuple>
#include <cmath>

#include <math_helper.h>

namespace trajectory_generation
{
using Point2d = std::pair<double, double>;
using Points2d = std::vector<Point2d>;
using Pose2d = std::tuple<double, double, double>;
using Poses2d = std::vector<Pose2d>;

class Curve
{
public:
  /**
   * @brief Construct a new Curve object
   * @param step  Simulation or interpolation size
   */
  Curve(double step);

  /**
   * @brief Destroy the curve generation object
   */
  virtual ~Curve() = default;

  /**
   * @brief Running trajectory generation
   * @param points path points <x, y>
   * @param path generated trajectory
   * @return true if generate successfully, else failed
   */
  virtual bool run(const Points2d points, Points2d& path) = 0;

  /**
   * @brief Running trajectory generation
   * @param points path points <x, y, theta>
   * @param path generated trajectory
   * @return true if generate successfully, else failed
   */
  virtual bool run(const Poses2d points, Points2d& path) = 0;

  /**
   * @brief Calculate length of given path.
   * @param path    the trajectory
   * @return length the length of path
   */
  double len(Points2d path);

  /**
   * @brief Configure the simulation step.
   * @param step    Simulation or interpolation size
   */
  void setStep(double step);

protected:
  double step_;  // Simulation or interpolation size
};
}  // namespace trajectory_generation

#endif