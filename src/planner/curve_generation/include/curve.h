/***********************************************************
 *
 * @file: curve.h
 * @breif: Trajectory generation
 * @author: Yang Haodong
 * @update: 2023-12-20
 * @version: 1.0
 *
 * Copyright (c) 2023, Yang Haodong
 * All rights reserved.
 * --------------------------------------------------------
 *
 **********************************************************/
#ifndef CURVE_H
#define CURVE_H

#include <vector>
#include <cmath>

namespace trajectory_generation
{
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
   * @param points path points
   * @param path generated trajectory
   * @return true if generate successfully, else failed
   */
  virtual bool run(const std::vector<std::pair<double, double>> points,
                   std::vector<std::pair<double, double>>& path) = 0;

  /**
   * @brief Calculate distance between the 2 nodes.
   * @param n1 Node 1
   * @param n2 Node 2
   * @return distance between nodes
   */
  double dist(const std::pair<double, double>& node1, const std::pair<double, double>& node2);

  /**
   * @brief Calculate length of given path.
   * @param path    the trajectory
   * @return length the length of path
   */
  double len(std::vector<std::pair<double, double>> path);

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