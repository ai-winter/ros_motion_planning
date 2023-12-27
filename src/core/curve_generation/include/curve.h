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
#include <tuple>
#include <cmath>

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
   * @brief Calculate distance between the 2 nodes.
   * @param n1 Node 1
   * @param n2 Node 2
   * @return distance between nodes
   */
  double dist(const Point2d& node1, const Point2d& node2);

  /**
   * @brief Calculate the angle of x-axis between the 2 nodes.
   * @param n1 Node 1
   * @param n2 Node 2
   * @return the angle of x-axis between the 2 node
   */
  double angle(const Point2d& node1, const Point2d& node2);

  /**
   * @brief Calculate length of given path.
   * @param path    the trajectory
   * @return length the length of path
   */
  double len(Points2d path);

  /**
   * @brief Perform modulus operation on 2π.
   * @param theta    the angle to modulu
   * @return theta_m the angle after modulus operator
   */
  double mod2pi(double theta);

  /**
   * @brief Truncate the angle to the interval of -π to π.
   * @param theta    the angle to truncate
   * @return theta_t the truncated angle
   */
  double pi2pi(double theta);

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