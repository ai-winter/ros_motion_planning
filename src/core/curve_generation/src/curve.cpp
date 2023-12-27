/***********************************************************
 *
 * @file: curve.cpp
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
#include <cassert>
#include "curve.h"

namespace trajectory_generation
{
/**
 * @brief Construct a new Curve object
 * @param step  Simulation or interpolation size
 */
Curve::Curve(double step) : step_(step)
{
}

/**
 * @brief Calculate distance between the 2 nodes.
 * @param n1 Node 1
 * @param n2 Node 2
 * @return distance between nodes
 */
double Curve::dist(const Point2d& node1, const Point2d& node2)
{
  return std::hypot(node1.first - node2.first, node1.second - node2.second);
}

/**
 * @brief Calculate the angle of x-axis between the 2 nodes.
 * @param n1 Node 1
 * @param n2 Node 2
 * @return the angle of x-axis between the 2 node
 */
double Curve::angle(const Point2d& node1, const Point2d& node2)
{
  return atan2(node2.second - node1.second, node2.first - node1.first);
}

/**
 * @brief Calculate length of given path.
 * @param path    the trajectory
 * @return length the length of path
 */
double Curve::len(Points2d path)
{
  double length = 0.0;
  for (size_t i = 1; i < path.size(); ++i)
    length += dist(path[i - 1], path[i]);
  return length;
}

/**
 * @brief Perform modulus operation on 2π.
 * @param theta    the angle to modulu
 * @return theta_m the angle after modulus operator
 */
double Curve::mod2pi(double theta)
{
  return theta - 2.0 * M_PI * floor(theta / M_PI / 2.0);
}

/**
 * @brief Truncate the angle to the interval of -π to π.
 * @param theta    the angle to truncate
 * @return theta_t the truncated angle
 */
double Curve::pi2pi(double theta)
{
  while (theta > M_PI)
    theta -= 2.0 * M_PI;
  while (theta < -M_PI)
    theta += 2.0 * M_PI;
  return theta;
}

/**
 * @brief Configure the simulation step.
 * @param step    Simulation or interpolation size
 */
void Curve::setStep(double step)
{
  assert(step > 0);
  step_ = step;
}
}  // namespace trajectory_generation