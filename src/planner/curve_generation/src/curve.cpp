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
double Curve::dist(const std::pair<double, double>& node1, const std::pair<double, double>& node2)
{
  return std::hypot(node1.first - node2.first, node1.second - node2.second);
}

/**
 * @brief Calculate length of given path.
 * @param path    the trajectory
 * @return length the length of path
 */
double Curve::len(std::vector<std::pair<double, double>> path)
{
  double length = 0.0;
  for (size_t i = 1; i < path.size(); ++i)
    length += dist(path[i - 1], path[i]);
  return length;
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