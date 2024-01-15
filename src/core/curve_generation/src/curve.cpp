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
 * @brief Calculate length of given path.
 * @param path    the trajectory
 * @return length the length of path
 */
double Curve::len(Points2d path)
{
  double length = 0.0;
  for (size_t i = 1; i < path.size(); ++i)
    length += helper::dist(path[i - 1], path[i]);
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