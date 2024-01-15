/**
 * *********************************************************
 *
 * @file: polynomial_curve.h
 * @brief: Polynomial curve generation
 * @author: Yang Haodong
 * @date: 2023-12-26
 * @version: 1.0
 *
 * Copyright (c) 2024, Yang Haodong. 
 * All rights reserved.
 * 
 * --------------------------------------------------------
 *
 * ********************************************************
 */
#ifndef POLYNOMIAL_CURVE_H
#define POLYNOMIAL_CURVE_H

#define POLY_DIR_ACC 0
#define POLY_DIR_JERK 1

#include "curve.h"

namespace trajectory_generation
{
// x, y, yaw, v, a
using PolyState = std::tuple<double, double, double, double, double>;

// Polynomial interpolation solver
class Poly
{
public:
  Poly(std::tuple<double, double, double> state_0, std::tuple<double, double, double> state_1, double t);
  ~Poly();

  double x(double t);
  double dx(double t);
  double ddx(double t);
  double dddx(double t);

protected:
  double p0, p1, p2, p3, p4, p5;  // Quintic polynomial coefficient
};

class PolyTrajectory
{
public:
  /**
   * @brief Construct a new Polynomial trajectory  object
   */
  PolyTrajectory();

  /**
   * @brief Destroy the Polynomial trajectory object
   */
  ~PolyTrajectory();

  /**
   * @brief Clear the Polynomial trajectory
   */
  void clear();

  /**
   * @brief Append the state to Polynomial trajectory
   * @param time  Current time step
   * @param x     Current x-position
   * @param y     Current y-position
   * @param yaw   Current yaw angle
   * @param v     Current speed
   * @param a     Current acceleration
   * @param jerk  Current jerk
   */
  void append(double time, double x, double y, double yaw, double v, double a, double jerk);

  /**
   * @brief Determine the direction of the motion trajectory
   * @param mode  trajectory mode, i.e., acceleration or jerk
   * @return direction  1 is positive direction and -1 is nagetive, 0 is invalid
   */
  int dir(int mode);

  /**
   * @brief Determine whether the generated trajectory is valid
   * @param max_acc     Maximum acceleration
   * @param max_jerk    Maximum jerk
   * @return flag       true is valid else invalid
   */
  bool valid(double max_acc, double max_jerk);

  /**
   * @brief Calculate the size of trajectory
   * @return size   the size of trajectory
   */
  double size();

  /**
   * @brief Convert the trajectory to path points
   * @return path  path points (x, y)
   */
  Points2d toPath();

protected:
  std::vector<double> time_;
  std::vector<double> x_;
  std::vector<double> y_;
  std::vector<double> yaw_;
  std::vector<double> v_;
  std::vector<double> a_;
  std::vector<double> jerk_;
};

class Polynomial : public Curve
{
public:
  /**
   * @brief Construct a new Polynomial generation object
   * @param max_acc     Maximum acceleration (default: 1.0)
   * @param max_jerk    Maximum jerk (default: 0.5)
   */
  Polynomial();
  Polynomial(double step, double max_acc, double max_jerk);

  /**
   * @brief Destroy the Polynomial generation object
   */
  ~Polynomial();

  /**
   * @brief Generate a valid trajectory from start state to goal state
   * @param start_state   start state
   * @param goal_state    goal state
   * @param traj    the trajectory
   */
  void generation(PolyState start_state, PolyState goal_state, PolyTrajectory& traj);

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
   * @brief Configure the maximum acceleration.
   * @param max_acc  The maximum acceleration
   */
  void setMaxAcceleration(double max_acc);

  /**
   * @brief Configure the maximum jerk.
   * @param max_jerk  The maximum jerk
   */
  void setMaxJerk(double max_jerk);

protected:
  double max_acc_;   // Maximum acceleration
  double max_jerk_;  // Maximum jerk
};
}  // namespace trajectory_generation
#endif