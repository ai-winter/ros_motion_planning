/**
 * *********************************************************
 *
 * @file: dubins_curve.h
 * @brief: Dubins curve generation
 * @author: Yang Haodong
 * @date: 2023-12-23
 * @version: 1.0
 *
 * Copyright (c) 2024, Yang Haodong. 
 * All rights reserved.
 * 
 * --------------------------------------------------------
 *
 * ********************************************************
 */
#ifndef DUBINS_CURVE_H
#define DUBINS_CURVE_H

#define DUBINS_NONE -1
#define DUBINS_L 0
#define DUBINS_S 1
#define DUBINS_R 2
#define DUBINS_MAX 1e10

#include "curve.h"

namespace trajectory_generation
{
using DubinsMode = std::tuple<int, int, int>;
using DubinsLength = std::tuple<double, double, double>;

#define UNPACK_DUBINS_INPUTS(alpha, beta)                                                                              \
  double sin_a = sin(alpha);                                                                                           \
  double sin_b = sin(beta);                                                                                            \
  double cos_a = cos(alpha);                                                                                           \
  double cos_b = cos(beta);                                                                                            \
  double cos_a_b = cos(alpha - beta);

class Dubins : public Curve
{
public:
  /**
   * @brief Construct a new Dubins generation object
   * @param step        Simulation or interpolation size (default: 0.1)
   * @param max_curv    The maximum curvature of the curve (default: 0.25)
   */
  Dubins();
  Dubins(double step, double max_curv);

  /**
   * @brief Destroy the Dubins generation object
   */
  ~Dubins();

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
   * @brief Left-Straight-Left generation mode.
   * @param alpha   Initial pose of (0, 0, alpha)
   * @param beta    Goal pose of (dist, 0, beta)
   * @param dist    Goal pose of (dist, 0, beta)
   * @param length  Moving lenght of segments (t, s, p)
   * @param mode    Motion mode
   */
  void LSL(double alpha, double beta, double dist, DubinsLength& length, DubinsMode& mode);

  /**
   * @brief Right-Straight-Right generation mode.
   * @param alpha   Initial pose of (0, 0, alpha)
   * @param beta    Goal pose of (dist, 0, beta)
   * @param dist    Goal pose of (dist, 0, beta)
   * @param length  Moving lenght of segments (t, s, p)
   * @param mode    Motion mode
   */
  void RSR(double alpha, double beta, double dist, DubinsLength& length, DubinsMode& mode);

  /**
   * @brief Left-Straight-Right generation mode.
   * @param alpha   Initial pose of (0, 0, alpha)
   * @param beta    Goal pose of (dist, 0, beta)
   * @param dist    Goal pose of (dist, 0, beta)
   * @param length  Moving lenght of segments (t, s, p)
   * @param mode    Motion mode
   */
  void LSR(double alpha, double beta, double dist, DubinsLength& length, DubinsMode& mode);

  /**
   * @brief Right-Straight-Left generation mode.
   * @param alpha   Initial pose of (0, 0, alpha)
   * @param beta    Goal pose of (dist, 0, beta)
   * @param dist    Goal pose of (dist, 0, beta)
   * @param length  Moving lenght of segments (t, s, p)
   * @param mode    Motion mode
   */
  void RSL(double alpha, double beta, double dist, DubinsLength& length, DubinsMode& mode);

  /**
   * @brief Right-Left-Right generation mode.
   * @param alpha   Initial pose of (0, 0, alpha)
   * @param beta    Goal pose of (dist, 0, beta)
   * @param dist    Goal pose of (dist, 0, beta)
   * @param length  Moving lenght of segments (t, s, p)
   * @param mode    Motion mode
   */
  void RLR(double alpha, double beta, double dist, DubinsLength& length, DubinsMode& mode);

  /**
   * @brief Left-Right-Left generation mode.
   * @param alpha   Initial pose of (0, 0, alpha)
   * @param beta    Goal pose of (dist, 0, beta)
   * @param dist    Goal pose of (dist, 0, beta)
   * @param length  Moving lenght of segments (t, s, p)
   * @param mode    Motion mode
   */
  void LRL(double alpha, double beta, double dist, DubinsLength& length, DubinsMode& mode);

  /**
   * @brief Planning path interpolation.
   * @param mode      motion, i.e., DUBINS_L, DUBINS_S, DUBINS_R
   * @param length    Single step motion path length
   * @param init_pose Initial pose (x, y, yaw)
   * @return new_pose	New pose (new_x, new_y, new_yaw) after moving
   */
  Pose2d interpolate(int mode, double length, Pose2d init_pose);

  /**
   * @brief Generate the path.
   * @param start Initial pose (x, y, yaw)
   * @param goal  Target pose (x, y, yaw)
   * @return path The smoothed trajectory points
   */
  Points2d generation(Pose2d start, Pose2d goal);

  /**
   * @brief Configure the maximum curvature.
   * @param max_curv  the maximum curvature
   */
  void setMaxCurv(double max_curv);

protected:
  /**
   * @brief Update the best motion mode.
   * @param length      the current motion length
   * @param mode        the current motion mode
   * @param best_length the best motion length so far
   * @param best_mode   the best motion mode so far
   * @param best_cost   the best motion cost so far
   */
  void _update(DubinsLength length, DubinsMode mode, DubinsLength& best_length, DubinsMode& best_mode,
               double& best_cost);

protected:
  double max_curv_;  // The maximum curvature of the curve
};

typedef void (Dubins::*DubinsSolver)(double, double, double, DubinsLength&, DubinsMode&);
extern DubinsSolver dubins_solvers[];
}  // namespace trajectory_generation

#endif