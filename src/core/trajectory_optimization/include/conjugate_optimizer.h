/***********************************************************
 *
 * @file: conjugate_optimizer.h
 * @breif: Trajectory optimization using conjugate gradient methods
 * @author: Yang Haodong
 * @update: 2023-12-29
 * @version: 1.0
 *
 * Copyright (c) 2023, Yang Haodong
 * All rights reserved.
 * --------------------------------------------------------
 *
 **********************************************************/
#ifndef CONJUGATE_OPTIMIZER_H
#define CONJUGATE_OPTIMIZER_H

#include <Eigen/Dense>

#include "math_helper.h"
#include "optimizer.h"

namespace trajectory_optimization
{
class CGOptimizer : public Optimizer
{
public:
  CGOptimizer(int max_iter, double alpha, double obs_dist_max, double k_max, double w_obstacle,
              double w_smooth, double w_curvature);
  ~CGOptimizer();

  /**
   * @brief Running trajectory optimization
   * @param path_i path points <x, y> before optimization
   * @param path_o path points <x, y> after optimization
   * @return true if optimizes successfully, else failed
   */
  bool run(const Points2d path_i, Points2d& path_o);
  Eigen::Vector2d calObstacleTerm(const Eigen::Vector2d xi);
  Eigen::Vector2d calSmoothTerm(const Eigen::Vector2d xi_c2, const Eigen::Vector2d xi_c1, const Eigen::Vector2d xi,
                                const Eigen::Vector2d xi_p1, const Eigen::Vector2d xi_p2);
  Eigen::Vector2d calCurvatureTerm(const Eigen::Vector2d xi_c1, const Eigen::Vector2d xi, const Eigen::Vector2d xi_p1);



protected:
  bool _insideMap(int x, int y);
  bool _isCuspPoint(DirPoints2d path, int i);

private:
  double alpha_;  // learning rate
  double obs_dist_max_;
  double k_max_;
  double w_obstacle_;
  double w_smooth_;
  double w_curvature_;
};
}  // namespace trajectory_optimization

#endif