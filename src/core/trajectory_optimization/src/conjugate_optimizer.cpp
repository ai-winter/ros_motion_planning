/***********************************************************
 *
 * @file: conjugate_optimizer.cpp
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
#include <iostream>
#include "conjugate_optimizer.h"

namespace trajectory_optimization
{
CGOptimizer::CGOptimizer(int max_iter, double alpha, double obs_dist_max, double k_max, double w_obstacle,
                         double w_smooth, double w_curvature)
  : Optimizer(max_iter)
  , alpha_(alpha)
  , obs_dist_max_(obs_dist_max)
  , k_max_(k_max)
  , w_obstacle_(w_obstacle)
  , w_smooth_(w_smooth)
  , w_curvature_(w_curvature)
{
}

CGOptimizer::~CGOptimizer()
{
}

bool CGOptimizer::run(const Points2d path_i, Points2d& path_o)
{
  path_o = path_i;

  int iter = 0;
  while (iter < max_iter_)
  {
    // choose the first three nodes of the path
    for (int i = 2; i < path_i.size() - 2; ++i)
    {
      Eigen::Vector2d xi_c2(path_i[i - 2].first, path_i[i - 2].second);
      Eigen::Vector2d xi_c1(path_i[i - 1].first, path_i[i - 1].second);
      Eigen::Vector2d xi(path_i[i].first, path_i[i].second);
      Eigen::Vector2d xi_p1(path_i[i + 1].first, path_i[i + 1].second);
      Eigen::Vector2d xi_p2(path_i[i + 2].first, path_i[i + 2].second);

      Eigen::Vector2d correction;

      correction = correction - calObstacleTerm(xi);
      if (!_insideMap((xi + correction)[0], (xi + correction)[1]))
        continue;

      correction = correction - calSmoothTerm(xi_c2, xi_c1, xi, xi_p1, xi_p2);
      if (!_insideMap((xi + correction)[0], (xi + correction)[1]))
        continue;

      correction = correction - calCurvatureTerm(xi_c1, xi, xi_p1);
      if (!_insideMap((xi + correction)[0], (xi + correction)[1]))
        continue;

      xi = xi + alpha_ * correction / (w_obstacle_ + w_smooth_ + w_curvature_);
      path_o[i] = std::make_pair(xi[0], xi[1]);
    }

    iter++;
  }

  return true;
}

Eigen::Vector2d CGOptimizer::calObstacleTerm(const Eigen::Vector2d xi)
{
  Eigen::Vector2d gradient(0, 0);

  // the distance to the closest obstacle from the current node
  double obs_dist = voronoi_.getDistance(xi[0], xi[1]);

  // the vector determining where the obstacle is
  int x = static_cast<int>(xi[0]);
  int y = static_cast<int>(xi[1]);
  if (_insideMap(x, y))
  {
    Eigen::Vector2d obs_vec(xi[0] - voronoi_.getObstacleX(x, y), xi[1] - voronoi_.getObstacleY(x, y));

    // the closest obstacle is closer than desired correct the path for that
    if (obs_dist < obs_dist_max_)
      gradient = w_obstacle_ * 2 * (obs_dist - obs_dist_max_) * obs_vec / obs_dist;
  }
  return gradient;
}

Eigen::Vector2d CGOptimizer::calSmoothTerm(const Eigen::Vector2d xi_c2, const Eigen::Vector2d xi_c1,
                                           const Eigen::Vector2d xi, const Eigen::Vector2d xi_p1,
                                           const Eigen::Vector2d xi_p2)
{
  return w_smooth_ * (xi_c2 - 4 * xi_c1 + 6 * xi - 4 * xi_p1 + xi_p2);
}

Eigen::Vector2d CGOptimizer::calCurvatureTerm(const Eigen::Vector2d xi_c1, const Eigen::Vector2d xi,
                                              const Eigen::Vector2d xi_p1)
{
  Eigen::Vector2d gradient(0, 0);

  Eigen::Vector2d d_xi = xi - xi_c1;
  Eigen::Vector2d d_xi_p1 = xi_p1 - xi;

  // orthogonal complements vector
  Eigen::Vector2d p1, p2;
  auto ort = [](Eigen::Vector2d a, Eigen::Vector2d b) { return a - b * a.dot(b) / std::pow(b.norm(), 2); };

  // the distance of the vectors
  double abs_dxi = d_xi.norm();
  double abs_dxi_p1 = d_xi_p1.norm();

  if (abs_dxi > 0 && abs_dxi_p1 > 0)
  {
    // the angular change at the node
    double d_phi = acos(helper::clamp(d_xi.dot(d_xi_p1) / (abs_dxi * abs_dxi_p1), -1.0, 1.0));
    double k = d_phi / abs_dxi;

    // if the curvature is smaller then the maximum do nothing
    if (k > k_max_)
    {
      double u = 1 / abs_dxi / std::sqrt(1 - std::pow(std::cos(d_phi), 2));

      // calculate the p1 and p2 terms
      p1 = ort(xi, -xi_p1) / (abs_dxi * abs_dxi_p1);
      p2 = -ort(xi_p1, xi) / (abs_dxi * abs_dxi_p1);

      // calculate the last terms
      double s = d_phi / (abs_dxi * abs_dxi);
      Eigen::Vector2d ones(1, 1);
      Eigen::Vector2d ki = u * (-p1 - p2) - (s * ones);
      Eigen::Vector2d ki_c1 = u * p2 - (s * ones);
      Eigen::Vector2d ki_p1 = u * p1;

      // calculate the gradient
      gradient = w_curvature_ * (0.25 * ki_c1 + 0.5 * ki + 0.25 * ki_p1);

      if (std::isnan(gradient[0]) || std::isnan(gradient[1]))
      {
        gradient = Eigen::Vector2d(0, 0);
        std::cout << "nan values in curvature term" << std::endl;
      }
    }
  }
  else
    std::cout << "abs values not larger than 0" << std::endl;

  return gradient;
}



bool CGOptimizer::_insideMap(int x, int y)
{
  if (x < voronoi_.getSizeX() && x >= 0 && y < voronoi_.getSizeY() && y >= 0)
    return true;
  return false;
}

bool CGOptimizer::_isCuspPoint(DirPoints2d path, int i)
{
  bool dir_c2 = std::get<2>(path[i - 2]) > 0 ? true : false;
  bool dir_c1 = std::get<2>(path[i - 1]) > 0 ? true : false;
  bool dir = std::get<2>(path[i]) > 0 ? true : false;
  bool dir_p1 = std::get<2>(path[i + 1]) > 0 ? true : false;

  if (dir_c2 != dir_c1 || dir_c1 != dir || dir != dir_p1)
    return true;

  return false;
}

}  // namespace trajectory_optimization