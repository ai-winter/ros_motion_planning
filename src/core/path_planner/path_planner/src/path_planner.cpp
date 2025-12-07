/**
 * *********************************************************
 *
 * @file: global_PathPlanner.cpp
 * @brief: Contains the abstract global PathPlanner class
 * @author: Yang Haodong
 * @date: 2023-10-24
 * @version: 2.1
 *
 * Copyright (c) 2024, Yang Haodong.
 * All rights reserved.
 *
 * --------------------------------------------------------
 *
 * ********************************************************
 */
#include <costmap_2d/cost_values.h>

#include "common/math/math_helper.h"
#include "path_planner/path_planner.h"
#include "system_config/system_config.h"

using namespace rmp::common::math;
using namespace rmp::common::geometry;

namespace rmp {
namespace path_planner {
/**
 * @brief Construct a new Global PathPlanner object
 * @param costmap_ros     the environment for path planning
 */
PathPlanner::PathPlanner(costmap_2d::Costmap2DROS* costmap_ros)
  : costmap_ros_(costmap_ros)
  , nx_(static_cast<int>(costmap_ros->getCostmap()->getSizeInCellsX()))
  , ny_(static_cast<int>(costmap_ros->getCostmap()->getSizeInCellsY()))
  , map_size_(nx_ * ny_)
  , costmap_(costmap_ros->getCostmap())
  , config_(system_config::SystemConfigPtr::Instance()->configure().path_planner()) {
  collision_checker_ = std::make_shared<CollisionChecker>(
      costmap_ros, config_.obstacle_inflation_factor());
}

/**
 * @brief get the planner configure parameters
 */
const pb::path_planner::PathPlanner& PathPlanner::config() const {
  return config_;
}

/**
 * @brief get the costmap
 * @return costmap costmap2d pointer
 */
costmap_2d::Costmap2D* PathPlanner::getCostMap() const {
  return costmap_;
}

/**
 * @brief get the size of costmap
 * @return map_size the size of costmap
 */
int PathPlanner::getMapSize() const {
  return map_size_;
}

/**
 * @brief Transform from grid map(x, y) to grid index(i)
 * @param x grid map x
 * @param y grid map y
 * @return index
 */
int PathPlanner::grid2Index(int x, int y) {
  return x + nx_ * y;
}

/**
 * @brief Transform from grid index(i) to grid map(x, y)
 * @param i grid index i
 * @param x grid map x
 * @param y grid map y
 */
void PathPlanner::index2Grid(int i, int& x, int& y) {
  x = static_cast<int>(i % nx_);
  y = static_cast<int>(i / nx_);
}

/**
 * @brief Tranform from world map(x, y) to costmap(x, y)
 * @param mx costmap x
 * @param my costmap y
 * @param wx world map x
 * @param wy world map y
 * @return true if successfull, else false
 */
bool PathPlanner::world2Map(double wx, double wy, double& mx, double& my) {
  if (wx < costmap_->getOriginX() || wy < costmap_->getOriginY())
    return false;

  mx = (wx - costmap_->getOriginX()) / costmap_->getResolution();
  my = (wy - costmap_->getOriginY()) / costmap_->getResolution();

  if (mx < nx_ && my < ny_)
    return true;

  return false;
}

/**
 * @brief Tranform from costmap(x, y) to world map(x, y)
 * @param mx costmap x
 * @param my costmap y
 * @param wx world map x
 * @param wy world map y
 */
void PathPlanner::map2World(double mx, double my, double& wx, double& wy) {
  wx = costmap_->getOriginX() + (mx + 0.5) * costmap_->getResolution();
  wy = costmap_->getOriginY() + (my + 0.5) * costmap_->getResolution();
}

/**
 * @brief Inflate the boundary of costmap into obstacles to prevent cross planning
 */
void PathPlanner::outlineMap() {
  auto pc = costmap_->getCharMap();
  for (int i = 0; i < nx_; i++)
    *pc++ = costmap_2d::LETHAL_OBSTACLE;
  pc = costmap_->getCharMap() + (ny_ - 1) * nx_;
  for (int i = 0; i < nx_; i++)
    *pc++ = costmap_2d::LETHAL_OBSTACLE;
  pc = costmap_->getCharMap();
  for (int i = 0; i < ny_; i++, pc += nx_)
    *pc = costmap_2d::LETHAL_OBSTACLE;
  pc = costmap_->getCharMap() + nx_ - 1;
  for (int i = 0; i < ny_; i++, pc += nx_)
    *pc = costmap_2d::LETHAL_OBSTACLE;
}

/**
 * @brief Check the validity of (wx, wy)
 * @param wx world map x
 * @param wy world map y
 * @param mx costmap x
 * @param my costmap y
 * @return flag true if the position is valid
 */
bool PathPlanner::validityCheck(double wx, double wy, double& mx, double& my) {
  if (!world2Map(wx, wy, mx, my)) {
    R_WARN << "The robot's position is off the global costmap. Planning will always "
              "fail, are you sure the robot "
              "has been properly localized?";
    return false;
  }
  return true;
}

/**
 * @brief Resample the given path based on a specified sampling ratio
 * @param path           the original path to be resampled
 * @param path_resample  the resulting resampled path
 * @param sample_ratio   the ratio used to determine the sampling intervals
 * @return true if the resampling is successful, false otherwise
 */
bool PathPlanner::resample(const Points3d& path, Points3d* path_resample,
                           double sample_ratio) {
  int path_size = static_cast<int>(path.size());
  if (path_size <= 1) {
    *path_resample = path;
    return true;
  }

  // 1) Uniform Resampling of (x, y) Coordinates Along Path Length
  // 1.1） Computation of Cumulative Distance and Sampling Distance List
  std::vector<double> cumulative_dist{ 0.0 };
  for (size_t i = 1; i < path.size(); ++i) {
    const double dx = path[i].x() - path[i - 1].x();
    const double dy = path[i].y() - path[i - 1].y();
    cumulative_dist.push_back(cumulative_dist.back() + std::hypot(dx, dy));
  }
  double total_length = cumulative_dist.back();

  if (total_length < kMathEpsilon) {
    *path_resample = path;
    return true;
  }

  std::vector<double> sample_dist_list;
  for (double s = 0.0; s <= total_length - kMathEpsilon; s += sample_ratio) {
    sample_dist_list.push_back(s);
  }
  if (sample_dist_list.empty() ||
      std::fabs(sample_dist_list.back() - total_length) > kMathEpsilon) {
    sample_dist_list.push_back(total_length);
  }

  // 1.2) Interpolation of Original Path Based on Sampling Distances
  path_resample->clear();
  path_resample->reserve(sample_dist_list.size());
  for (double s : sample_dist_list) {
    auto it = std::upper_bound(cumulative_dist.begin(), cumulative_dist.end(), s);
    size_t idx = std::distance(cumulative_dist.begin(), it) - 1;

    if (idx >= path.size() - 1) {
      path_resample->push_back(path.back());
      continue;
    }

    double seg_start = cumulative_dist[idx];
    double seg_end = cumulative_dist[idx + 1];
    double seg_length = seg_end - seg_start;

    if (seg_length < kMathEpsilon) {
      path_resample->push_back(path[idx]);
      continue;
    }

    double t = (s - seg_start) / seg_length;
    double x = path[idx].x() + t * (path[idx + 1].x() - path[idx].x());
    double y = path[idx].y() + t * (path[idx + 1].y() - path[idx].y());
    path_resample->emplace_back(x, y);
  }

  // 2) Heading Angle Assignment for Interpolated Points Based on Tangent Direction
  auto prelast_dir = Vec2d(std::numeric_limits<double>::infinity(),
                           std::numeric_limits<double>::infinity());
  for (size_t i = 1; i <= path_resample->size(); ++i) {
    if ((!std::isinf(prelast_dir.x())) && (!std::isinf(prelast_dir.y()))) {
      auto last_dir = Vec2d();
      auto prelast_vec =
          Vec2d(path_resample->at(i - 2).x(), path_resample->at(i - 2).y());
      auto last_vec = Vec2d(path_resample->at(i - 1).x(), path_resample->at(i - 1).y());

      // compute orientation of last point
      if (i < path_resample->size()) {
        auto current_vec = Vec2d(path_resample->at(i).x(), path_resample->at(i).y());
        auto tangent_dir =
            rmp::common::math::tangentDir(prelast_vec, last_vec, current_vec, false);
        last_dir = tangent_dir.innerProd(current_vec - last_vec) >= 0 ? tangent_dir :
                                                                        -tangent_dir;
        last_dir.normalize();
      } else {
        last_dir = Vec2d(std::cos(path.back().theta()), std::sin(path.back().theta()));
      }

      path_resample->at(i - 1).setTheta(last_dir.angle());
      prelast_dir = last_dir;
    } else {
      // start pose
      auto dir = Vec2d(std::cos(path[0].theta()), std::sin(path[0].theta()));
      dir.normalize();
      path_resample->front().setTheta(dir.angle());
      prelast_dir = dir;
    }
  }

  return true;
}
}  // namespace path_planner
}  // namespace rmp