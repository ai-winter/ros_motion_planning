/**
 * *********************************************************
 *
 * @file: footprint_collision_checker.hpp
 * @brief: Footprint collision checker used in MPPI controller.
 * @author: Zhanyu Guo
 * @date: 2026-02-21
 * @version: 1.0
 *
 * Copyright (c) 2026, Zhanyu Guo.
 * All rights reserved.
 *
 * --------------------------------------------------------
 *
 * ********************************************************
 */

#ifndef RMP_CONTROLLER_MPPI_FOOTPRINT_COLLISION_CHECKER_HPP_
#define RMP_CONTROLLER_MPPI_FOOTPRINT_COLLISION_CHECKER_HPP_

#include <string>
#include <vector>
#include <memory>
#include <algorithm>

#include <geometry_msgs/PoseStamped.h>
#include <costmap_2d/costmap_2d.h>

namespace costmap_2d {

typedef std::vector<geometry_msgs::Point> Footprint;

/**
 * @class FootprintCollisionChecker
 * @brief Checker for collision with a footprint on a costmap
 */
template <typename CostmapT>
class FootprintCollisionChecker {
public:
  /**
   * @brief A constructor.
   */
  FootprintCollisionChecker();
  /**
   * @brief A constructor.
   */
  explicit FootprintCollisionChecker(CostmapT costmap);
  /**
   * @brief Find the footprint cost in oriented footprint
   */
  double footprintCost(const Footprint& footprint);
  /**
   * @brief Find the footprint cost a a post with an unoriented footprint
   */
  double footprintCostAtPose(double x, double y, double theta,
                             const Footprint& footprint);
  /**
   * @brief Get the cost for a line segment
   */
  double lineCost(int x0, int x1, int y0, int y1) const;
  /**
   * @brief Get the map coordinates from a world point
   */
  bool worldToMap(double wx, double wy, unsigned int& mx, unsigned int& my);
  /**
   * @brief Get the cost of a point
   */
  double pointCost(int x, int y) const;
  /**
   * @brief Set the current costmap object to use for collision detection
   */
  void setCostmap(CostmapT costmap);
  /**
   * @brief Get the current costmap object
   */
  CostmapT getCostmap() {
    return costmap_;
  }

protected:
  CostmapT costmap_;
};

}  // namespace costmap_2d

#endif  // RMP_CONTROLLER_MPPI_FOOTPRINT_COLLISION_CHECKER_HPP_