/**
 * *********************************************************
 *
 * @file: theta_star_planner.h
 * @brief: Contains the Theta* planner class
 * @author: Wu Maojia, Yang Haodong
 * @date: 2023-10-01
 * @version: 1.3
 *
 * Copyright (c) 2024, Wu Maojia, Yang Haodong.
 * All rights reserved.
 *
 * --------------------------------------------------------
 *
 * ********************************************************
 */
#ifndef RMP_PATH_PLANNER_GRAPH_PLANNER_THETA_STAR_H_
#define RMP_PATH_PLANNER_GRAPH_PLANNER_THETA_STAR_H_

#include <vector>

#include "common/geometry/point.h"
#include "path_planner/path_planner.h"

namespace rmp
{
namespace path_planner
{
/**
 * @brief Class for objects that plan using the Theta* algorithm
 */
class ThetaStarPathPlanner : public PathPlanner
{
private:
  using Node = rmp::common::structure::Node<int>;

public:
  /**
   * @brief Construct a new ThetaStar object
   * @param costmap   the environment for path planning
   * @param obstacle_factor obstacle factor(greater means obstacles)
   */
  ThetaStarPathPlanner(costmap_2d::Costmap2DROS* costmap_ros, double obstacle_factor = 1.0);

  /**
   * @brief Theta* implementation
   * @param start         start node
   * @param goal          goal node
   * @param path          optimal path consists of Node
   * @param expand        containing the node been search during the process
   * @return  true if path found, else false
   */
  bool plan(const Point3d& start, const Point3d& goal, Points3d& path, Points3d& expand);

protected:
  /**
   * @brief update the g value of child node
   * @param parent
   * @param child
   */
  void _updateVertex(const Node& parent, Node& child);

private:
  const std::vector<Node> motions = {
    { 0, 1, 1.0 },          { 1, 0, 1.0 },           { 0, -1, 1.0 },          { -1, 0, 1.0 },
    { 1, 1, std::sqrt(2) }, { 1, -1, std::sqrt(2) }, { -1, 1, std::sqrt(2) }, { -1, -1, std::sqrt(2) },
  };
};
}  // namespace path_planner
}  // namespace rmp
#endif
