/**
 * *********************************************************
 *
 * @file: lazy_planner.cpp
 * @brief: Contains the lazy planner class
 * @author: Guo Zhanyu
 * @date: 2024-10-05
 * @version: 0.0
 *
 * Copyright (c) 2024, Guo Zhanyu.
 * All rights reserved.
 *
 * --------------------------------------------------------
 *
 * ********************************************************
 */
#include "path_planner/graph_planner/lazy_planner.h"

namespace rmp
{
namespace path_planner
{
/**
 * @brief Construct a new Lazy planner
 * @param costmap   the environment for path planning
 */
LazyPathPlanner::LazyPathPlanner(costmap_2d::Costmap2DROS* costmap_ros) : PathPlanner(costmap_ros) {};

/**
 * @brief Lazy implementation
 * @param start          start node
 * @param goal           goal node
 * @param path           optimal path consists of Node
 * @param expand         containing the node been search during the process
 * @return true if path found, else false
 */
bool LazyPathPlanner::plan(const Point3d& start, const Point3d& goal, Points3d& path, Points3d& expand)
{
  path.emplace_back(goal);
  return true;
}
}  // namespace path_planner
}  // namespace rmp
