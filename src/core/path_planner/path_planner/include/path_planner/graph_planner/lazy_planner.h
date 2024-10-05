/**
 * *********************************************************
 *
 * @file: lazy_planner.h
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
#ifndef RMP_PATH_PLANNER_GRAPH_PLANNER_LAZY_H
#define RMP_PATH_PLANNER_GRAPH_PLANNER_LAZY_H

#include "path_planner/path_planner.h"

namespace rmp
{
namespace path_planner
{
/**
 * @brief Class for lazy planner
 */
class LazyPathPlanner : public PathPlanner
{
public:
  /**
   * @brief Construct a new Lazy planner
   * @param costmap   the environment for path planning
   */
  LazyPathPlanner(costmap_2d::Costmap2DROS* costmap_ros);

  /**
   * @brief Lazy implementation
   * @param start          start node
   * @param goal           goal node
   * @param path           optimal path consists of Node
   * @param expand         containing the node been search during the process
   * @return true if path found, else false
   */
  bool plan(const Point3d& start, const Point3d& goal, Points3d& path, Points3d& expand);
};
}  // namespace path_planner
}  // namespace rmp
#endif
