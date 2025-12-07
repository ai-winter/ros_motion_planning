/**
 * *********************************************************
 *
 * @file: astar_planner.h
 * @brief: Contains the A* (dijkstra and GBFS) planner class
 * @author: Yang Haodong
 * @date: 2023-12-12
 * @version: 1.2
 *
 * Copyright (c) 2024, Yang Haodong.
 * All rights reserved.
 *
 * --------------------------------------------------------
 *
 * ********************************************************
 */
#ifndef RMP_PATH_PLANNER_GRAPH_PLANNER_A_STAR_H
#define RMP_PATH_PLANNER_GRAPH_PLANNER_A_STAR_H

#include "path_planner/path_planner.h"

namespace rmp {
namespace path_planner {
/**
 * @brief Class for objects that plan using the A* algorithm
 */
class AStarPathPlanner : public PathPlanner {
public:
  /**
   * @brief Construct a new AStar object
   * @param costmap   the environment for path planning
   * @param dijkstra   using diksktra implementation
   * @param gbfs       using gbfs implementation
   */
  AStarPathPlanner(costmap_2d::Costmap2DROS* costmap_ros, bool dijkstra = false,
                   bool gbfs = false);

  /**
   * @brief A* implementation
   * @param start          start node
   * @param goal           goal node
   * @param path           The resulting path in (x, y, theta)
   * @param expand         containing the node been search during the process
   * @return true if path found, else false
   */
  bool plan(const common::geometry::Point3d& start, const common::geometry::Point3d& goal,
            common::geometry::Points3d* path, common::geometry::Points3d* expand);

private:
  bool is_dijkstra_;  // using diksktra
  bool is_gbfs_;      // using greedy best first search(GBFS)

  using Node = rmp::common::structure::Node<int>;
  static std::vector<Node> motions_;
};
}  // namespace path_planner
}  // namespace rmp
#endif
