/**
 * *********************************************************
 *
 * @file: hybrid_astar_planner.h
 * @brief: Contains the Hybrid A* planner class
 * @author: Yang Haodong
 * @date: 2024-01-03
 * @version: 1.0
 *
 * Copyright (c) 2024, Yang Haodong.
 * All rights reserved.
 *
 * --------------------------------------------------------
 *
 * ********************************************************
 */
#ifndef RMP_PATH_PLANNER_GRAPH_PLANNER_HYBRID_A_STAR_H_
#define RMP_PATH_PLANNER_GRAPH_PLANNER_HYBRID_A_STAR_H_

#include "common/geometry/curve/dubins_curve.h"

#include "path_planner/path_planner.h"
#include "path_planner/graph_planner/hybrid_astar_planner/node_hybrid.h"
#include "path_planner/graph_planner/hybrid_astar_planner/astar_framework.h"

namespace rmp
{
namespace path_planner
{
/**
 * @brief Class for objects that plan using the A* algorithm
 */
class HybridAStarPathPlanner : public PathPlanner
{
public:
  /**
   * @brief Construct a new Hybrid A* object
   * @param costmap   the environment for path planning
   * @param obstacle_factor obstacle factor(greater means obstacles)
   */
  HybridAStarPathPlanner(costmap_2d::Costmap2DROS* costmap_ros, double obstacle_factor, const HybridSearchInfo& info);

  /**
   * @brief Destory the Hybrid A* object
   */
  ~HybridAStarPathPlanner() = default;

  /**
   * @brief Hybrid A* implementation
   * @param start          start node
   * @param goal           goal node
   * @param path           optimal path consists of Node
   * @param expand         containing the node been search during the process
   * @return true if path found, else false
   */
  bool plan(const Point3d& start, const Point3d& goal, Points3d& path, Points3d& expand);

private:
  std::unique_ptr<AStarFramework<NodeHybrid>> astar_framework_;

  Point3d goal_;
  Points3d last_path_;
};
}  // namespace path_planner
}  // namespace rmp
#endif