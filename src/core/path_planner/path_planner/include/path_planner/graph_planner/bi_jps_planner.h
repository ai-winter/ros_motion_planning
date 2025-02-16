/**
 * *********************************************************
 *
 * @file: bi_jps_planner.h
 * @brief: Contains the Bi-Jump Point Search(bi-JPS) planner class
 * @author: Yang Haodong
 * @date: 2024-09-30
 * @version: 1.0
 *
 * Copyright (c) 2024, Yang Haodong.
 * All rights reserved.
 *
 * --------------------------------------------------------
 *
 * ********************************************************
 */
#ifndef RMP_PATH_PLANNER_GRAPH_PLANNER_BI_JUMP_POINT_SEARCH_H_
#define RMP_PATH_PLANNER_GRAPH_PLANNER_BI_JUMP_POINT_SEARCH_H_

#include <array>

#include "path_planner/graph_planner/jps_planner.h"

namespace rmp
{
namespace path_planner
{
/**
 * @brief lass for objects that plan using the Bi-Jump Point Search(BiJPSPlanner) algorithm
 */
class BiJPSPathPlanner : public JPSPathPlanner
{
public:
  /**
   * @brief Constructor
   * @param costmap   the environment for path planning
   * @param obstacle_factor obstacle factor(greater means obstacles)
   */
  BiJPSPathPlanner(costmap_2d::Costmap2DROS* costmap_ros, double obstacle_factor = 1.0);

  /**
   * @brief Bi-Jump Point Search(bi-JPS) implementation
   * @param start  start node
   * @param goal   goal node
   * @param expand containing the node been search during the process
   * @return tuple contatining a bool as to whether a path was found, and the path
   */
  bool plan(const Point3d& start, const Point3d& goal, Points3d& path, Points3d& expand);

private:
  double last_len_;     // length of last planning path
  Points3d last_path_;  // last planning path
  JNode start_, goal_;  // start and goal node
  std::priority_queue<JNode, std::vector<JNode>, JNode::compare_cost> f_open_list_, b_open_list_;  // open list
  std::unordered_map<int, JNode> f_closed_list_, b_closed_list_;                                   // closed list
};
}  // namespace path_planner
}  // namespace rmp
#endif  // JUMP_POINT_SEARCH_H
