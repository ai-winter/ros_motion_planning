/**
 * *********************************************************
 *
 * @file: rrt_connect.h
 * @brief: Contains the RRT Connect planner class
 * @author: Yang Haodong
 * @date: 2023-01-18
 * @version: 1.0
 *
 * Copyright (c) 2024, Yang Haodong.
 * All rights reserved.
 *
 * --------------------------------------------------------
 *
 * ********************************************************
 */
#ifndef RRT_CONNECT_H
#define RRT_CONNECT_H

#include "rrt.h"

namespace global_planner
{
/**
 * @brief Class for objects that plan using the RRT Connect algorithm
 */
class RRTConnect : public RRT
{
public:
  /**
   * @brief Construct a new RRTConnect object
   * @param costmap    the environment for path planning
   * @param sample_num andom sample points
   * @param max_dist   max distance between sample points
   */
  RRTConnect(costmap_2d::Costmap2D* costmap, int sample_num, double max_dist);

  /**
   * @brief RRT connect implementation
   * @param start  start node
   * @param goal   goal node
   * @param expand containing the node been search during the process
   * @return  true if path found, else false
   */
  bool plan(const Node& start, const Node& goal, std::vector<Node>& path, std::vector<Node>& expand);

protected:
  /**
   * @brief Convert closed list to path
   * @param boundary connected node that the boudary of forward and backward
   * @return vector containing path nodes
   */
  std::vector<Node> _convertClosedListToPath(const Node& boundary);

protected:
  std::unordered_map<int, Node> sample_list_f_;  // sampled list forward
  std::unordered_map<int, Node> sample_list_b_;  // sampled list backward
};
}  // namespace global_planner
#endif  // RRT_CONNECT_H
