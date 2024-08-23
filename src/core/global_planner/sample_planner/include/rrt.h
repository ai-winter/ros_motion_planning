/**
 * *********************************************************
 *
 * @file: rrt.h
 * @brief: Contains the Rapidly-Exploring Random Tree (RRT) planner class
 * @author: Yang Haodong
 * @date: 2022-10-27
 * @version: 1.0
 *
 * Copyright (c) 2024, Yang Haodong.
 * All rights reserved.
 *
 * --------------------------------------------------------
 *
 * ********************************************************
 */
#ifndef RRT_H
#define RRT_H

#include <tuple>

#include "global_planner.h"

namespace global_planner
{
/**
 * @brief Class for objects that plan using the RRT algorithm
 */
class RRT : public GlobalPlanner
{
public:
  /**
   * @brief  Constructor
   * @param   costmap   the environment for path planning
   * @param   sample_num  andom sample points
   * @param   max_dist    max distance between sample points
   */
  RRT(costmap_2d::Costmap2D* costmap, int sample_num, double max_dist);

  /**
   * @brief RRT implementation
   * @param start         start node
   * @param goal          goal node
   * @param path          optimal path consists of Node
   * @param expand        containing the node been search during the process
   * @return  true if path found, else false
   */
  bool plan(const Node& start, const Node& goal, std::vector<Node>& path, std::vector<Node>& expand);

protected:
  /**
   * @brief Regular the sample node by the nearest node in the sample list
   * @param list  samplee list
   * @param node  sample node
   * @return nearest node
   */
  Node _findNearestPoint(std::unordered_map<int, Node>& list, const Node& node);
  /**
   * @brief Check if there is any obstacle between the 2 nodes.
   * @param n1        Node 1
   * @param n2        Node 2
   * @return bool value of whether obstacle exists between nodes
   */
  bool _isAnyObstacleInPath(const Node& n1, const Node& n2);
  /**
   * @brief Generates a random node
   * @return Generated node
   */
  Node _generateRandomNode();
  /**
   * @brief Check if goal is reachable from current node
   * @param new_node Current node
   * @return bool value of whether goal is reachable from current node
   */
  bool _checkGoal(const Node& new_node);

protected:
  Node start_, goal_;                          // start and goal node copy
  std::unordered_map<int, Node> sample_list_;  // set of sample nodes
  int sample_num_;                             // max sample number
  double max_dist_;                            // max distance threshold
  double opti_sample_p_ = 0.05;                // optimized sample probability, default to 0.05
};
}  // namespace global_planner
#endif  // RRT_H
