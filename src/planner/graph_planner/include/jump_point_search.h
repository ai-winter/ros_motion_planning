/***********************************************************
 *
 * @file: jump_point_search.h
 * @breif: Contains the Jump Point Search(JPS) planner class
 * @author: Yang Haodong
 * @update: 2022-10-27
 * @version: 1.0
 *
 * Copyright (c) 2022， Yang Haodong
 * All rights reserved.
 * --------------------------------------------------------
 *
 **********************************************************/
#ifndef JUMP_POINT_SEARCH_H
#define JUMP_POINT_SEARCH_H

#include <queue>
#include <unordered_set>
#include <ros/ros.h>

#include "global_planner.h"
#include "utils.h"

namespace jps_planner
{
/**
 * @brief lass for objects that plan using the Jump Point Search(JPS) algorithm
 */
class JumpPointSearch : public global_planner::GlobalPlanner
{
public:
  /**
   * @brief  Constructor
   * @param   nx          pixel number in costmap x direction
   * @param   ny          pixel number in costmap y direction
   * @param   resolution  costmap resolution
   */
  JumpPointSearch(int nx, int ny, double resolution);
  /**
   * @brief Jump Point Search(JPS) implementation
   * @param costs     costmap
   * @param start     start node
   * @param goal      goal node
   * @param expand    containing the node been search during the process
   * @return tuple contatining a bool as to whether a path was found, and the path
   */
  std::tuple<bool, std::vector<Node>> plan(const unsigned char* costs, const Node& start, const Node& goal,
                                           std::vector<Node>& expand);
  /**
   * @brief detect whether current node has forced neighbor or not
   * @param point     current node
   * @param motion    the motion that current node executes
   * @return true if current node has forced neighbor else false
   */
  bool detectForceNeighbor(const Node& point, const Node& motion);

  /**
   * @brief calculate jump node recursively
   * @param point     current node
   * @param motion    the motion that current node executes
   * @return jump node
   */
  Node jump(const Node& point, const Node& motion);

private:
  // start and goal node
  Node start_, goal_;
  // costmap
  const unsigned char* costs_;
};
}  // namespace jps_planner
#endif  // JUMP_POINT_SEARCH_H
