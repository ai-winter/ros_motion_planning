/**
 * *********************************************************
 *
 * @file: jump_point_search.h
 * @brief: Contains the Jump Point Search(JPS) planner class
 * @author: Yang Haodong
 * @date: 2023-12-14
 * @version: 1.1
 *
 * Copyright (c) 2024, Yang Haodong.
 * All rights reserved.
 *
 * --------------------------------------------------------
 *
 * ********************************************************
 */
#ifndef JUMP_POINT_SEARCH_H
#define JUMP_POINT_SEARCH_H

#include "global_planner.h"

namespace global_planner
{
/**
 * @brief Class for objects that plan using the Jump Point Search(JPS) algorithm
 */
class JumpPointSearch : public GlobalPlanner
{
public:
  /**
   * @brief Constructor
   * @param costmap the environment for path planning
   */
  JumpPointSearch(costmap_2d::Costmap2D* costmap);

  /**
   * @brief Jump Point Search(JPS) implementation
   * @param start  start node
   * @param goal   goal node
   * @param expand containing the node been search during the process
   * @return tuple contatining a bool as to whether a path was found, and the path
   */
  bool plan(const Node& start, const Node& goal, std::vector<Node>& path, std::vector<Node>& expand);

  /**
   * @brief Calculate jump node recursively
   * @param point  current node
   * @param motion the motion that current node executes
   * @return jump node
   */
  Node jump(const Node& point, const Node& motion);

  /**
   * @brief Detect whether current node has forced neighbor or not
   * @param point  current node
   * @param motion the motion that current node executes
   * @return true if current node has forced neighbor else false
   */
  bool detectForceNeighbor(const Node& point, const Node& motion);

private:
  Node start_, goal_;  // start and goal node
};
}  // namespace global_planner
#endif
