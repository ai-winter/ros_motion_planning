/**
 * *********************************************************
 *
 * @file: theta_star.h
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
#ifndef THETA_STAR_H
#define THETA_STAR_H

#include "global_planner.h"

namespace global_planner
{
/**
 * @brief Class for objects that plan using the Theta* algorithm
 */
class ThetaStar : public GlobalPlanner
{
public:
  /**
   * @brief Construct a new ThetaStar object
   * @param costmap the environment for path planning
   */
  ThetaStar(costmap_2d::Costmap2D* costmap);

  /**
   * @brief Theta* implementation
   * @param start  start node
   * @param goal   goal node
   * @param path   optimal path consists of Node
   * @param expand containing the node been search during the process
   * @return true if path found, else false
   */
  bool plan(const Node& start, const Node& goal, std::vector<Node>& path, std::vector<Node>& expand);

protected:
  /**
   * @brief Bresenham algorithm to check if there is any obstacle between node1 and node2
   * @param node1 node1
   * @param node2 node2
   * @return true if no obstacle, else false
   */
  bool _lineOfSight(const Node& node1, const Node& node2);
};
}  // namespace global_planner
#endif
