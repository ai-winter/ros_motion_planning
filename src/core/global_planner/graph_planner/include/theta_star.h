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

#include <vector>
#include <queue>
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
   * @param nx          pixel number in costmap x direction
   * @param ny          pixel number in costmap y direction
   * @param resolution  costmap resolution
   */
  ThetaStar(int nx, int ny, double resolution);

  /**
   * @brief Theta* implementation
   * @param global_costmap global costmap
   * @param start         start node
   * @param goal          goal node
   * @param path          optimal path consists of Node
   * @param expand        containing the node been search during the process
   * @return  true if path found, else false
   */
  bool plan(const unsigned char* global_costmap, const Node& start, const Node& goal, std::vector<Node>& path,
            std::vector<Node>& expand);

protected:
  /**
   * @brief update the g value of child node
   * @param parent
   * @param child
   */
  void _updateVertex(const Node& parent, Node& child);

  /**
   * @brief Bresenham algorithm to check if there is any obstacle between parent and child
   * @param parent
   * @param child
   * @return true if no obstacle, else false
   */
  bool _lineOfSight(const Node& parent, const Node& child);

protected:
  const unsigned char* costs_;  // costmap copy
};
}  // namespace global_planner
#endif
