/**
 * *********************************************************
 *
 * @file: lazy_theta_star.h
 * @brief: Contains the lazy Theta* planner class
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
#ifndef LAZY_THETA_STAR_H
#define LAZY_THETA_STAR_H

#include "theta_star.h"

namespace global_planner
{
/**
 * @brief Class for objects that plan using the lazy Theta* algorithm
 */
class LazyThetaStar : public ThetaStar
{
public:
  /**
   * @brief Construct a new LaztThetaStar object
   * @param costmap the environment for path planning
   */
  LazyThetaStar(costmap_2d::Costmap2D* costmap);

  /**
   * @brief Lazy Theta* implementation
   * @param start  start node
   * @param goal   goal node
   * @param path   optimal path consists of Node
   * @param expand containing the node been search during the process
   * @return  true if path found, else false
   */
  bool plan(const Node& start, const Node& goal, std::vector<Node>& path, std::vector<Node>& expand);

protected:
  /**
   * @brief check if the parent of vertex need to be updated. if so, update it
   * @param node
   */
  void _setVertex(Node& node);

private:
  std::unordered_map<int, Node> closed_list_;  // closed list
  std::vector<Node> motions_;                  // possible motions
};
}  // namespace global_planner
#endif
