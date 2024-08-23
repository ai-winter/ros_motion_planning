/**
 * *********************************************************
 *
 * @file: s_theta_star.h
 * @brief: Contains the S-Theta* planner class
 * @author: Wu Maojia
 * @date: 2024-3-9
 * @version: 1.0
 *
 * Copyright (c) 2024, Wu Maojia, Yang Haodong.
 * All rights reserved.
 *
 * --------------------------------------------------------
 *
 * ********************************************************
 */
#ifndef S_THETA_STAR_H
#define S_THETA_STAR_H

#include <vector>
#include <queue>
#include "theta_star.h"

namespace global_planner
{
/**
 * @brief Class for objects that plan using the S-Theta* algorithm
 */
class SThetaStar : public ThetaStar
{
public:
  /**
   * @brief Construct a new SThetaStar object
   * @param costmap   the environment for path planning
   */
  SThetaStar(costmap_2d::Costmap2D* costmap);

  /**
   * @brief S-Theta* implementation
   * @param start         start node
   * @param goal          goal node
   * @param path          optimal path consists of Node
   * @param expand        containing the node been search during the process
   * @return  true if path found, else false
   */
  bool plan(const Node& start, const Node& goal, std::vector<Node>& path, std::vector<Node>& expand);

protected:
  /**
   * @brief update the g value of child node
   * @param parent
   * @param child
   * @param alpha
   */
  void _updateVertex(const Node& parent, Node& child, const double alpha);

  /**
   * @brief Get the deviation cost
   * @param parent
   * @param child
   * @param goal
   * @return deviation cost
   */
  double _alpha(const Node& parent, const Node& child, const Node& goal);

  /**
   * @brief Get the Euclidean distance between two nodes
   * @param node  current node
   * @param goal  goal node
   * @return  Euclidean distance
   */
  double _getDistance(const Node& node, const Node& goal);

private:
  const double pi_ = std::acos(-1);  // pi
};
}  // namespace global_planner
#endif
