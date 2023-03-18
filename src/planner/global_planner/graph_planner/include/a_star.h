/***********************************************************
 *
 * @file: a_star.h
 * @breif: Contains the A* (dijkstra and GBFS) planner class
 * @author: Yang Haodong
 * @update: 2022-10-27
 * @version: 1.1
 *
 * Copyright (c) 2022， Yang Haodong
 * All rights reserved.
 * --------------------------------------------------------
 *
 **********************************************************/
#ifndef A_STAR_H
#define A_STAR_H

#include "global_planner.h"

namespace a_star_planner
{
/**
 * @brief Class for objects that plan using the A* algorithm
 */
class AStar : public global_planner::GlobalPlanner
{
public:
  /**
   * @brief Construct a new AStar object
   * @param nx          pixel number in costmap x direction
   * @param ny          pixel number in costmap y direction
   * @param resolution  costmap resolution
   * @param dijkstra    using diksktra implementation
   * @param gbfs        using gbfs implementation
   */
  AStar(int nx, int ny, double resolution, bool dijkstra = false, bool gbfs = false);

  /**
   * @brief A* implementation
   * @param gloal_costmap global costmap
   * @param start         start node
   * @param goal          goal node
   * @param path          optimal path consists of Node
   * @param expand        containing the node been search during the process
   * @return  true if path found, else false
   */
  bool plan(const unsigned char* gloal_costmap, const Node& start, const Node& goal, std::vector<Node>& path,
            std::vector<Node>& expand);

  /**
   * @brief Get the Heuristics
   * @param node  current node
   * @param goal  goal node
   * @return  heuristics
   */
  double getHeuristics(const Node& node, const Node& goal);

private:
  bool is_dijkstra_;  // using diksktra
  bool is_gbfs_;      // using greedy best first search(GBFS)
};
}  // namespace a_star_planner
#endif
