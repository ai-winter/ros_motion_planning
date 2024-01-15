/**
 * *********************************************************
 *
 * @file: a_star.h
 * @brief: Contains the A* (dijkstra and GBFS) planner class
 * @author: Yang Haodong
 * @date: 2023-12-12
 * @version: 1.2
 *
 * Copyright (c) 2024, Yang Haodong. 
 * All rights reserved.
 * 
 * --------------------------------------------------------
 *
 * ********************************************************
 */
#ifndef A_STAR_H
#define A_STAR_H

#include "global_planner.h"

namespace global_planner
{
/**
 * @brief Class for objects that plan using the A* algorithm
 */
class AStar : public GlobalPlanner
{
public:
  /**
   * @brief Construct a new AStar object
   * @param nx         pixel number in costmap x direction
   * @param ny         pixel number in costmap y direction
   * @param resolution costmap resolution
   * @param dijkstra   using diksktra implementation
   * @param gbfs       using gbfs implementation
   */
  AStar(int nx, int ny, double resolution, bool dijkstra = false, bool gbfs = false);

  /**
   * @brief A* implementation
   * @param global_costmap global costmap
   * @param start          start node
   * @param goal           goal node
   * @param path           optimal path consists of Node
   * @param expand         containing the node been search during the process
   * @return true if path found, else false
   */
  bool plan(const unsigned char* global_costmap, const Node& start, const Node& goal, std::vector<Node>& path,
            std::vector<Node>& expand);

private:
  bool is_dijkstra_;  // using diksktra
  bool is_gbfs_;      // using greedy best first search(GBFS)
};
}  // namespace global_planner
#endif
