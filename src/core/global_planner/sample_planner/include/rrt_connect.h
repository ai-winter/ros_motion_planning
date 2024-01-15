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
   * @brief  Constructor
   * @param   nx          pixel number in costmap x direction
   * @param   ny          pixel number in costmap y direction
   * @param   resolution  costmap resolution
   * @param   sample_num  andom sample points
   * @param   max_dist    max distance between sample points
   */
  RRTConnect(int nx, int ny, double resolution, int sample_num, double max_dist);

  /**
   * @brief RRT implementation
   * @param costs     costmap
   * @param start     start node
   * @param goal      goal node
   * @param expand    containing the node been search during the process
   * @return tuple contatining a bool as to whether a path was found, and the path
   */
  bool plan(const unsigned char* global_costmap, const Node& start, const Node& goal, std::vector<Node>& path,
            std::vector<Node>& expand);

protected:
  /**
   * @brief convert closed list to path
   * @param boundary  connected node that the boudary of forward and backward
   * @return ector containing path nodes
   */
  std::vector<Node> _convertClosedListToPath(const Node& boundary);

protected:
  // Sampled list forward
  std::unordered_map<int, Node> sample_list_f_;
  // Sampled list backward
  std::unordered_map<int, Node> sample_list_b_;
};
}  // namespace global_planner

#endif