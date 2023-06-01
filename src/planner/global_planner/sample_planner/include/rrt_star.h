/***********************************************************
 *
 * @file: rrt_star.h
 * @breif: Contains the Rapidly-Exploring Random Tree Star(RRT*) planner class
 * @author: Yang Haodong
 * @update: 2022-10-29
 * @version: 1.0
 *
 * Copyright (c) 2022， Yang Haodong
 * All rights reserved.
 * --------------------------------------------------------
 *
 **********************************************************/
#ifndef RRT_STAR_H
#define RRT_STAR_H

#include "rrt.h"
#include "utils.h"

namespace rrt_planner
{
/**
 * @brief Class for objects that plan using the RRT* algorithm
 */
class RRTStar : public RRT
{
public:
  /**
   * @brief  Constructor
   * @param   nx          pixel number in costmap x direction
   * @param   ny          pixel number in costmap y direction
   * @param   resolution  costmap resolution
   * @param   sample_num  andom sample points
   * @param   max_dist    max distance between sample points
   * @param   r           optimization radius
   */
  RRTStar(int nx, int ny, double resolution, int sample_num, double max_dist, double r);
  /**
   * @brief RRT implementation
   * @param costs     costmap
   * @param start     start node
   * @param goal      goal node
   * @param expand    containing the node been search during the process
   * @return tuple contatining a bool as to whether a path was found, and the path
   */
  bool plan(const unsigned char* gloal_costmap, const Node& start, const Node& goal, std::vector<Node>& path,
            std::vector<Node>& expand);

protected:
  /**
   * @brief Regular the new node by the nearest node in the sample list
   * @param list     sample list
   * @param node     sample node
   * @return nearest node
   */
  Node _findNearestPoint(std::unordered_set<Node, NodeIdAsHash, compare_coordinates> list, Node& node);

protected:
  double r_;  // optimization radius
};
}  // namespace rrt_planner
#endif  // RRT_H
