/**
 * *********************************************************
 *
 * @file: quick_informed_rrt.h
 * @brief: Contains the quick informed RRT* planner class
 * @author: Honbo He
 * @date: 2024-06-05
 * @version: 0.9
 *
 * Copyright (c) 2024, Yang Haodong.
 * All rights reserved.
 *
 * --------------------------------------------------------
 *
 * ********************************************************
 */
#ifndef QUICK_INFORMED_RRT_H
#define QUICK_INFORMED_RRT_H

#include "informed_rrt.h"

namespace global_planner
{
/**
 * @brief Class for objects that plan using the quick informed RRT* algorithm
 */
class QuickInformedRRT : public InformedRRT
{
public:
  /**
   * @brief Construct a quick informed new RRTStar object
   * @param costmap    the environment for path planning
   * @param sample_num andom sample points
   * @param max_dist   max distance between sample points
   * @param r          optimization radius
   * @param r_set      radius of priority circles set
   * @param n_threads  parallel rewire process
   * @param d_extend   increased distance of adaptive extend step size
   * @param t_freedom  freedom of t distribution
   */
  QuickInformedRRT(costmap_2d::Costmap2D* costmap, int sample_num, double max_dist, double r, double r_set,
                   int n_threads, double d_extend, double t_freedom);

  /**
   * @brief Quick informed RRT star implementation
   * @param start  start node
   * @param goal   goal node
   * @param expand containing the node been search during the process
   * @return true if path found, else false
   */
  bool plan(const Node& start, const Node& goal, std::vector<Node>& path, std::vector<Node>& expand);

protected:
  /**
   * @brief Generates a random node
   * @return generated node
   * @param mu
   * @param path
   */
  Node _generateRandomNode(int mu, std::vector<Node> path);

  /**
   * @brief Regular the new node by the nearest node in the sample list
   * @param list sample list
   * @param node sample node
   * @return nearest node
   */
  Node _findNearestPoint(std::unordered_map<int, Node> list, Node& node);

protected:
  int n_threads_;     // parallel rewire process
  double r_set_;      // radius of priority circles set
  double d_extend_;   // increased distance of adaptive extend step size
  double max_dist_;   // recover value of max_dist_ when met obstacle
  double t_freedom_;  // freedom of t distribution
};
}  // namespace global_planner
#endif  // QUICK_INFORMED_RRT_H
