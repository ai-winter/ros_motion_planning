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
 * @brief Class for objects that plan using the Informed RRT* algorithm
 */
class QuickInformedRRT : public InformedRRT
{
public:
  /**
   * @brief  Constructor
   * @param   costmap   the environment for path planning
   * @param   sample_num  andom sample points
   * @param   max_dist    max distance between sample points
   * @param   r           optimization radius
   */
  QuickInformedRRT(costmap_2d::Costmap2D* costmap, int sample_num, double max_dist, double r, double r_set, int n_threads, double d_extend, double t_freedom);

  /**
   * @brief RRT implementation
   * @param start     start node
   * @param goal      goal node
   * @param expand    containing the node been search during the process
   * @return tuple contatining a bool as to whether a path was found, and the path
   */
  bool plan(const Node& start, const Node& goal, std::vector<Node>& path, std::vector<Node>& expand);

protected:
  /**
   * @brief Generates a random node
   * @return Generated node
   * @param mu
   * @param path
   */
  Node _generateRandomNode(int mu, std::vector<Node> path);

  /**
   * @brief Regular the new node by the nearest node in the sample list
   * @param list     sample list
   * @param node     sample node
   * @return nearest node
   */
  Node _findNearestPoint(std::unordered_map<int, Node> list, Node& node);

protected:
  double set_r_;             // radius of priority circles set
  int rewire_threads_;       // parallel rewire process
  double step_extend_d_;     // increased distance of adaptive extend step size
  double recover_max_dist_;  // recover value of max_dist_ when met obstacle
  double t_distr_freedom_;   // freedom of t distribution
};
}  // namespace global_planner
#endif
