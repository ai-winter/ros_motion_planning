/**
 * *********************************************************
 *
 * @file: quick_informed_rrt_star_planner.h
 * @brief: Contains the quick informed RRT* planner class
 * @author: Honbo He, Haodong Yang
 * @date: 2024-06-05
 * @version: 1.0
 *
 * Copyright (c) 2024, Yang Haodong.
 * All rights reserved.
 *
 * --------------------------------------------------------
 *
 * ********************************************************
 */
#ifndef RMP_PATH_PLANNER_SAMPLE_PLANNER_QUICK_INFORMED_RRT_H_
#define RMP_PATH_PLANNER_SAMPLE_PLANNER_QUICK_INFORMED_RRT_H_

#include "path_planner/sample_planner/informed_rrt_star_planner.h"

namespace rmp
{
namespace path_planner
{
/**
 * @brief Class for objects that plan using the Informed RRT* algorithm
 */
class QuickInformedRRTStarPathPlanner : public InformedRRTStarPathPlanner
{
private:
  using Node = rmp::common::structure::Node<int>;

public:
  /**
   * @brief  Constructor
   * @param   costmap   the environment for path planning
   * @param   obstacle_factor obstacle factor(greater means obstacles)
   * @param   sample_num  andom sample points
   * @param   max_dist    max distance between sample points
   * @param   r           optimization radius
   */
  QuickInformedRRTStarPathPlanner(costmap_2d::Costmap2DROS* costmap_ros, double obstacle_factor, int sample_num,
                                  double max_dist, double r, double r_set, int n_threads, double d_extend,
                                  double t_freedom);

  /**
   * @brief RRT implementation
   * @param start     start node
   * @param goal      goal node
   * @param expand    containing the node been search during the process
   * @return tuple contatining a bool as to whether a path was found, and the path
   */
  bool plan(const Point3d& start, const Point3d& goal, Points3d& path, Points3d& expand);

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
}  // namespace path_planner
}  // namespace rmp
#endif
