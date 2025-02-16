/**
 * *********************************************************
 *
 * @file: rrt_planner.h
 * @brief: Contains the Rapidly-Exploring Random Tree (RRT) planner class
 * @author: Yang Haodong
 * @date: 2024-9-24
 * @version: 2.0
 *
 * Copyright (c) 2024, Yang Haodong.
 * All rights reserved.
 *
 * --------------------------------------------------------
 *
 * ********************************************************
 */
#ifndef RMP_PATH_PLANNER_SAMPLE_PLANNER_RRT_H_
#define RMP_PATH_PLANNER_SAMPLE_PLANNER_RRT_H_

#include "path_planner/path_planner.h"

namespace rmp
{
namespace path_planner
{
/**
 * @brief Class for objects that plan using the RRT algorithm
 */
class RRTPathPlanner : public PathPlanner
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
   */
  RRTPathPlanner(costmap_2d::Costmap2DROS* costmap_ros, double obstacle_factor, int sample_num, double max_dist);

  /**
   * @brief RRT implementation
   * @param start         start node
   * @param goal          goal node
   * @param path          optimal path consists of Node
   * @param expand        containing the node been search during the process
   * @return  true if path found, else false
   */
  bool plan(const Point3d& start, const Point3d& goal, Points3d& path, Points3d& expand);

protected:
  /**
   * @brief Regular the sample node by the nearest node in the sample list
   * @param list  samplee list
   * @param node  sample node
   * @return nearest node
   */
  Node _findNearestPoint(std::unordered_map<int, Node>& list, const Node& node);
  /**
   * @brief Generates a random node
   * @return Generated node
   */
  Node _generateRandomNode();
  /**
   * @brief Check if goal is reachable from current node
   * @param new_node Current node
   * @return bool value of whether goal is reachable from current node
   */
  bool _checkGoal(const Node& new_node);

protected:
  Node start_, goal_;                          // start and goal node copy
  std::unordered_map<int, Node> sample_list_;  // set of sample nodes
  int sample_num_;                             // max sample number
  double max_dist_;                            // max distance threshold
  double opti_sample_p_ = 0.05;                // optimized sample probability, default to 0.05
};
}  // namespace path_planner
}  // namespace rmp
#endif  // RRT_H
