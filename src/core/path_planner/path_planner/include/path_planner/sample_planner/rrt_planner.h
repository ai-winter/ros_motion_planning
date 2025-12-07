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

namespace rmp {
namespace path_planner {
/**
 * @brief Class for objects that plan using the RRT algorithm
 */
class RRTPathPlanner : public PathPlanner {
private:
  using Node = rmp::common::structure::Node<int>;

public:
  /**
   * @brief  Constructor
   * @param   costmap   the environment for path planning
   */
  RRTPathPlanner(costmap_2d::Costmap2DROS* costmap_ros);

  /**
   * @brief RRT implementation
   * @param start         start node
   * @param goal          goal node
   * @param path          The resulting path in (x, y, theta)
   * @param expand        containing the node been search during the process
   * @return  true if path found, else false
   */
  bool plan(const common::geometry::Point3d& start, const common::geometry::Point3d& goal,
            common::geometry::Points3d* path, common::geometry::Points3d* expand);

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
};
}  // namespace path_planner
}  // namespace rmp
#endif  // RRT_H
