/**
 * *********************************************************
 *
 * @file: rrt_star_planner.h
 * @brief: Contains the Rapidly-Exploring Random Tree Star(RRT*) planner class
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
#ifndef RMP_PATH_PLANNER_SAMPLE_PLANNER_RRT_STAR_H_
#define RMP_PATH_PLANNER_SAMPLE_PLANNER_RRT_STAR_H_

#include "path_planner/sample_planner/rrt_planner.h"

namespace rmp {
namespace path_planner {
/**
 * @brief Class for objects that plan using the RRT* algorithm
 */
class RRTStarPathPlanner : public RRTPathPlanner {
private:
  using Node = rmp::common::structure::Node<int>;

public:
  /**
   * @brief  Constructor
   * @param   costmap   the environment for path planning
   */
  RRTStarPathPlanner(costmap_2d::Costmap2DROS* costmap_ros);
  /**
   * @brief RRT implementation
   * @param start     start node
   * @param goal      goal node
   * @param expand    containing the node been search during the process
   * @return tuple contatining a bool as to whether a path was found, and the path
   */
  bool plan(const common::geometry::Point3d& start, const common::geometry::Point3d& goal,
            common::geometry::Points3d* path, common::geometry::Points3d* expand);

protected:
  /**
   * @brief Regular the new node by the nearest node in the sample list
   * @param list     sample list
   * @param node     sample node
   * @return nearest node
   */
  Node _findNearestPoint(std::unordered_map<int, Node>& list, Node& node);
};
}  // namespace path_planner
}  // namespace rmp
#endif  // RRT_H
