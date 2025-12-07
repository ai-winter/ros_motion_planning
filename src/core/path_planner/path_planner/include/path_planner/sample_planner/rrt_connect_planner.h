/**
 * *********************************************************
 *
 * @file: rrt_connect_planner.h
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
#ifndef RMP_PATH_PLANNER_SAMPLE_PLANNER_RRT_CONNECT_H_
#define RMP_PATH_PLANNER_SAMPLE_PLANNER_RRT_CONNECT_H_

#include "path_planner/sample_planner/rrt_planner.h"

namespace rmp {
namespace path_planner {
/**
 * @brief Class for objects that plan using the RRT Connect algorithm
 */
class RRTConnectPathPlanner : public RRTPathPlanner {
private:
  using Node = rmp::common::structure::Node<int>;

public:
  /**
   * @brief  Constructor
   * @param   costmap   the environment for path planning
   */
  RRTConnectPathPlanner(costmap_2d::Costmap2DROS* costmap_ros);

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
  // Sampled list forward
  std::unordered_map<int, Node> sample_list_f_;
  // Sampled list backward
  std::unordered_map<int, Node> sample_list_b_;
};
}  // namespace path_planner
}  // namespace rmp

#endif