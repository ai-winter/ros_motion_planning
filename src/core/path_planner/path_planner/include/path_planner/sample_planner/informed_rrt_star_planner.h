/**
 * *********************************************************
 *
 * @file: informed_rrt_star_planner.h
 * @brief: Contains the informed RRT* planner class
 * @author: Yang Haodong
 * @date: 2024-09-24
 * @version: 2.0
 *
 * Copyright (c) 2024, Yang Haodong.
 * All rights reserved.
 *
 * --------------------------------------------------------
 *
 * ********************************************************
 */
#ifndef RMP_PATH_PLANNER_SAMPLE_PLANNER_INFORMED_RRT_STAR_H_
#define RMP_PATH_PLANNER_SAMPLE_PLANNER_INFORMED_RRT_STAR_H_

#include "path_planner/sample_planner/rrt_star_planner.h"

namespace rmp {
namespace path_planner {
/**
 * @brief Class for objects that plan using the RRT* algorithm
 */
class InformedRRTStarPathPlanner : public RRTStarPathPlanner {
private:
  using Node = rmp::common::structure::Node<int>;

public:
  /**
   * @brief  Constructor
   * @param   costmap   the environment for path planning
   */
  InformedRRTStarPathPlanner(costmap_2d::Costmap2DROS* costmap_ros);

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
   * @brief Sample in ellipse
   * @param   x   random sampling x
   * @param   y   random sampling y
   * @return ellipse node
   */
  Node _transform(double x, double y);

  /**
   * @brief Generates a random node
   * @return Generated node
   */
  Node _generateRandomNode();

protected:
  double c_best_;  // best planning cost
  double c_min_;   // distance between start and goal
};
}  // namespace path_planner
}  // namespace rmp
#endif
