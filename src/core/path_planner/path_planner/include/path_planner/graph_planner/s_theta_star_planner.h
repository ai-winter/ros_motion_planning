/**
 * *********************************************************
 *
 * @file: s_theta_star_planner.h
 * @brief: Contains the S-Theta* planner class
 * @author: Wu Maojia
 * @date: 2024-3-9
 * @version: 1.0
 *
 * Copyright (c) 2024, Wu Maojia, Yang Haodong.
 * All rights reserved.
 *
 * --------------------------------------------------------
 *
 * ********************************************************
 */
#ifndef RMP_PATH_PLANNER_GRAPH_PLANNER_S_THETA_STAR_H_
#define RMP_PATH_PLANNER_GRAPH_PLANNER_S_THETA_STAR_H_

#include "path_planner/graph_planner/theta_star_planner.h"

namespace rmp {
namespace path_planner {
/**
 * @brief Class for objects that plan using the S-Theta* algorithm
 */
class SThetaStarPathPlanner : public ThetaStarPathPlanner {
private:
  using Node = rmp::common::structure::Node<int>;

public:
  /**
   * @brief Construct a new SThetaStar object
   * @param costmap   the environment for path planning
   */
  SThetaStarPathPlanner(costmap_2d::Costmap2DROS* costmap_ros);

  /**
   * @brief S-Theta* implementation
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
   * @brief update the g value of child node
   * @param parent
   * @param child
   * @param alpha
   */
  void _updateVertex(const Node& parent, Node& child, const double alpha);

  /**
   * @brief Get the deviation cost
   * @param parent
   * @param child
   * @param goal
   * @return deviation cost
   */
  double _alpha(const Node& parent, const Node& child, const Node& goal);

private:
  const double pi_ = std::acos(-1);  // pi
  const std::vector<Node> motions = {
    { 0, 1, 1.0 },           { 1, 0, 1.0 },
    { 0, -1, 1.0 },          { -1, 0, 1.0 },
    { 1, 1, std::sqrt(2) },  { 1, -1, std::sqrt(2) },
    { -1, 1, std::sqrt(2) }, { -1, -1, std::sqrt(2) },
  };
};
}  // namespace path_planner
}  // namespace rmp
#endif
