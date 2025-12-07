/**
 * *********************************************************
 *
 * @file: lazy_theta_star_planner.h
 * @brief: Contains the lazy Theta* planner class
 * @author: Wu Maojia, Yang Haodong
 * @date: 2023-10-01
 * @version: 1.3
 *
 * Copyright (c) 2024, Wu Maojia, Yang Haodong.
 * All rights reserved.
 *
 * --------------------------------------------------------
 *
 * ********************************************************
 */
#ifndef RMP_PATH_PLANNER_GRAPH_PLANNER_LAZY_THETA_STAR_H
#define RMP_PATH_PLANNER_GRAPH_PLANNER_LAZY_THETA_STAR_H

#include "path_planner/graph_planner/theta_star_planner.h"

namespace rmp {
namespace path_planner {
/**
 * @brief Class for objects that plan using the lazy Theta* algorithm
 */
class LazyThetaStarPathPlanner : public ThetaStarPathPlanner {
private:
  using Node = rmp::common::structure::Node<int>;

public:
  /**
   * @brief Construct a new LaztThetaStar object
   * @param costmap   the environment for path planning
   */
  LazyThetaStarPathPlanner(costmap_2d::Costmap2DROS* costmap_ros);

  /**
   * @brief Lazy Theta* implementation
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
   */
  void _updateVertex(const Node& parent, Node& child);

  /**
   * @brief check if the parent of vertex need to be updated. if so, update it
   * @param node
   */
  void _setVertex(Node& node);

private:
  std::unordered_map<int, Node> closed_list_;  // closed list
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
