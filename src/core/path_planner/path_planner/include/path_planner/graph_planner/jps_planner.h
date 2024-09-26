/**
 * *********************************************************
 *
 * @file: jps_planner.h
 * @brief: Contains the Jump Point Search(JPS) planner class
 * @author: Yang Haodong
 * @date: 2023-12-14
 * @version: 1.1
 *
 * Copyright (c) 2024, Yang Haodong.
 * All rights reserved.
 *
 * --------------------------------------------------------
 *
 * ********************************************************
 */
#ifndef RMP_PATH_PLANNER_GRAPH_PLANNER_JUMP_POINT_SEARCH_H_
#define RMP_PATH_PLANNER_GRAPH_PLANNER_JUMP_POINT_SEARCH_H_

#include "path_planner/path_planner.h"

namespace rmp
{
namespace path_planner
{
/**
 * @brief lass for objects that plan using the Jump Point Search(JPSPlanner) algorithm
 */
class JPSPathPlanner : public PathPlanner
{
private:
  using Node = rmp::common::structure::Node<int>;

public:
  /**
   * @brief Constructor
   * @param costmap   the environment for path planning
   */
  JPSPathPlanner(costmap_2d::Costmap2DROS* costmap_ros);

  /**
   * @brief Jump Point Search(JPS) implementation
   * @param start  start node
   * @param goal   goal node
   * @param expand containing the node been search during the process
   * @return tuple contatining a bool as to whether a path was found, and the path
   */
  bool plan(const Point3d& start, const Point3d& goal, Points3d& path, Points3d& expand);

  /**
   * @brief Calculate jump node recursively
   * @param point  current node
   * @param motion the motion that current node executes
   * @return jump node
   */
  Node jump(const Node& point, const Node& motion);

  /**
   * @brief Detect whether current node has forced neighbor or not
   * @param point  current node
   * @param motion the motion that current node executes
   * @return true if current node has forced neighbor else false
   */
  bool detectForceNeighbor(const Node& point, const Node& motion);

private:
  Node start_, goal_;  // start and goal node
  const std::vector<Node> motions = {
    { 0, 1, 1.0 },          { 1, 0, 1.0 },           { 0, -1, 1.0 },          { -1, 0, 1.0 },
    { 1, 1, std::sqrt(2) }, { 1, -1, std::sqrt(2) }, { -1, 1, std::sqrt(2) }, { -1, -1, std::sqrt(2) },
  };
};
}  // namespace path_planner
}  // namespace rmp
#endif  // JUMP_POINT_SEARCH_H
