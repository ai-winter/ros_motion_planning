/**
 * *********************************************************
 *
 * @file: path_planner.h
 * @brief: Contains the abstract global planner class
 * @author: Yang Haodong
 * @date: 2023-10-24
 * @version: 2.1
 *
 * Copyright (c) 2024, Yang Haodong.
 * All rights reserved.
 *
 * --------------------------------------------------------
 *
 * ********************************************************
 */
#ifndef RMP_PATH_PLANNER_PATH_PLANNER_H_
#define RMP_PATH_PLANNER_PATH_PLANNER_H_

#include <unordered_map>

#include <ros/ros.h>
#include <costmap_2d/costmap_2d.h>
#include <costmap_2d/costmap_2d_ros.h>

#include "common/structure/node.h"
#include "common/geometry/point.h"
#include "common/geometry/collision_checker.h"

namespace rmp
{
namespace path_planner
{
/**
 * @brief Abstract class that is inherited by concerete implementaions of global planner classes.
 *        The Plan function is a pure virtual funciton that is overloaded
 */
class PathPlanner
{
public:
  using Point2d = rmp::common::geometry::Point2d;
  using Point3d = rmp::common::geometry::Point3d;
  using Points2d = rmp::common::geometry::Points2d;
  using Points3d = rmp::common::geometry::Points3d;

public:
  /**
   * @brief Construct a new Global PathPlanner object
   * @param costmap_ros     the environment for path planning
   * @param obstacle_factor obstacle factor(greater means obstacles)
   */
  PathPlanner(costmap_2d::Costmap2DROS* costmap_ros, double obstacle_factor = 1.0);

  /**
   * @brief Destroy the Global PathPlanner object
   */
  virtual ~PathPlanner() = default;

  /**
   * @brief Pure virtual function that is overloadde by planner implementations
   * @param start          start node
   * @param goal           goal node
   * @param path           optimal path consists of Node
   * @param expand         containing the node been search during the process
   * @return true if path found, else false
   */
  virtual bool plan(const Point3d& start, const Point3d& goal, Points3d& path, Points3d& expand) = 0;

  /**
   * @brief Set or reset obstacle factor
   * @param factor obstacle factor
   */
  void setFactor(float factor);

  /**
   * @brief get the costmap
   * @return costmap costmap2d pointer
   */
  costmap_2d::Costmap2D* getCostMap() const;

  /**
   * @brief get the size of costmap
   * @return map_size the size of costmap
   */
  int getMapSize() const;

  /**
   * @brief Transform from grid map(x, y) to grid index(i)
   * @param x grid map x
   * @param y grid map y
   * @return index
   */
  int grid2Index(int x, int y);

  /**
   * @brief Transform from grid index(i) to grid map(x, y)
   * @param i grid index i
   * @param x grid map x
   * @param y grid map y
   */
  void index2Grid(int i, int& x, int& y);

  /**
   * @brief Tranform from world map(x, y) to costmap(x, y)
   * @param mx costmap x
   * @param my costmap y
   * @param wx world map x
   * @param wy world map y
   * @return true if successfull, else false
   */
  bool world2Map(double wx, double wy, double& mx, double& my);

  /**
   * @brief Tranform from costmap(x, y) to world map(x, y)
   * @param mx costmap x
   * @param my costmap y
   * @param wx world map x
   * @param wy world map y
   */
  void map2World(double mx, double my, double& wx, double& wy);

  /**
   * @brief Inflate the boundary of costmap into obstacles to prevent cross planning
   */
  void outlineMap();

  /**
   * @brief Check the validity of (wx, wy)
   * @param wx world map x
   * @param wy world map y
   * @param mx costmap x
   * @param my costmap y
   * @return flag true if the position is valid
   */
  bool validityCheck(double wx, double wy, double& mx, double& my);

protected:
  /**
   * @brief Convert closed list to path
   * @param closed_list closed list
   * @param start       start node
   * @param goal        goal node
   * @return vector containing path nodes
   */
  template <typename Node>
  std::vector<Node> _convertClosedListToPath(std::unordered_map<int, Node>& closed_list, const Node& start,
                                             const Node& goal)
  {
    std::vector<Node> path;
    auto current = closed_list.find(goal.id());
    while (current->second != start)
    {
      path.emplace_back(current->second.x(), current->second.y());
      auto it = closed_list.find(current->second.pid());
      if (it != closed_list.end())
        current = it;
      else
        return {};
    }
    path.push_back(start);
    return path;
  }

  template <typename Node>
  std::vector<Node> _convertBiClosedListToPath(std::unordered_map<int, Node>& f_closed_list,
                                               std::unordered_map<int, Node>& b_closed_list, const Node& start,
                                               const Node& goal, const Node& boundary)
  {
    if (f_closed_list.find(start.id()) == f_closed_list.end())
      std::swap(f_closed_list, b_closed_list);

    std::vector<Node> path, path_b;

    // backward
    auto current = b_closed_list.find(boundary.id());
    while (current->second != goal)
    {
      path_b.push_back(current->second);
      auto it = b_closed_list.find(current->second.pid());
      if (it != b_closed_list.end())
        current = it;
      else
        return {};
    }
    path_b.push_back(goal);

    // forward
    for (auto rit = path_b.rbegin(); rit != path_b.rend(); rit++)
      path.push_back(*rit);

    current = f_closed_list.find(boundary.id());
    while (current->second != start)
    {
      auto it = f_closed_list.find(current->second.pid());
      if (it != f_closed_list.end())
        current = it;
      else
        return {};
      path.push_back(current->second);
    }

    return path;
  }

protected:
  float obstacle_factor_;                  // obstacle factor(greater means obstacles)
  int nx_, ny_, map_size_;                 // pixel number in costmap
  costmap_2d::Costmap2DROS* costmap_ros_;  // costmap ROS wrapper
  costmap_2d::Costmap2D* costmap_;         // costmap buffer
  std::shared_ptr<rmp::common::geometry::CollisionChecker> collision_checker_;  // gridmap collision checker
};
}  // namespace path_planner
}  // namespace rmp
#endif  // PLANNER_HPP