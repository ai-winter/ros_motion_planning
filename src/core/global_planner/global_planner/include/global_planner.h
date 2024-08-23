/**
 * *********************************************************
 *
 * @file: global_planner.h
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
#ifndef GLOBAL_PLANNER_H
#define GLOBAL_PLANNER_H

#include <costmap_2d/cost_values.h>
#include <costmap_2d/costmap_2d.h>
#include <unordered_set>
#include <unordered_map>

#include "math_helper.h"

namespace global_planner
{
/**
 * @brief Abstract class that is inherited by concerete implementaions of global planner classes.
 *        The Plan function is a pure virtual funciton that is overloaded
 */
class GlobalPlanner
{
public:
  /**
   * @brief Construct a new Global Planner object
   * @param costmap the environment for path planning
   */
  GlobalPlanner(costmap_2d::Costmap2D* costmap);

  /**
   * @brief Destroy the Global Planner object
   */
  virtual ~GlobalPlanner() = default;

  /**
   * @brief Pure virtual function that is overloadde by planner implementations
   * @param start  start node
   * @param goal   goal node
   * @param path   optimal path consists of Node
   * @param expand containing the node been search during the process
   * @return true if path found, else false
   */
  virtual bool plan(const Node& start, const Node& goal, std::vector<Node>& path, std::vector<Node>& expand) = 0;

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
  bool world2Map(double wx, double wy, unsigned int& mx, unsigned int& my);

  /**
   * @brief Tranform from costmap(x, y) to world map(x, y)
   * @param mx costmap x
   * @param my costmap y
   * @param wx world map x
   * @param wy world map y
   */
  void map2World(unsigned int mx, unsigned int my, double& wx, double& wy);

  /**
   * @brief Inflate the boundary of costmap into obstacles to prevent cross planning
   */
  void outlineMap();

protected:
  /**
   * @brief Convert closed list to path
   * @param closed_list closed list
   * @param start       start node
   * @param goal        goal node
   * @return vector containing path nodes
   */
  std::vector<Node> _convertClosedListToPath(std::unordered_map<int, Node>& closed_list, const Node& start,
                                             const Node& goal);

  int map_size_;                    // pixel number in costmap
  float factor_;                    // obstacle factor(greater means obstacles)
  costmap_2d::Costmap2D* costmap_;  // costmap buffer
};
}  // namespace global_planner
#endif  // PLANNER_HPP