/***********************************************************
 *
 * @file: global_planner.cpp
 * @breif: Contains some implement of global planner class
 * @author: Yang Haodong
 * @update: 2022-10-24
 * @version: 2.1
 *
 * Copyright (c) 2023， Yang Haodong
 * All rights reserved.
 * --------------------------------------------------------
 *
 **********************************************************/
#include "global_planner.h"

namespace global_planner
{
/**
 * @brief Construct a new Global Planner object
 * @param nx         pixel number in costmap x direction
 * @param ny         pixel number in costmap y direction
 * @param resolution costmap resolution
 */
GlobalPlanner::GlobalPlanner(int nx, int ny, double resolution)
  : lethal_cost_(LETHAL_COST), neutral_cost_(NEUTRAL_COST), factor_(OBSTACLE_FACTOR)
{
  setSize(nx, ny);
  setResolution(resolution);
}

/**
 * @brief Set or reset costmap size
 * @param nx pixel number in costmap x direction
 * @param ny pixel number in costmap y direction
 */
void GlobalPlanner::setSize(int nx, int ny)
{
  nx_ = nx;
  ny_ = ny;
  ns_ = nx * ny;
}

/**
 * @brief Set or reset costmap resolution
 * @param resolution costmap resolution
 */
void GlobalPlanner::setResolution(double resolution)
{
  resolution_ = resolution;
}

void GlobalPlanner::setLethalCost(unsigned char lethal_cost)
{
  lethal_cost_ = lethal_cost;
}

/**
 * @brief Set or reset neutral cost
 * @param neutral_cost neutral cost
 */
void GlobalPlanner::setNeutralCost(unsigned char neutral_cost)
{
  neutral_cost_ = neutral_cost;
}

/**
 * @brief Set or reset obstacle factor
 * @param factor obstacle factor
 */
void GlobalPlanner::setFactor(double factor)
{
  factor_ = factor;
}

/**
 * @brief Transform from grid map(x, y) to grid index(i)
 * @param x grid map x
 * @param y grid map y
 * @return index
 */
int GlobalPlanner::grid2Index(int x, int y)
{
  return x + nx_ * y;
}

/**
 * @brief Transform from grid index(i) to grid map(x, y)
 * @param i grid index i
 * @param x grid map x
 * @param y grid map y
 */
void GlobalPlanner::index2Grid(int i, int& x, int& y)
{
  x = i % nx_;
  y = i / nx_;
}

/**
 * @brief Transform from grid map(x, y) to costmap(x, y)
 * @param gx grid map x
 * @param gy grid map y
 * @param mx costmap x
 * @param my costmap y
 */
void GlobalPlanner::map2Grid(double mx, double my, int& gx, int& gy)
{
  gx = (int)mx;
  gy = (int)my;
}

/**
 * @brief Transform from costmap(x, y) to grid map(x, y)
 * @param gx grid map x
 * @param gy grid map y
 * @param mx costmap x
 * @param my costmap y
 */
void GlobalPlanner::grid2Map(int gx, int gy, double& mx, double& my)
{
  mx = resolution_ * (gx + 0.5);
  my = resolution_ * (gy + 0.5);
}

/**
 * @brief Get permissible motion
 * @return Node vector of permissible motions
 */
std::vector<Node> GlobalPlanner::getMotion()
{
  return {
    Node(0, 1, 1),
    Node(1, 0, 1),
    Node(0, -1, 1),
    Node(-1, 0, 1),
    Node(1, 1, std::sqrt(2)),
    Node(1, -1, std::sqrt(2)),
    Node(-1, 1, std::sqrt(2)),
    Node(-1, -1, std::sqrt(2)),
  };
}

/**
 * @brief Inflate the boundary of costmap into obstacles to prevent cross planning
 * @param costarr costmap pointer
 */
void GlobalPlanner::outlineMap(unsigned char* costarr)
{
  unsigned char* pc = costarr;
  for (int i = 0; i < nx_; i++)
    *pc++ = costmap_2d::LETHAL_OBSTACLE;
  pc = costarr + (ny_ - 1) * nx_;
  for (int i = 0; i < nx_; i++)
    *pc++ = costmap_2d::LETHAL_OBSTACLE;
  pc = costarr;
  for (int i = 0; i < ny_; i++, pc += nx_)
    *pc = costmap_2d::LETHAL_OBSTACLE;
  pc = costarr + nx_ - 1;
  for (int i = 0; i < ny_; i++, pc += nx_)
    *pc = costmap_2d::LETHAL_OBSTACLE;
}

/**
 * @brief Calculate distance between the 2 nodes.
 * @param n1 Node 1
 * @param n2 Node 2
 * @return distance between nodes
 */
double GlobalPlanner::dist(const Node& node1, const Node& node2)
{
  return std::hypot(node1.x_ - node2.x_, node1.y_ - node2.y_);
}

/**
 * @brief Calculate the angle of x-axis between the 2 nodes.
 * @param n1 Node 1
 * @param n2 Node 2
 * @return the angle of x-axis between the 2 node
 */
double GlobalPlanner::angle(const Node& node1, const Node& node2)
{
  return atan2(node2.y_ - node1.y_, node2.x_ - node1.x_);
}

/**
 * @brief Convert closed list to path
 * @param closed_list closed list
 * @param start       start node
 * @param goal        goal node
 * @return vector containing path nodes
 */
std::vector<Node> GlobalPlanner::_convertClosedListToPath(
    std::unordered_set<Node, NodeIdAsHash, compare_coordinates>& closed_list, const Node& start, const Node& goal)
{
  auto current = *closed_list.find(goal);

  std::vector<Node> path;
  while (current != start)
  {
    path.push_back(current);
    auto it = closed_list.find(Node(current.pid_ % nx_, current.pid_ / nx_, 0, 0, current.pid_));
    if (it != closed_list.end())
      current = *it;
    else
      return {};
  }
  path.push_back(start);

  return path;
}

}  // namespace global_planner