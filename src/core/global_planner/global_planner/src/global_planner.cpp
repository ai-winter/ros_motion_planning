/**
 * *********************************************************
 *
 * @file: global_planner.cpp
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
 * @brief Set or reset costmap origin
 * @param origin_x  origin in costmap x direction
 * @param origin_y  origin in costmap y direction
 */
void GlobalPlanner::setOrigin(double origin_x, double origin_y)
{
  origin_x_ = origin_x;
  origin_y_ = origin_y;
}

/**
 * @brief Set convert offset
 * @param origin_x  origin in costmap x direction
 * @param origin_y  origin in costmap y direction
 */
void GlobalPlanner::setConvertOffset(double convert_offset)
{
  convert_offset_ = convert_offset;
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
 * @brief Tranform from world map(x, y) to costmap(x, y)
 * @param mx costmap x
 * @param my costmap y
 * @param wx world map x
 * @param wy world map y
 * @return true if successfull, else false
 */
bool GlobalPlanner::world2Map(double wx, double wy, double& mx, double& my)
{
  if (wx < origin_x_ || wy < origin_y_)
    return false;

  mx = (wx - origin_x_) / resolution_ - convert_offset_;
  my = (wy - origin_y_) / resolution_ - convert_offset_;
  if (mx < nx_ && my < ny_)
    return true;

  return false;
}

/**
 * @brief Tranform from costmap(x, y) to world map(x, y)
 * @param mx costmap x
 * @param my costmap y
 * @param wx world map x
 * @param wy world map y
 */
void GlobalPlanner::map2World(double mx, double my, double& wx, double& wy)
{
  wx = origin_x_ + (mx + convert_offset_) * resolution_;
  wy = origin_y_ + (my + convert_offset_) * resolution_;
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
 * @brief Convert closed list to path
 * @param closed_list closed list
 * @param start       start node
 * @param goal        goal node
 * @return vector containing path nodes
 */
std::vector<Node> GlobalPlanner::_convertClosedListToPath(std::unordered_map<int, Node>& closed_list, const Node& start,
                                                          const Node& goal)
{
  std::vector<Node> path;
  auto current = closed_list.find(goal.id_);
  while (current->second != start)
  {
    path.emplace_back(current->second.x_, current->second.y_);
    auto it = closed_list.find(current->second.pid_);
    if (it != closed_list.end())
      current = it;
    else
      return {};
  }
  path.push_back(start);
  return path;
}

}  // namespace global_planner