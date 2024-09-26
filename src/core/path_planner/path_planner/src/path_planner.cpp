/**
 * *********************************************************
 *
 * @file: global_PathPlanner.cpp
 * @brief: Contains the abstract global PathPlanner class
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
#include <costmap_2d/cost_values.h>

#include "path_planner/path_planner.h"

namespace rmp
{
namespace path_planner
{
/**
 * @brief Set or reset obstacle factor
 * @param factor obstacle factor
 */
void PathPlanner::setFactor(float factor)
{
  factor_ = factor;
}

/**
 * @brief get the costmap
 * @return costmap costmap2d pointer
 */
costmap_2d::Costmap2D* PathPlanner::getCostMap() const
{
  return costmap_;
}

/**
 * @brief get the size of costmap
 * @return map_size the size of costmap
 */
int PathPlanner::getMapSize() const
{
  return map_size_;
}

/**
 * @brief Transform from grid map(x, y) to grid index(i)
 * @param x grid map x
 * @param y grid map y
 * @return index
 */
int PathPlanner::grid2Index(int x, int y)
{
  return x + static_cast<int>(costmap_->getSizeInCellsX() * y);
}

/**
 * @brief Transform from grid index(i) to grid map(x, y)
 * @param i grid index i
 * @param x grid map x
 * @param y grid map y
 */
void PathPlanner::index2Grid(int i, int& x, int& y)
{
  x = static_cast<int>(i % costmap_->getSizeInCellsX());
  y = static_cast<int>(i / costmap_->getSizeInCellsX());
}

/**
 * @brief Tranform from world map(x, y) to costmap(x, y)
 * @param mx costmap x
 * @param my costmap y
 * @param wx world map x
 * @param wy world map y
 * @return true if successfull, else false
 */
bool PathPlanner::world2Map(double wx, double wy, double& mx, double& my)
{
  if (wx < costmap_->getOriginX() || wy < costmap_->getOriginY())
    return false;

  mx = (wx - costmap_->getOriginX()) / costmap_->getResolution();
  my = (wy - costmap_->getOriginY()) / costmap_->getResolution();

  if (mx < costmap_->getSizeInCellsX() && my < costmap_->getSizeInCellsY())
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
void PathPlanner::map2World(double mx, double my, double& wx, double& wy)
{
  wx = costmap_->getOriginX() + (mx + 0.5) * costmap_->getResolution();
  wy = costmap_->getOriginY() + (my + 0.5) * costmap_->getResolution();
}

/**
 * @brief Inflate the boundary of costmap into obstacles to prevent cross planning
 */
void PathPlanner::outlineMap()
{
  auto nx = costmap_->getSizeInCellsX();
  auto ny = costmap_->getSizeInCellsY();
  auto pc = costmap_->getCharMap();
  for (int i = 0; i < nx; i++)
    *pc++ = costmap_2d::LETHAL_OBSTACLE;
  pc = costmap_->getCharMap() + (ny - 1) * nx;
  for (int i = 0; i < nx; i++)
    *pc++ = costmap_2d::LETHAL_OBSTACLE;
  pc = costmap_->getCharMap();
  for (int i = 0; i < ny; i++, pc += nx)
    *pc = costmap_2d::LETHAL_OBSTACLE;
  pc = costmap_->getCharMap() + nx - 1;
  for (int i = 0; i < ny; i++, pc += nx)
    *pc = costmap_2d::LETHAL_OBSTACLE;
}
}  // namespace path_planner
}  // namespace rmp