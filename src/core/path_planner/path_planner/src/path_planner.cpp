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

using namespace rmp::common::geometry;

namespace rmp
{
namespace path_planner
{
/**
 * @brief Construct a new Global PathPlanner object
 * @param costmap_ros     the environment for path planning
 * @param obstacle_factor obstacle factor(greater means obstacles)
 */
PathPlanner::PathPlanner(costmap_2d::Costmap2DROS* costmap_ros, double obstacle_factor)
  : obstacle_factor_(obstacle_factor)
  , nx_(static_cast<int>(costmap_ros->getCostmap()->getSizeInCellsX()))
  , ny_(static_cast<int>(costmap_ros->getCostmap()->getSizeInCellsY()))
  , map_size_(nx_ * ny_)
  , costmap_ros_(costmap_ros)
  , costmap_(costmap_ros->getCostmap())
  , collision_checker_(std::make_shared<CollisionChecker>(costmap_ros, obstacle_factor))
{
}

/**
 * @brief Set or reset obstacle factor
 * @param obstacle_factor obstacle factor
 */
void PathPlanner::setFactor(float obstacle_factor)
{
  obstacle_factor_ = obstacle_factor;
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
  return x + nx_ * y;
}

/**
 * @brief Transform from grid index(i) to grid map(x, y)
 * @param i grid index i
 * @param x grid map x
 * @param y grid map y
 */
void PathPlanner::index2Grid(int i, int& x, int& y)
{
  x = static_cast<int>(i % nx_);
  y = static_cast<int>(i / nx_);
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
  auto pc = costmap_->getCharMap();
  for (int i = 0; i < nx_; i++)
    *pc++ = costmap_2d::LETHAL_OBSTACLE;
  pc = costmap_->getCharMap() + (ny_ - 1) * nx_;
  for (int i = 0; i < nx_; i++)
    *pc++ = costmap_2d::LETHAL_OBSTACLE;
  pc = costmap_->getCharMap();
  for (int i = 0; i < ny_; i++, pc += nx_)
    *pc = costmap_2d::LETHAL_OBSTACLE;
  pc = costmap_->getCharMap() + nx_ - 1;
  for (int i = 0; i < ny_; i++, pc += nx_)
    *pc = costmap_2d::LETHAL_OBSTACLE;
}

/**
 * @brief Check the validity of (wx, wy)
 * @param wx world map x
 * @param wy world map y
 * @param mx costmap x
 * @param my costmap y
 * @return flag true if the position is valid
 */
bool PathPlanner::validityCheck(double wx, double wy, double& mx, double& my)
{
  if (!world2Map(wx, wy, mx, my))
  {
    R_WARN << "The robot's position is off the global costmap. Planning will always fail, are you sure the robot "
              "has been properly localized?";
    return false;
  }
  return true;
}
}  // namespace path_planner
}  // namespace rmp