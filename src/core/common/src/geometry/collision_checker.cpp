/***********************************************************
 *
 * @file: collision_checker.cpp
 * @breif: Collision Checker
 * @author: Yang Haodong
 * @update: 2024-10-07
 * @version: 1.0
 *
 * Copyright (c) 2024, Yang Haodong
 * All rights reserved.
 * --------------------------------------------------------
 *
 **********************************************************/
#include "common/geometry/collision_checker.h"

namespace rmp
{
namespace common
{
namespace geometry
{
/**
 * @brief Get the current costmap ROS wrapper object
 */
costmap_2d::Costmap2DROS* CollisionChecker::getCostmapROS() const
{
  return costmap_ros_;
}

/**
 * @brief grid map collision detection
 * @param i grid index
 * @param traverse_unknown Whether or not to traverse in unknown space
 * @return true if collision occurs, else false
 */
bool CollisionChecker::inCollision(const unsigned int& i, const bool& traverse_unknown)
{
  double center_cost = getCost(i);
  if (std::isinf(center_cost))
  {
    return true;
  }
  if (center_cost == costmap_2d::NO_INFORMATION && traverse_unknown)
  {
    return false;
  }
  // if occupied or unknown and not to traverse unknown space
  return center_cost >= costmap_2d::INSCRIBED_INFLATED_OBSTACLE * obstacle_factor_;
}

/**
 * @brief get cost value in costmap
 * @param i grid index
 * @return cost value
 */
float CollisionChecker::getCost(const unsigned int& i)
{
  if (isInsideMap(i))
  {
    return costmap_ros_->getCostmap()->getCharMap()[i];
  }
  return std::numeric_limits<float>::infinity();
}

/**
 * @brief Judge whether the grid is inside the map
 * @param i grid index
 * @return true if inside the map else false
 */
bool CollisionChecker::isInsideMap(const unsigned int& i)
{
  unsigned int size = static_cast<unsigned int>(costmap_ros_->getCostmap()->getSizeInCellsX() *
                                                costmap_ros_->getCostmap()->getSizeInCellsY());
  return ((i > 0) && (i < size));
}

}  // namespace geometry
}  // namespace common
}  // namespace rmp