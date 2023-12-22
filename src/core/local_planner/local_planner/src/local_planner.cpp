/***********************************************************
 *
 * @file: local_planner.cpp
 * @breif: Contains some implement of local planner class
 * @author: Yang Haodong
 * @update: 2023-10-2
 * @version: 1.0
 *
 * Copyright (c) 2023， Yang Haodong
 * All rights reserved.
 * --------------------------------------------------------
 *
 **********************************************************/
#include <tf2/utils.h>

#include "local_planner.h"

namespace local_planner
{
/**
 * @brief Construct a new Local Planner object
 */
LocalPlanner::LocalPlanner()
  : lethal_cost_(LETHAL_COST)
  , neutral_cost_(NEUTRAL_COST)
  , factor_(OBSTACLE_FACTOR)
  , base_frame_("base_link")
  , map_frame_("map")
  , convert_offset_(0.0)
{
}

/**
 * @brief Set or reset costmap size
 * @param nx  pixel number in costmap x direction
 * @param ny  pixel number in costmap y direction
 */
void LocalPlanner::setSize(int nx, int ny)
{
  nx_ = nx;
  ny_ = ny;
  ns_ = nx * ny;
}

/**
 * @brief Set or reset costmap resolution
 * @param resolution  costmap resolution
 */
void LocalPlanner::setResolution(double resolution)
{
  resolution_ = resolution;
}

/**
 * @brief Set or reset costmap origin
 * @param origin_x  origin in costmap x direction
 * @param origin_y  origin in costmap y direction
 */
void LocalPlanner::setOrigin(double origin_x, double origin_y)
{
  origin_x_ = origin_x;
  origin_y_ = origin_y;
}

/**
 * @brief Set or reset lethal cost
 * @param neutral_cost  neutral cost
 */
void LocalPlanner::setLethalCost(unsigned char lethal_cost)
{
  lethal_cost_ = lethal_cost;
}

/**
 * @brief Set or reset neutral cost
 * @param neutral_cost  neutral cost
 */
void LocalPlanner::setNeutralCost(unsigned char neutral_cost)
{
  neutral_cost_ = neutral_cost;
}

/**
 * @brief Set or reset obstacle factor
 * @param factor  obstacle factor
 */
void LocalPlanner::setFactor(double factor)
{
  factor_ = factor;
}

/**
 * @brief Set or reset frame name
 * @param frame_name
 */
void LocalPlanner::setBaseFrame(std::string base_frame)
{
  base_frame_ = base_frame;
}
void LocalPlanner::setMapFrame(std::string map_frame)
{
  map_frame_ = map_frame;
}

/**
 * @brief Calculate distance between the 2 points.
 * @param n1        point 1
 * @param n2        point 2
 * @return distance between points
 */
double LocalPlanner::dist(const Eigen::Vector2d& point1, const Eigen::Vector2d& point2)
{
  return std::hypot(point1.x() - point2.x(), point1.y() - point2.y());
}

/**
 * @brief Regularize angle to [-pi, pi]
 * @param angle the angle (rad) to regularize
 */
void LocalPlanner::regularizeAngle(double& angle)
{
  angle = angle - 2.0 * M_PI * std::floor((angle + M_PI) / (2.0 * M_PI));
}

/**
 * @brief Get the Euler Angles from PoseStamped
 * @param ps  PoseStamped to calculate
 * @return  roll, pitch and yaw in XYZ order
 */
Eigen::Vector3d LocalPlanner::getEulerAngles(geometry_msgs::PoseStamped& ps)
{
  tf2::Quaternion q(ps.pose.orientation.x, ps.pose.orientation.y, ps.pose.orientation.z, ps.pose.orientation.w);
  tf2::Matrix3x3 m(q);

  double roll(0.0), pitch(0.0), yaw(0.0);
  m.getRPY(roll, pitch, yaw);

  return Eigen::Vector3d(roll, pitch, yaw);
}

/**
 * @brief Tranform from world map(x, y) to costmap(x, y)
 * @param mx  costmap x
 * @param my  costmap y
 * @param wx  world map x
 * @param wy  world map y
 * @return true if successfull, else false
 */
bool LocalPlanner::worldToMap(double wx, double wy, int& mx, int& my)
{
  if (wx < origin_x_ || wy < origin_y_)
    return false;

  mx = (int)((wx - origin_x_) / resolution_ - convert_offset_);
  my = (int)((wy - origin_y_) / resolution_ - convert_offset_);
  if (mx < nx_ && my < ny_)
    return true;

  return false;
}
}  // namespace local_planner